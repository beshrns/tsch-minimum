/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include <string.h>

#include "contiki.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

#include "dev/leds.h"
#include "dev/spi.h"
#include "dev/cc2420.h"
#include "dev/cc2420_const.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "net/mac/frame802154.h"
#include "lib/memb.h"
#include "lib/list.h"

#include "cooja-debug.h"
#include "tsch-parameters.h"

volatile int need_flush=0;

#define RSSI_THR -14

#define WITH_SEND_CCA 0

#define FOOTER_LEN 2
//Timesync IE length with header = 4
//#ifndef EXTRA_ACK_LEN
#define EXTRA_ACK_LEN 4
//#endif
#define ACK_LEN (3)

#define FIFOP_THRESHOLD (ACK_LEN+EXTRA_ACK_LEN-1)
#define CC2420_CONF_AUTOACK 0

#ifndef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL 26
#endif /* CC2420_CONF_CHANNEL */

#ifndef CC2420_CONF_CCA_THRESH
#define CC2420_CONF_CCA_THRESH (RSSI_THR-32)
#endif /* CC2420_CONF_CCA_THRESH */


#ifndef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK 0
#endif /* CC2420_CONF_AUTOACK */

#if CC2420_CONF_CHECKSUM
#include "lib/crc16.h"
#define CHECKSUM_LEN 2
#else
#define CHECKSUM_LEN 0
#endif /* CC2420_CONF_CHECKSUM */

#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LEDS DEBUG
#undef LEDS_ON
#undef LEDS_OFF
#if DEBUG_LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

void cc2420_arch_init(void);

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t cc2420_time_of_arrival, cc2420_time_of_departure;

int cc2420_authority_level_of_sender;

int cc2420_packets_seen, cc2420_packets_read;

static uint8_t volatile pending;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

volatile uint8_t cc2420_sfd_counter;
volatile uint16_t cc2420_sfd_start_time;
volatile uint16_t cc2420_sfd_end_time;

static volatile uint16_t last_packet_timestamp = 0xffff;
/*---------------------------------------------------------------------------*/
PROCESS(cc2420_process, "CC2420 driver");
/*---------------------------------------------------------------------------*/
/* Read a byte from RAM in the CC2420 */
#define CC2420_READ_RAM_BYTE(var,adr)                    \
  do {                                                       \
    CC2420_SPI_ENABLE();                                     \
    SPI_WRITE(0x80 | ((adr) & 0x7f));                        \
    SPI_WRITE((((adr) >> 1) & 0xc0) | 0x20);                 \
    SPI_RXBUF;                                               \
    SPI_READ((var));                    \
    CC2420_SPI_DISABLE();                                    \
  } while(0)

int cc2420_on(void);
int cc2420_off(void);

static int cc2420_read(void *buf, unsigned short bufsize);

static int cc2420_prepare(const void *data, unsigned short len);
static int cc2420_transmit(unsigned short len);
static int cc2420_send(const void *data, unsigned short len);

static int cc2420_receiving_packet(void);
static int pending_packet(void);
static int cc2420_cca(void);
/*static int detected_energy(void);*/

signed char cc2420_last_rssi;
uint8_t cc2420_last_correlation;

const struct radio_driver cc2420_driver =
  {
    cc2420_init,
    cc2420_prepare,
    cc2420_transmit,
    cc2420_send,
    cc2420_read,
    /* cc2420_set_channel, */
    /* detected_energy, */
    cc2420_cca,
    cc2420_receiving_packet,
    pending_packet,
    cc2420_on,
    cc2420_off,
  };

static uint8_t receive_on;

static int channel;

/*---------------------------------------------------------------------------*/

static void
getrxdata(void *buf, int len)
{
  CC2420_READ_FIFO_BUF(buf, len);
}
static void
getrxbyte(uint8_t *byte)
{
  CC2420_READ_FIFO_BYTE(*byte);
}
static void
flushrx(void)
{
  uint8_t dummy;

  CC2420_READ_FIFO_BYTE(dummy);
  CC2420_STROBE(CC2420_SFLUSHRX);
  CC2420_STROBE(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
static void
strobe(enum cc2420_register regname)
{
  CC2420_STROBE(regname);
}
/*---------------------------------------------------------------------------*/
static unsigned int
status(void)
{
  uint8_t status;
  CC2420_GET_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
  COOJA_DEBUG_STR("cc2420_on\n");

  CC2420_ENABLE_FIFOP_INT();
  strobe(CC2420_SRXON);

  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 100);

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  receive_on = 1;
}
static void
off(void)
{
  COOJA_DEBUG_STR("cc2420_off\n");

  /*  PRINTF("off\n");*/
  receive_on = 0;

  /* Wait for transmission to end before turning radio off. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 200);

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  strobe(CC2420_SRFOFF);
  CC2420_DISABLE_FIFOP_INT();
  COOJA_DEBUG_STR("cc2420_off end\n");

//  if(!CC2420_FIFOP_IS_1) {
//    flushrx();
//  }
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static unsigned
getreg(enum cc2420_register regname)
{
  unsigned reg;
  CC2420_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
static void
setreg(enum cc2420_register regname, unsigned value)
{
  CC2420_WRITE_REG(regname, value);
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
  uint16_t reg;

  reg = getreg(CC2420_TXCTRL);
  reg = (reg & 0xffe0) | (power & 0x1f);
  setreg(CC2420_TXCTRL, reg);
}
/*---------------------------------------------------------------------------*/
#define AUTOACK (1 << 4)
#define ADR_DECODE (1 << 11)
#define RXFIFO_PROTECTION (1 << 9)
#define CORR_THR(n) (((n) & 0x1f) << 6)
#define FIFOP_THR(n) ((n) & 0x7f)
#define RXBPF_LOCUR (1 << 13);
/*---------------------------------------------------------------------------*/
/* Data structure used as the internal RX buffer */
MEMB(rf_memb, struct received_frame_s, 4);
LIST(rf_list);
/*---------------------------------------------------------------------------*/
int
cc2420_init(void)
{
  COOJA_DEBUG_STR("cc2420_init\n");

  uint16_t reg;
  {
    int s = splhigh();
    cc2420_arch_init();		/* Initalize ports and SPI. */
    CC2420_DISABLE_FIFOP_INT();
    CC2420_FIFOP_INT_INIT();
    splx(s);
  }

  /* Turn on voltage regulator and reset. */
  SET_VREG_ACTIVE();
  clock_delay(250);
  SET_RESET_ACTIVE();
  clock_delay(127);
  SET_RESET_INACTIVE();
  clock_delay(125);


  /* Turn on the crystal oscillator. */
  strobe(CC2420_SXOSCON);

  /* Turn on/off automatic packet acknowledgment and address decoding. */
  reg = getreg(CC2420_MDMCTRL0);

//#if CC2420_CONF_AUTOACK
//  reg |= AUTOACK | ADR_DECODE;
//#else
//  //reg |= ADR_DECODE;
//  //reg &= ~(AUTOACK | ADR_DECODE);
//  //reg = (reg & ~AUTOACK) | ADR_DECODE;
//#endif /* CC2420_CONF_AUTOACK */
  //reg |= AUTOACK | ADR_DECODE;
  //reg &= ~(AUTOACK | ADR_DECODE);
  //reg |= ADR_DECODE;
  reg = (reg & ~AUTOACK) | ADR_DECODE;
  //reg &= ~(AUTOACK | ADR_DECODE);
  setreg(CC2420_MDMCTRL0, reg);

  /* Set transmission turnaround time to the lower setting (8 symbols
     = 0.128 ms) instead of the default (12 symbols = 0.192 ms). */
  /*  reg = getreg(CC2420_TXCTRL);
  reg &= ~(1 << 13);
  setreg(CC2420_TXCTRL, reg);*/


  /* Change default values as recomended in the data sheet, */
  /* correlation threshold = 20, RX bandpass filter = 1.3uA. */
  setreg(CC2420_MDMCTRL1, CORR_THR(20));
  reg = getreg(CC2420_RXCTRL1);
  reg |= RXBPF_LOCUR;
  setreg(CC2420_RXCTRL1, reg);

  /* Set the FIFOP threshold to maximum. */
  setreg(CC2420_IOCFG0, FIFOP_THR(FIFOP_THRESHOLD));

  /* Turn off "Security enable" (page 32). */
  reg = getreg(CC2420_SECCTRL0);
  reg &= ~RXFIFO_PROTECTION;
  setreg(CC2420_SECCTRL0, reg);

  cc2420_set_pan_addr(0xffff, 0x0000, NULL);
  cc2420_set_channel(CC2420_CONF_CHANNEL);
  cc2420_set_cca_threshold(CC2420_CONF_CCA_THRESH);

  flushrx();
  memb_init(&rf_memb);
  list_init(rf_list);
  process_start(&cc2420_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_transmit(unsigned short payload_len)
{
  int i, txpower;
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */

  GET_LOCK();

  txpower = 0;
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Remember the current transmission power */
    txpower = cc2420_get_txpower();
    /* Set the specified transmission power */
    set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  }

  total_len = payload_len + AUX_LEN;

  /* The TX FIFO can only hold one packet. Make sure to not overrun
   * FIFO by waiting for transmission to start here and synchronizing
   * with the CC2420_TX_ACTIVE check in cc2420_send.
   *
   * Note that we may have to wait up to 320 us (20 symbols) before
   * transmission starts.
   */
#ifndef CC2420_CONF_SYMBOL_LOOP_COUNT
#error CC2420_CONF_SYMBOL_LOOP_COUNT needs to be set!!!
#else
#define LOOP_20_SYMBOLS CC2420_CONF_SYMBOL_LOOP_COUNT
#endif

#if WITH_SEND_CCA
  strobe(CC2420_SRXON);
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 10);
  strobe(CC2420_STXONCCA);
#else /* WITH_SEND_CCA */
  strobe(CC2420_STXON);
#endif /* WITH_SEND_CCA */
  for(i = LOOP_20_SYMBOLS; i > 0; i--) {
    if(CC2420_SFD_IS_1) {
//write sfd...
    	{
        rtimer_clock_t sfd_timestamp;
        sfd_timestamp = cc2420_sfd_start_time;
        if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
           PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP) {
          /* Write timestamp to last two bytes of packet in TXFIFO. */
          CC2420_WRITE_RAM(&sfd_timestamp, CC2420RAM_TXFIFO + payload_len - 1, 2);
        }
      }

      if(!(status() & BV(CC2420_TX_ACTIVE))) {
        /* SFD went high but we are not transmitting. This means that
           we just started receiving a packet, so we drop the
           transmission. */
        RELEASE_LOCK();
        return RADIO_TX_COLLISION;
      }
      if(receive_on) {
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
      }
      ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
      /* We wait until transmission has ended so that we get an
	 accurate measurement of the transmission time.*/
      BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
      ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,cc2420_get_txpower());
#endif
      ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
      if(receive_on) {
	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
      } else {
	/* We need to explicitly turn off the radio,
	 * since STXON[CCA] -> TX_ACTIVE -> RX_ACTIVE */
	off();
      }

      if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
        /* Restore the transmission power */
        set_txpower(txpower & 0xff);
      }

      RELEASE_LOCK();
      return RADIO_TX_OK;
    }
  }

  /* If we are using WITH_SEND_CCA, we get here if the packet wasn't
     transmitted because of other channel activity. */
  RIMESTATS_ADD(contentiondrop);
  PRINTF("cc2420: do_send() transmission never started\n");

  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    /* Restore the transmission power */
    set_txpower(txpower & 0xff);
  }

  RELEASE_LOCK();
  return RADIO_TX_COLLISION;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t total_len;
#if CC2420_CONF_CHECKSUM
  uint16_t checksum;
#endif /* CC2420_CONF_CHECKSUM */
  GET_LOCK();

  PRINTF("cc2420: sending %d bytes\n", payload_len);

  RIMESTATS_ADD(lltx);

  /* Wait for any previous transmission to finish. */
  /*  while(status() & BV(CC2420_TX_ACTIVE));*/

  /* Write packet to TX FIFO. */
  strobe(CC2420_SFLUSHTX);

#if CC2420_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif /* CC2420_CONF_CHECKSUM */
  total_len = payload_len + AUX_LEN;
  CC2420_WRITE_FIFO_BUF(&total_len, 1);
  CC2420_WRITE_FIFO_BUF(payload, payload_len);
#if CC2420_CONF_CHECKSUM
  CC2420_WRITE_FIFO_BUF(&checksum, CHECKSUM_LEN);
#endif /* CC2420_CONF_CHECKSUM */

  RELEASE_LOCK();
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_send(const void *payload, unsigned short payload_len)
{
  cc2420_prepare(payload, payload_len);
  return cc2420_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2420_off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    /*    printf("Off when locked (%d)\n", locked);*/
    lock_off = 1;
    return 1;
  }

  GET_LOCK();
  /* If we are currently receiving a packet (indicated by SFD == 1),
     we don't actually switch the radio off now, but signal that the
     driver should switch off the radio once the packet has been
     received and processed, by setting the 'lock_off' variable. */
  if(status() & BV(CC2420_TX_ACTIVE)) {
    lock_off = 1;
  } else {
    off();
  }
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_on(void)
{
  if(receive_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }

  GET_LOCK();
  on();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
cc2420_set_channel(int c)
{
  uint16_t f;

  GET_LOCK();
  /*
   * Subtract the base channel (11), multiply by 5, which is the
   * channel spacing. 357 is 2405-2048 and 0x4000 is LOCK_THR = 1.
   */
  channel = c;

  f = 5 * (c - 11) + 357 + 0x4000;
  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL((status() & (BV(CC2420_XOSC16M_STABLE))), RTIMER_SECOND / 10);

  /* Wait for any transmission to end. */
  BUSYWAIT_UNTIL(!(status() & BV(CC2420_TX_ACTIVE)), RTIMER_SECOND / 10);

  setreg(CC2420_FSCTRL, f);

  /* If we are in receive mode, we issue an SRXON command to ensure
     that the VCO is calibrated. */
  if(receive_on) {
    strobe(CC2420_SRXON);
  }

  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  uint16_t f = 0;
  uint8_t tmp[2];

  GET_LOCK();

  /*
   * Writing RAM requires crystal oscillator to be stable.
   */
  BUSYWAIT_UNTIL(status() & (BV(CC2420_XOSC16M_STABLE)), RTIMER_SECOND / 10);

  tmp[0] = pan & 0xff;
  tmp[1] = pan >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_PANID, 2);

  tmp[0] = addr & 0xff;
  tmp[1] = addr >> 8;
  CC2420_WRITE_RAM(&tmp, CC2420RAM_SHORTADDR, 2);
  if(ieee_addr != NULL) {
    uint8_t tmp_addr[8];
    /* LSB first, MSB last for 802.15.4 addresses in CC2420 */
    for (f = 0; f < 8; f++) {
      tmp_addr[7 - f] = ieee_addr[f];
    }
    CC2420_WRITE_RAM(tmp_addr, CC2420RAM_IEEEADDR, 8);
  }
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO. !!! not anymore !!!
 */
#if CC2420_TIMETABLE_PROFILING
#define cc2420_timetable_size 16
TIMETABLE(cc2420_timetable);
TIMETABLE_AGGREGATE(aggregate_time, 10);
#endif /* CC2420_TIMETABLE_PROFILING */

#define RXFIFO_START  0x080
#define RXFIFO_END    0x0FF
#define RXFIFO_SIZE   128
#define RXFIFO_ADDR(index) (RXFIFO_START + (index) % RXFIFO_SIZE)

#define DO_ACK 2
#define IS_DATA 4
#define IS_ACK 8

uint8_t
frame80254_parse_irq(uint8_t *data, uint8_t len)
{
	if(len < ACK_LEN) {
		return 0;
	}
	/* decode the FCF */
	uint8_t do_ack = (data[0] >> 5) & 1 == 1 ? DO_ACK : 0;
	uint8_t is_data = (data[0] & 7) == FRAME802154_DATAFRAME ? IS_DATA : 0;
	uint8_t is_ack = (data[0] & 7) == FRAME802154_ACKFRAME ? IS_ACK : 0;

	return do_ack | is_data | is_ack;
}
/*---------------------------------------------------------------------------*/
volatile rtimer_clock_t cell_start_time = 0;

void
cc2420_arch_sfd_sync(rtimer_clock_t start_time)
{
  /* Start Timer_B in continuous mode. */
  //TBCTL |= MC1;
  cell_start_time = start_time;
  TBR = RTIMER_NOW();
}
/*---------------------------------------------------------------------------*/
volatile struct received_frame_s *last_rf;
volatile uint8_t last_acked;
volatile rtimer_clock_t rx_end_time;
int
cc2420_interrupt(void)
{
	COOJA_DEBUG_STR("cc2420_interrupt\n");
	uint8_t nack = 0;
  uint8_t len, fcf, seqno, footer1, is_data, for_us, is_ack;
  uint8_t len_a, len_b;
  int do_ack = 0;
  int frame_valid = 0;
  uint8_t ret = 0;
  struct received_frame_s *rf;
  uint16_t ack_status=0;
  //{0x02, 0x00, seqno, 0x02, 0x1e, ack_status&0xff, (ack_status>>8)&0xff};
  static unsigned char ackbuf[1+ACK_LEN + EXTRA_ACK_LEN]; // = {ACK_LEN + EXTRA_ACK_LEN + AUX_LEN, 0x02, 0x00, seqno, 0x02, 0x1e, ack_status_LSB, ack_status_MSB};
  unsigned char* buf_ptr = NULL;
  last_acked=0;
#if CC2420_TIMETABLE_PROFILING
  timetable_clear(&cc2420_timetable);
  TIMETABLE_TIMESTAMP(cc2420_timetable, "interrupt");
#endif /* CC2420_TIMETABLE_PROFILING */

  last_packet_timestamp = cc2420_sfd_start_time;
  /* If the lock is taken, we cannot access the FIFO. */
  if(/*locked || need_flush ||*/ !CC2420_FIFO_IS_1) {
  	COOJA_DEBUG_STR("! locked || need_flush || !CC2420_FIFO_IS_1");
    need_flush = 1;
		if(last_packet_timestamp == cc2420_sfd_start_time) {
			while(CC2420_SFD_IS_1);
		}
		COOJA_DEBUG_STR("! locked || need_flush || !CC2420_FIFO_IS_1 end CC2420_SFD_IS_1");
		rx_end_time = RTIMER_NOW();
		off();
    CC2420_CLEAR_FIFOP_INT();
    return tsch_resume_powercycle();
  }

  GET_LOCK();
  CC2420_READ_FIFO_BYTE(len);
  if(len > CC2420_MAX_PACKET_LEN || len <= AUX_LEN) {
  	COOJA_DEBUG_STR("! len > CC2420_MAX_PACKET_LEN || len <= AUX_LEN");
    flushrx();
    CC2420_CLEAR_FIFOP_INT();
		if(last_packet_timestamp == cc2420_sfd_start_time) {
			while(CC2420_SFD_IS_1);
		}
		COOJA_DEBUG_STR("end CC2420_SFD_IS_1");
		rx_end_time = RTIMER_NOW();
		off();
    RELEASE_LOCK();
    COOJA_DEBUG_STR("cc2420_interrupt end\n");
    return tsch_resume_powercycle();
  }

	len -= AUX_LEN;
	/* Allocate space to store the received frame */
	rf=memb_alloc(&rf_memb);
	//rf=NULL; //XXX inject error
  if(rf != NULL) {
  	COOJA_DEBUG_STR("irq rf!=NULL memb_alloc ok");
  	nack = 0;
  	len_a = len > FIFOP_THRESHOLD ? FIFOP_THRESHOLD : len;
  	buf_ptr = rf->buf;
		rf->len = len;
		rf->acked = 0;
		list_add(rf_list, rf);
  } else {
  	COOJA_DEBUG_STR("irq rf=NULL");
  	nack = 1;
  	buf_ptr = ackbuf;
  	len_a = len > ACK_LEN + EXTRA_ACK_LEN ? ACK_LEN + EXTRA_ACK_LEN : len;
  }
  len_b = len - len_a;
	CC2420_READ_FIFO_BUF(buf_ptr, len_a);

	fcf = buf_ptr[0];
	seqno = buf_ptr[2];
	if(rf != NULL) {
		rf->seqno = seqno;
	}
	ret = frame80254_parse_irq(buf_ptr, len_a);
	is_data = ret & IS_DATA;
	do_ack = ret & DO_ACK;
	is_ack = ret & IS_ACK;
	if(!is_ack) {
	  process_poll(&cc2420_process);
	}
	if(do_ack) {   /* Prepare ack */
		COOJA_DEBUG_STR("do_ack");
		//calculating sync
		int16_t time_difference = (uint16_t)(cell_start_time + TsTxOffset) - cc2420_sfd_start_time;
		time_difference = (time_difference * 3051)/100;
		if(time_difference >=0) {
			ack_status=time_difference & 0x07ff;
		} else {
			ack_status=(-time_difference) & 0x07ff + 0x0800;
		}
		if(nack) {
			ack_status |= 0x8000;
		}
		//ackbuf[1+ACK_LEN + EXTRA_ACK_LEN] = {ACK_LEN + EXTRA_ACK_LEN + AUX_LEN, 0x02, 0x00, seqno, 0x02, 0x1e, ack_status_LSB, ack_status_MSB};
		ackbuf[0] = ACK_LEN + EXTRA_ACK_LEN + AUX_LEN; //total_len
		ackbuf[1] = 0x02;
		ackbuf[2] = 0x00;
		/* Append IE timesync */
		ackbuf[3] = seqno;
		ackbuf[4] = 0x02;
		ackbuf[5] = 0x1e;
		ackbuf[6] = ack_status&0xff;
		ackbuf[7] = (ack_status>>8)&0xff;
		/* Write ack in fifo */
		CC2420_STROBE(CC2420_SFLUSHTX); /* Flush Tx fifo */
		CC2420_WRITE_FIFO_BUF(ackbuf, 1+ACK_LEN + EXTRA_ACK_LEN);
	}

	/* Wait for end of reception */
	if(last_packet_timestamp == cc2420_sfd_start_time) {
		while(CC2420_SFD_IS_1);
	}
	COOJA_DEBUG_STR("end CC2420_SFD_IS_1");
	rx_end_time = RTIMER_NOW();
	off();
	//XXX rx_end_time should not be 0
	if(!rx_end_time) {
		rx_end_time++;
		COOJA_DEBUG_STR("end CC2420_SFD_IS_1 rx_end_time=0++");
	}

	int overflow = CC2420_FIFOP_IS_1 && !CC2420_FIFO_IS_1;
	CC2420_READ_RAM_BYTE(footer1, RXFIFO_ADDR(len + AUX_LEN));

	if(!overflow && (footer1 & FOOTER1_CRC_OK)) { /* CRC is correct */
		if(rf && len_b>0) { /* Get rest of the data.
		 No need to read the footer; we already checked it in place
		 before acking. */
			COOJA_DEBUG_STR("rf && len_b>0");
			CC2420_READ_FIFO_BUF(rf->buf + len_a, len_b);
		}
		frame_valid = 1;
	} else { /* CRC is wrong */
		COOJA_DEBUG_STR("! overflow || CRC is wrong ");
		if(do_ack) {
			CC2420_STROBE(CC2420_SFLUSHTX); /* Flush Tx fifo */
		}
		if(rf) {
			list_chop(rf_list);
			memb_free(&rf_memb, rf);
			rf = NULL;
		}
	}
	if(frame_valid && do_ack) {
		last_acked = 1 + nack;
		last_rf = rf;
	} else {
		last_rf = NULL;
		last_acked = 0;
	}
  /* Flush rx fifo (because we're doing direct FIFO addressing and
   * we don't want to lose track of where we are in the FIFO) */
  flushrx();
  CC2420_CLEAR_FIFOP_INT();
  RELEASE_LOCK();
  COOJA_DEBUG_STR("cc2420_interrupt end\n");
  if(!last_acked) {
  	COOJA_DEBUG_STR("cc2420_interrupt don't need ack\n");
  } else {
  	COOJA_DEBUG_STR("cc2420_interrupt needs ack\n");
  }
//  if(fcf & 7 != FRAME802154_ACKFRAME) {
    COOJA_DEBUG_STR("cc2420_interrupt end call tsch_resume_powercycle()\n");
  	return tsch_resume_powercycle();
//  }
//  return 1;
}
void send_ack(void) {
	COOJA_DEBUG_STR("Send ACK");
	strobe(CC2420_STXON); /* Send ACK */
  off();
  last_rf = NULL;
  last_acked = 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2420_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("cc2420_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
#if CC2420_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(cc2420_timetable, "poll");
#endif /* CC2420_TIMETABLE_PROFILING */
    
    PRINTF("cc2420_process: calling receiver callback\n");
  	COOJA_DEBUG_STR("cc2420_process: calling receiver callback\n");

    if(need_flush) {
      need_flush = 0;
      GET_LOCK();
      flushrx();
      RELEASE_LOCK();
      COOJA_DEBUG_STR("cc2420_process: need_flush\n");
    }

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = cc2420_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    int frame_type = ((uint8_t*)packetbuf_dataptr())[0] & 7;
    if(frame_type == FRAME802154_ACKFRAME) {
      len = 0;
    }
    packetbuf_set_datalen(len);

    NETSTACK_RDC.input();

#if CC2420_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(cc2420_timetable, "end");
    timetable_aggregate_compute_detailed(&aggregate_time,
                                         &cc2420_timetable);
      timetable_clear(&cc2420_timetable);
#endif /* CC2420_TIMETABLE_PROFILING */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
cc2420_read(void *buf, unsigned short bufsize)
{
  GET_LOCK();
  COOJA_DEBUG_STR("cc2420_read \n");

  struct received_frame_s *rf = list_pop(rf_list);
  if(rf == NULL) {
    RELEASE_LOCK();
    COOJA_DEBUG_STR("cc2420_read rf == NULL\n");
    return 0;
  } else {
    if(list_head(rf_list) != NULL) {
      COOJA_DEBUG_STR("cc2420_read list_head(rf_list) != NULL\n");
      /* If there are other packets pending, poll */
      process_poll(&cc2420_process);
    }
    int len = rf->len;
    if(len > bufsize) {
      memb_free(&rf_memb, rf);
      RELEASE_LOCK();
      COOJA_DEBUG_STR("cc2420_read len > bufsize\n");
      return 0;
    }
    memcpy(buf, rf->buf, len);
    memb_free(&rf_memb, rf);
    RELEASE_LOCK();
    return len;
  }
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_txpower(uint8_t power)
{
  GET_LOCK();
  set_txpower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
cc2420_get_txpower(void)
{
  int power;
  GET_LOCK();
  power = (int)(getreg(CC2420_TXCTRL) & 0x001f);
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
int
cc2420_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

  if(locked) {
    return 0;
  }
  
  GET_LOCK();

  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }
  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  rssi = (int)((signed char)getreg(CC2420_RSSI));

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return rssi;
}
/*---------------------------------------------------------------------------*/
/*
static int
detected_energy(void)
{
  return cc2420_rssi();
}
*/
/*---------------------------------------------------------------------------*/
int
cc2420_cca_valid(void)
{
  int valid;
  if(locked) {
    return 1;
  }
  GET_LOCK();
  valid = !!(status() & BV(CC2420_RSSI_VALID));
  RELEASE_LOCK();
  return valid;
}
/*---------------------------------------------------------------------------*/
static int
cc2420_cca(void)
{
  int cca;
  int radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we preted that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  if(!receive_on) {
    radio_was_off = 1;
    cc2420_on();
  }

  /* Make sure that the radio really got turned on. */
  if(!receive_on) {
    RELEASE_LOCK();
    if(radio_was_off) {
      cc2420_off();
    }
    return 1;
  }

  BUSYWAIT_UNTIL(status() & BV(CC2420_RSSI_VALID), RTIMER_SECOND / 100);

  cca = CC2420_CCA_IS_1;

  if(radio_was_off) {
    cc2420_off();
  }
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
cc2420_receiving_packet(void)
{
  return CC2420_SFD_IS_1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return list_head(rf_list) != NULL;
}
/*---------------------------------------------------------------------------*/
void
cc2420_set_cca_threshold(int value)
{
  uint16_t shifted = value << 8;
  GET_LOCK();
  setreg(CC2420_RSSI, shifted);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
int
cc2420_pending_irq(void)
{
  return CC2420_FIFO_IS_1;
}
