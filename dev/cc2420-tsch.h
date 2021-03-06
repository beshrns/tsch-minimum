/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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

/**
 * \file
 *         CC2420 driver header file
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#ifndef __CC2420_H__
#define __CC2420_H__

#include "contiki.h"
#include "dev/spi.h"
#include "dev/radio.h"
#include "dev/cc2420_const.h"
#include "rimeaddr.h"

int cc2420_init(void);
//Timesync IE length with header = 4
#if CC2420_CONF_CHECKSUM
#include "lib/crc16.h"
#define CHECKSUM_LEN 2
#else
#define CHECKSUM_LEN 0
#endif /* CC2420_CONF_CHECKSUM */
#define FOOTER_LEN 2
#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#define ACK_LEN 3
#define EXTRA_ACK_LEN 4
#define CC2420_MAX_PACKET_LEN      127
struct received_frame_s {
  struct received_frame_s *next;
  uint8_t buf[CC2420_MAX_PACKET_LEN];
  uint8_t len;
  rimeaddr_t source_address;
};

int cc2420_set_channel(int channel);
int cc2420_get_channel(void);

void cc2420_set_pan_addr(unsigned pan,
                                unsigned addr,
                                const uint8_t *ieee_addr);

extern signed char cc2420_last_rssi;
extern uint8_t cc2420_last_correlation;

int cc2420_rssi(void);

extern const struct radio_driver cc2420_driver;

/**
 * \param power Between 1 and 31.
 */
void cc2420_set_txpower(uint8_t power);
int cc2420_get_txpower(void);
#define CC2420_TXPOWER_MAX  31
#define CC2420_TXPOWER_MIN   0

/**
 * Interrupt function, called from the simple-cc2420-arch driver.
 *
 */
int cc2420_interrupt(void);

/* XXX hack: these will be made as Chameleon packet attributes */
extern rtimer_clock_t cc2420_time_of_arrival,
  cc2420_time_of_departure;
extern int cc2420_authority_level_of_sender;

int cc2420_on(void);
int cc2420_off(void);

void cc2420_set_cca_threshold(int value);

/************************************************************************/
/* Additional low-level functions for the CC2420 */
/************************************************************************/
typedef void(softack_make_callback_f)(uint8_t **ackbuf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack);
typedef void(softack_interrupt_exit_callback_f)(uint8_t is_ack, uint8_t need_ack, struct received_frame_s * last_rf);

/* Subscribe with two callbacks called from FIFOP interrupt */
void cc2420_softack_subscribe(softack_make_callback_f *softack_make, softack_interrupt_exit_callback_f *interrupt_exit);
rtimer_clock_t cc2420_get_rx_end_time(void);
void cc2420_arch_init(void);
void cc2420_send_ack(void);
int cc2420_read_ack(void *buf, int);
int cc2420_pending_irq(void);
void cc2420_address_decode(uint8_t enable);
//to initialize radio sfd counter and synchronize it with rtimer
void cc2420_sfd_sync(uint8_t capture_start_sfd,
		uint8_t capture_end_sfd);
uint16_t cc2420_read_sfd_timer(void);

#define NETSTACK_RADIO_softack_subscribe 	cc2420_softack_subscribe
#define NETSTACK_RADIO_get_rx_end_time 		cc2420_get_rx_end_time
#define NETSTACK_RADIO_send_ack 					cc2420_send_ack
#define NETSTACK_RADIO_read_ack 					cc2420_read_ack
#define NETSTACK_RADIO_pending_irq 				cc2420_pending_irq
#define NETSTACK_RADIO_address_decode 		cc2420_address_decode
#define NETSTACK_RADIO_sfd_sync 					cc2420_sfd_sync
#define NETSTACK_RADIO_read_sfd_timer 		cc2420_read_sfd_timer
#define NETSTACK_RADIO_set_channel 				cc2420_set_channel

/************************************************************************/
/* Additional SPI Macros for the CC2420 */
/************************************************************************/
/* Send a strobe to the CC2420 */
#define CC2420_STROBE(s)                                   \
  do {                                                  \
    CC2420_SPI_ENABLE();                                \
    SPI_WRITE(s);                                       \
    CC2420_SPI_DISABLE();                               \
  } while (0)

/* Write to a register in the CC2420                         */
/* Note: the SPI_WRITE(0) seems to be needed for getting the */
/* write reg working on the Z1 / MSP430X platform            */
#define CC2420_WRITE_REG(adr,data)                              \
  do {                                                       \
    CC2420_SPI_ENABLE();                                     \
    SPI_WRITE_FAST(adr);                                     \
    SPI_WRITE_FAST((uint8_t)((data) >> 8));                  \
    SPI_WRITE_FAST((uint8_t)(data & 0xff));                  \
    SPI_WAITFORTx_ENDED();                                   \
    SPI_WRITE(0);                                            \
    CC2420_SPI_DISABLE();                                    \
  } while(0)

/* Read a register in the CC2420 */
#define CC2420_READ_REG(adr,data)                          \
  do {                                                  \
    CC2420_SPI_ENABLE();                                \
    SPI_WRITE(adr | 0x40);                              \
    data = (uint8_t)SPI_RXBUF;                          \
    SPI_TXBUF = 0;                                      \
    SPI_WAITFOREORx();                                  \
    data = SPI_RXBUF << 8;                              \
    SPI_TXBUF = 0;                                      \
    SPI_WAITFOREORx();                                  \
    data |= SPI_RXBUF;                                  \
    CC2420_SPI_DISABLE();                               \
  } while(0)

#define CC2420_READ_FIFO_BYTE(data)                        \
  do {                                                  \
    CC2420_SPI_ENABLE();                                \
    SPI_WRITE(CC2420_RXFIFO | 0x40);                    \
    (void)SPI_RXBUF;                                    \
    SPI_READ(data);                                     \
    clock_delay(1);                                     \
    CC2420_SPI_DISABLE();                               \
  } while(0)

#define CC2420_READ_FIFO_BUF(buffer,count)                                 \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2420_SPI_ENABLE();                                                \
    SPI_WRITE(CC2420_RXFIFO | 0x40);                                    \
    (void)SPI_RXBUF;                                                    \
    for(i = 0; i < (count); i++) {                                      \
      SPI_READ(((uint8_t *)(buffer))[i]);                               \
    }                                                                   \
    clock_delay(1);                                                     \
    CC2420_SPI_DISABLE();                                               \
  } while(0)

#define CC2420_WRITE_FIFO_BUF(buffer,count)                                \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2420_SPI_ENABLE();                                                \
    SPI_WRITE_FAST(CC2420_TXFIFO);                                           \
    for(i = 0; i < (count); i++) {                                      \
      SPI_WRITE_FAST(((uint8_t *)(buffer))[i]);                              \
    }                                                                   \
    SPI_WAITFORTx_ENDED();                                              \
    CC2420_SPI_DISABLE();                                               \
  } while(0)

/* Write to RAM in the CC2420 */
#define CC2420_WRITE_RAM(buffer,adr,count)                 \
  do {                                                       \
    uint8_t i;                                               \
    CC2420_SPI_ENABLE();                                     \
    SPI_WRITE_FAST(0x80 | ((adr) & 0x7f));                   \
    SPI_WRITE_FAST(((adr) >> 1) & 0xc0);                     \
    for(i = 0; i < (count); i++) {                           \
      SPI_WRITE_FAST(((uint8_t*)(buffer))[i]);               \
    }                                                        \
    SPI_WAITFORTx_ENDED();                                   \
    CC2420_SPI_DISABLE();                                    \
  } while(0)

/* Read from RAM in the CC2420 */
#define CC2420_READ_RAM(buffer,adr,count)                    \
  do {                                                       \
    uint8_t i;                                               \
    CC2420_SPI_ENABLE();                                     \
    SPI_WRITE(0x80 | ((adr) & 0x7f));                        \
    SPI_WRITE((((adr) >> 1) & 0xc0) | 0x20);                 \
    SPI_RXBUF;                                               \
    for(i = 0; i < (count); i++) {                           \
      SPI_READ(((uint8_t*)(buffer))[i]);                     \
    }                                                        \
    CC2420_SPI_DISABLE();                                    \
  } while(0)

/* Read status of the CC2420 */
#define CC2420_GET_STATUS(s)                       \
  do {                                          \
    CC2420_SPI_ENABLE();                        \
    SPI_WRITE(CC2420_SNOP);                     \
    s = SPI_RXBUF;                              \
    CC2420_SPI_DISABLE();                       \
  } while (0)

#endif /* __CC2420_H__ */
