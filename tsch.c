/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
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
 *         TSCH.
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */
#include "contiki.h"
#include "contiki-conf.h"
#include "tsch.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include <string.h>
#include "sys/rtimer.h"
#include "cooja-debug.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"

static volatile ieee154e_vars_t ieee154e_vars;

#define ACK_LEN 3
#define EXTRA_ACK_LEN 4

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef TSCH_CONF_ADDRESS_FILTER
#define TSCH_ADDRESS_FILTER TSCH_CONF_ADDRESS_FILTER
#else
#define TSCH_ADDRESS_FILTER 0
#endif /* TSCH_CONF_ADDRESS_FILTER */

#ifndef TSCH_802154_AUTOACK
#ifdef TSCH_CONF_802154_AUTOACK
#define TSCH_802154_AUTOACK TSCH_CONF_802154_AUTOACK
#else
#define TSCH_802154_AUTOACK 0
#endif /* TSCH_CONF_802154_AUTOACK */
#endif /* TSCH_802154_AUTOACK */

#ifndef TSCH_802154_AUTOACK_HW
#ifdef TSCH_CONF_802154_AUTOACK_HW
#define TSCH_802154_AUTOACK_HW TSCH_CONF_802154_AUTOACK_HW
#else
#define TSCH_802154_AUTOACK_HW 0
#endif /* TSCH_CONF_802154_AUTOACK_HW */
#endif /* TSCH_802154_AUTOACK_HW */

#if TSCH_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef TSCH_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME TSCH_CONF_ACK_WAIT_TIME
#else /* TSCH_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* TSCH_CONF_ACK_WAIT_TIME */
#ifdef TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* TSCH_802154_AUTOACK */

#ifdef TSCH_CONF_SEND_802154_ACK
#define TSCH_SEND_802154_ACK TSCH_CONF_SEND_802154_ACK
#else /* TSCH_CONF_SEND_802154_ACK */
#define TSCH_SEND_802154_ACK 0
#endif /* TSCH_CONF_SEND_802154_ACK */

#if TSCH_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* TSCH_SEND_802154_ACK */

#if TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW
struct seqno {
	rimeaddr_t sender;
	uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 8
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */

static struct seqno received_seqnos[MAX_SEQNOS];
#endif /* TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW */

// variable to protect queue structure
volatile uint8_t working_on_queue;

#define MAX_NUM_PKT 8 // POWER OF 2
#define MAX_NEIGHBOR 16
#define macMinBE 3
#define macMaxFrameRetries 4
#define macMaxBE 5

// TSCH PACKET STRUCTURE
struct TSCH_packet
{
	struct queuebuf * pkt; // pointer to the packet to be sent
	uint8_t transmissions; // #transmissions performed for this packet
	mac_callback_t sent; // callback for this packet
	void *ptr; //?? for callback ...
};

// NEIGHBOR QUEUE STRUCTURE
struct neighbor_queue
{
	struct neighbor_queue *next; // pointer to next neighbor
	rimeaddr_t addr; // neighbor address
	uint8_t BE_value; // current value of backoff exponent for this neighbor
	uint8_t BW_value; // current value of backoff counter fot this neighbor
	struct TSCH_packet buffer[MAX_NUM_PKT]; // circular buffer of packets for this neighbor
	uint8_t put_ptr, get_ptr, mask; // data-structures for circular buffer
};

// DECLARATION OF THE LIST OF NEIGHBORS
MEMB(neighbor_memb, struct neighbor_queue, MAX_NEIGHBOR);
LIST(neighbor_list);

// PROTOTYPES
struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr);
int
add_queue(const rimeaddr_t *addr);
int
remove_queue(const rimeaddr_t *addr);
int
add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr);
int
remove_packet_from_queue(const rimeaddr_t *addr);
struct TSCH_packet*
read_packet_from_queue(const rimeaddr_t *addr);
static void
tsch_timer(void *ptr);

// This function returns a pointer to the queue of neighbor whose address is equal to addr

struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = list_head(neighbor_list);
	while (n != NULL) {
		if (rimeaddr_cmp(&n->addr, addr)) {
			return n;
		}
		n = list_item_next(n);
	}
	return NULL;
}

// This function adds one queue for neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to allocate

int
add_queue(const rimeaddr_t *addr)
{
	working_on_queue = 1;
	struct neighbor_queue *n;
	int i;
	n = memb_alloc(&neighbor_memb);
	if (n != NULL) {
		/* Init neighbor entry */
		rimeaddr_copy(&n->addr, addr);
		n->BE_value = macMinBE;
    n->BW_value = 0;
		n->put_ptr = 0;
		n->get_ptr = 0;
		n->mask = MAX_NUM_PKT - 1;
		for (i = 0; i < MAX_NUM_PKT; i++) {
			n->buffer[i].pkt = 0;
			n->buffer[i].transmissions = 0;
		}
		list_add(neighbor_list, n);
		working_on_queue = 0;
		COOJA_DEBUG_PRINTF("ADD QUEUE %d Succeeded\n", addr->u8[7]);
    //COOJA_DEBUG_PRINTF("addr %u %u %u %u %u %u %u %u\n", addr->u8[7], addr->u8[6], addr->u8[5], addr->u8[4], addr->u8[3], addr->u8[2], addr->u8[1], addr->u8[0]);
		return 1;
	}
	COOJA_DEBUG_PRINTF("ADD QUEUE %d FAILED\n", addr->u8[7]);

	working_on_queue = 0;
	return 0;
}

// This function remove the queue of neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to find the queue

int
remove_queue(const rimeaddr_t *addr)
{
	working_on_queue = 1;
	int i;
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		for (i = 0; i < MAX_NUM_PKT; i++) {      // free packets of neighbor
			queuebuf_free(n->buffer[i].pkt);
		}
		list_remove(neighbor_list, n); // free queue of neighbor
		memb_free(&neighbor_memb, n);
		working_on_queue = 0;
		return 1;
	}
	working_on_queue = 0;
	return 0;
}

// This function adds one packet to the queue of neighbor whose address is addr
// return 1 ok, 0 failed to allocate
// the packet to be inserted is in packetbuf
int
add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr)
{
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		if (((n->put_ptr - n->get_ptr) & n->mask) == n->mask) {
			return 0;
		}
		n->buffer[n->put_ptr].pkt = queuebuf_new_from_packetbuf(); // create new packet from packetbuf
		void *p = queuebuf_dataptr(n->buffer[n->put_ptr].pkt);

		n->buffer[n->put_ptr].sent = sent;
		n->buffer[n->put_ptr].ptr = ptr;
		n->put_ptr = (n->put_ptr + 1) & n->mask;
		return 1;
	}
	return 0;
}

// This function removes the head-packet of the queue of neighbor whose address is addr
// return 1 ok, 0 failed
// remove one packet from the queue
int
remove_packet_from_queue(const rimeaddr_t *addr)
{
//COOJA_DEBUG_STR("CHIAMATO RIMOSSO\n");
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		//COOJA_DEBUG_STR("ENTRATO \n");

		if (((n->put_ptr - n->get_ptr) & n->mask) > 0) {
			queuebuf_free(n->buffer[n->get_ptr].pkt);
			n->get_ptr = (n->get_ptr + 1) & n->mask;
			//COOJA_DEBUG_STR("RIMOSSO FRAME\n");
			return 1;
		} else {
			//COOJA_DEBUG_STR("QUA1\n");
			return 0;
		}
	}
//COOJA_DEBUG_STR("QUA2\n");
	return 0;
}

// This function returns the first packet in the queue of neighbor whose address is addr
struct TSCH_packet*
read_packet_from_queue(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		if (((n->put_ptr - n->get_ptr) & n->mask) > 0) {
			return &(n->buffer[n->get_ptr]);
		} else {
			return 0;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
// Function send for TSCH-MAC, puts the packet in packetbuf in the MAC queue
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
	//send_one_packet(sent, ptr);
	COOJA_DEBUG_STR("TSCH send_one_packet\n");

	struct neighbor_queue *n;
	uint16_t seqno;
	const rimeaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
	//Ask for ACK if we are sending anything otherthan broadcast
	if(!rimeaddr_cmp(addr, &rimeaddr_null)) {
		packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
	}
	COOJA_DEBUG_ADDR(addr);
	/* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a pecuilarity
	 in framer-802154.c. */
	seqno = (++ieee154e_vars.dsn) ? ieee154e_vars.dsn : ++ieee154e_vars.dsn;
	packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno);
	if (NETSTACK_FRAMER.create() < 0) {
		return 0;
	}

	/* Look for the neighbor entry */
	n = neighbor_queue_from_addr(addr);
	if (n == NULL) {
		//add new neighbor to list of neighbors
		COOJA_DEBUG_STR("Aadd new neighbor to list of neighbors");
		if (!add_queue(addr))
			return 0;
		COOJA_DEBUG_STR("add new packet to neighbor list");
		//add new packet to neighbor list
		if (!add_packet_to_queue(sent, ptr, addr))
			return 0;
	} else {
		//add new packet to neighbor list
		COOJA_DEBUG_STR("add new packet to neighbor list");
		if (!add_packet_to_queue(sent, ptr, addr))
			return 0;
	}
	COOJA_DEBUG_STR("TSCH send_one_packet OK\n");
	return 1;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
	send_one_packet(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
	COOJA_DEBUG_STR("TSCH send_list\n");

	while (buf_list != NULL) {
		/* We backup the next pointer, as it may be nullified by
		 * mac_call_sent_callback() */
		struct rdc_buf_list *next = buf_list->next;
		int last_sent_ok;

		queuebuf_to_packetbuf(buf_list->buf);
		last_sent_ok = send_one_packet(sent, ptr);

		/* If packet transmission was not successful, we should back off and let
		 * upper layers retransmit, rather than potentially sending out-of-order
		 * packet fragments. */
		if (!last_sent_ok) {
			return;
		}
		buf_list = next;
	}
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
	COOJA_DEBUG_STR("tsch packet_input begin\n");

	int original_datalen;
	uint8_t *original_dataptr;

	original_datalen = packetbuf_datalen();
	original_dataptr = packetbuf_dataptr();
#ifdef NETSTACK_DECRYPT
	NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

#if TSCH_802154_AUTOACK
	if(packetbuf_datalen() == ACK_LEN) {
		/* Ignore ack packets */
		PRINTF("nullrdc: ignored ack\n");
	} else
#endif /* TSCH_802154_AUTOACK */
	if (NETSTACK_FRAMER.parse() < 0) {
		PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if TSCH_ADDRESS_FILTER
	} else if (!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
					&rimeaddr_node_addr)
			&& !rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
					&rimeaddr_null)) {
		PRINTF("nullrdc: not for us\n");
#endif /* TSCH_ADDRESS_FILTER */
	} else {
		int duplicate = 0;

#if TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW
		/* Check for duplicate packet by comparing the sequence number
		 of the incoming packet with the last few ones we saw. */
		int i;
		for(i = 0; i < MAX_SEQNOS; ++i) {
			if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
					rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
							&received_seqnos[i].sender)) {
				/* Drop the packet. */
				PRINTF("nullrdc: drop duplicate link layer packet %u\n",
						packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
				duplicate = 1;
			}
		}
		if(!duplicate) {
			for(i = MAX_SEQNOS - 1; i > 0; --i) {
				memcpy(&received_seqnos[i], &received_seqnos[i - 1],
						sizeof(struct seqno));
			}
			received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
			rimeaddr_copy(&received_seqnos[0].sender,
					packetbuf_addr(PACKETBUF_ADDR_SENDER));
		}
#endif /* TSCH_802154_AUTOACK */

#if TSCH_SEND_802154_ACK
		{
			frame802154_t info154;
			frame802154_parse(original_dataptr, original_datalen, &info154);
			if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
					info154.fcf.ack_required != 0 &&
					rimeaddr_cmp((rimeaddr_t *)&info154.dest_addr,
							&rimeaddr_node_addr)) {
				uint8_t ackdata[ACK_LEN] = {0, 0, 0};

				ackdata[0] = FRAME802154_ACKFRAME;
				ackdata[1] = 0;
				ackdata[2] = info154.seq;
				NETSTACK_RADIO.send(ackdata, ACK_LEN);
			}
		}
#endif /* TSCH_SEND_ACK */
		if (!duplicate) {
			NETSTACK_MAC.input();
			COOJA_DEBUG_STR("tsch packet_input, Not duplicate\n");
		}
	}
	COOJA_DEBUG_STR("tsch packet_input end\n");
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
	return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
	if (keep_radio_on) {
		return NETSTACK_RADIO.on();
	} else {
		return NETSTACK_RADIO.off();
	}
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
	return 0;
}
/*---------------------------------------------------------------------------*/
#define BUSYWAIT_UNTIL_ABS(cond, t0, duration)                     	    \
  do { rtimer_clock_t now = RTIMER_NOW(), t1=t0+duration;               \
  	if((rtimer_clock_t)(t1-now)>duration) break;												\
    while(!(cond) && RTIMER_CLOCK_LT(now, t0));  												\
  } while(0)
/*---------------------------------------------------------------------------*/
#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */
static int keep_radio_on = 0;
int cc2420_set_channel(int c);
#define NETSTACK_RADIO_set_channel cc2420_set_channel

/* to get last packet timing information */
extern volatile uint16_t cc2420_sfd_start_time;
/*---------------------------------------------------------------------------*/
static uint16_t
get_sfd_start_time(void)
{
	return cc2420_sfd_start_time;
}
/*---------------------------------------------------------------------------*/
static uint8_t
hop_channel(uint8_t offset)
{
	uint8_t channel = 11 + (offset + ieee154e_vars.asn) % 16;
	if ( NETSTACK_RADIO_set_channel(channel)) {
		return channel;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/

const rimeaddr_t BROADCAST_CELL_ADDRESS = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
const rimeaddr_t CELL_ADDRESS1 = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } };
const rimeaddr_t CELL_ADDRESS2 = { { 0x00, 0x12, 0x74, 02, 00, 02, 02, 02 } };

const cell_t generic_shared_cell = {
0xffff, 0, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
		LINK_TYPE_NORMAL,
		&BROADCAST_CELL_ADDRESS };

const cell_t generic_eb_cell = { 0, 0, LINK_OPTION_TX, LINK_TYPE_ADVERTISING,
&BROADCAST_CELL_ADDRESS };

const cell_t cell_to_1 ={ 1, 0, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED, LINK_TYPE_NORMAL,  &CELL_ADDRESS1};
const cell_t cell_to_2 ={ 2, 0, LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED, LINK_TYPE_NORMAL,  &CELL_ADDRESS2};

const cell_t * minimum_cells[8] = { &generic_eb_cell, &generic_shared_cell,
		&generic_shared_cell, &generic_shared_cell, &generic_shared_cell,
		&generic_shared_cell, &cell_to_1, &cell_to_2, };

const slotframe_t minimum_slotframe = { 0, 101, 8, minimum_cells };

//to initialize radio sfd counter and synchronize it with rtimer

void cc2420_arch_sfd_sync(rtimer_clock_t start_time);
#include "dev/leds.h"
static slotframe_t const * current_slotframe;
static volatile struct pt mpt;
static volatile struct rtimer t;
static volatile rtimer_clock_t start;
#include "net/netstack.h"
volatile unsigned char we_are_sending = 0;
/*---------------------------------------------------------------------------*/
static cell_t *
get_cell(uint16_t timeslot)
{
	return (timeslot >= current_slotframe->on_size) ?
	NULL : current_slotframe->cells[timeslot];
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_next_on_timeslot(uint16_t timeslot)
{
	return (timeslot >= current_slotframe->on_size - 1) ? 0 : timeslot + 1;
}
/*---------------------------------------------------------------------------*/
static int
powercycle(struct rtimer *t, void *ptr);
/* Schedule a wakeup from a reference time for a specific duration.
 * Provides basic protection against missed deadlines and timer overflows */
static uint8_t
schedule_fixed(struct rtimer *t, rtimer_clock_t ref_time,
		rtimer_clock_t duration)
{
	int r, ret=1;
	rtimer_clock_t now = RTIMER_NOW() + 1;
	ref_time += duration;
	if (ref_time - now > duration) {
//		COOJA_DEBUG_PRINTF("Missed deadline or timer overflow deadline %x < now %x\n", ref_time-now, duration);
		COOJA_DEBUG_STR("schedule_fixed: missed deadline!\n");
		// XXX! fix so missed deadline does not corrupt the whole schedule
		ref_time = RTIMER_NOW() + 1;
		ret=0;
	}

	r = rtimer_set(t, ref_time, 1, (void
	(*)(struct rtimer *, void *)) powercycle, NULL);
	if (r != RTIMER_OK) {
		COOJA_DEBUG_STR("schedule_fixed: could not set rtimer\n");
		ret*=2;
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
int	cc2420_address_decode(uint8_t enable);
static volatile uint8_t waiting_for_radio_interrupt = 0;
extern volatile rtimer_clock_t rx_end_time;
/*---------------------------------------------------------------------------*/
int
tsch_resume_powercycle()
{
	if (waiting_for_radio_interrupt || rx_end_time != 0) {
		waiting_for_radio_interrupt = 0;
		schedule_fixed(&t, RTIMER_NOW(), 5);
	}
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
powercycle(struct rtimer *t, void *ptr)
{
	/* if timeslot for tx, and we have a packet, call timeslot_tx
	 * else if timeslot for rx, call timeslot_rx
	 * otherwise, schedule next wakeup
	 */
	PT_BEGIN(&mpt);
	static uint16_t timeslot = 0;
	static cell_t * cell = NULL;
  static struct TSCH_packet* p = NULL;
  static struct neighbor_queue *n = NULL;
	start = RTIMER_NOW();
	//while MAC-RDC is not disabled, and while its synchronized
	while (ieee154e_vars.is_sync && ieee154e_vars.state != TSCH_OFF) {
		COOJA_DEBUG_STR("Cell start\n");
		leds_on(LEDS_GREEN);
		cell = get_cell(timeslot);
		if (cell == NULL || working_on_queue) {
			COOJA_DEBUG_STR("Off cell\n");
			//off cell
			off(keep_radio_on);
		} else {
			hop_channel(cell->channel_offset);
	    p = NULL;
      n = NULL;
			if (cell->link_options & LINK_OPTION_TX) {
				//is there a packet to send? if not check if it is RX too
				if (cell->link_type == LINK_TYPE_ADVERTISING) {
				//TODO fetch adv packets
				} else { //NORMAL link
				//TODO use neighbor table / map
          n=list_head(neighbor_list);
					while (n != NULL) {
						if (rimeaddr_cmp(&n->addr, cell->node_address)) {
              p = read_packet_from_queue(&n->addr);
							break;
						}
						n = list_item_next(n);
					}
				}
			}
				//Is there a packet to send?
				if ((cell->link_options & LINK_OPTION_TX) && p != NULL) {
        COOJA_DEBUG_STR("TO TRANSMIT\n");
				//timeslot_tx(t, start, msg, MSG_LEN);
				{
					// if dedicated slot or shared slot and BW_value=0, we transmit the packet
					if(!(cell->link_options & LINK_OPTION_SHARED) || ((cell->link_options & LINK_OPTION_SHARED) && n->BW_value == 0)){
					static void * payload = NULL;
					static unsigned short payload_len = 0;
					payload = queuebuf_dataptr(p->pkt);
					payload_len = queuebuf_datalen(p->pkt);
					//TODO There are small timing variations visible in cooja, which needs tuning
					static uint8_t is_broadcast = 0, len, seqno, ret;
					uint16_t ack_sfd_time = 0;
					rtimer_clock_t ack_sfd_rtime = 0;
					is_broadcast = rimeaddr_cmp(cell->node_address, &rimeaddr_null);
          //COOJA_DEBUG_STR("HERE\n");
          if(!is_broadcast){
          COOJA_DEBUG_STR("NO TX BROADCAST\n");
          }
					we_are_sending = 1;
					char* payload_ptr = payload;
					//read seqno from payload!
					seqno = payload_ptr[2];

					//prepare packet to send
					//COOJA_DEBUG_STR("prepare tx\n");
					uint8_t success = !NETSTACK_RADIO.prepare(payload, payload_len);
					//COOJA_DEBUG_STR("prepare tx done\n");
					//delay before CCA
					schedule_fixed(t, start, TsCCAOffset);
					PT_YIELD(&mpt);
					on();
					//CCA
					uint8_t cca_status = 0;
					BUSYWAIT_UNTIL_ABS(!(cca_status |= NETSTACK_RADIO.channel_clear()),
							start, TsCCAOffset + TsCCA);
//					off(keep_radio_on);
					if (cca_status == 0) {
						success = RADIO_TX_COLLISION;
					} else {
						//delay before TX
						schedule_fixed(t, start, TsTxOffset - delayTx);
						PT_YIELD(&mpt);
						//send
						static rtimer_clock_t tx_time;
						tx_time = RTIMER_NOW();
						success = NETSTACK_RADIO.transmit(payload_len);
//						off(keep_radio_on);
						//tx_time = (111 * payload_len)/100; //110*payload_len/10=32768*337*payload_len/1000000; //sec
						tx_time = RTIMER_NOW() - tx_time;

						if (success == RADIO_TX_OK) {
							//uint8_t do_ack = (((uint8_t*)(p->ptr))[0] >> 5) & 1 == 1 ;
							if (is_broadcast) {
								//remove_packet_from_queue(cell->node_address);
								COOJA_DEBUG_STR("is_broadcast - don't wait for ack\n");
							} else {
								//delay wait for ack: after tx
								COOJA_DEBUG_STR("wait for ack\n");
								schedule_fixed(t, start,
										TsTxOffset + MIN(tx_time, wdDataDuration) + TsTxAckDelay
												- TsShortGT);
								PT_YIELD(&mpt);
								COOJA_DEBUG_STR("wait for detecting ack\n");
								waiting_for_radio_interrupt = 1;
								on();
								cca_status=0;
								cca_status |= NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() || !NETSTACK_RADIO.channel_clear();
								if(!cca_status) {
									schedule_fixed(t, start,
											TsTxOffset + MIN(tx_time, wdDataDuration) + TsTxAckDelay
													+ TsShortGT);
									PT_YIELD(&mpt);
									cca_status |= NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() || !NETSTACK_RADIO.channel_clear();
								}
								//wait for detecting ACK
//								BUSYWAIT_UNTIL_ABS(
//										(cca_status |= (NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() || NETSTACK_RADIO.channel_clear() == 0)),
//										start, TsTxOffset + MIN(tx_time, wdDataDuration) + TsTxAckDelay + TsShortGT);
								if (cca_status) {
									COOJA_DEBUG_STR("ACK detected\n");
									uint8_t ackbuf[ACK_LEN + EXTRA_ACK_LEN];
									if(!NETSTACK_RADIO.pending_packet()) {
										COOJA_DEBUG_STR("not pending_packet\n");
										if(NETSTACK_RADIO.receiving_packet()) {
											COOJA_DEBUG_STR("receiving_packet\n");
											ack_sfd_rtime = RTIMER_NOW();
											ack_sfd_time = MIN(get_sfd_start_time(), ack_sfd_rtime);
											schedule_fixed(t, start,
													TsTxOffset + MIN(tx_time, wdDataDuration) +
													TsTxAckDelay + TsShortGT + wdAckDuration);
											PT_YIELD(&mpt);
										} else {
//											schedule_fixed(t, start,
//													TsTxOffset + MIN(tx_time, wdDataDuration) +
//													TsTxAckDelay + TsShortGT + wdAckDuration);
//											PT_YIELD(&mpt);
										}

										int	read_ack(void *buf, int);
										int cc2420_pending_irq(void);
										int cc2420_interrupt(void);
										if(cc2420_pending_irq() || !NETSTACK_RADIO.pending_packet()) {
											COOJA_DEBUG_STR("ACK read_ack:\n");
											len = read_ack(ackbuf, ACK_LEN + EXTRA_ACK_LEN);
										}
									} else {
										COOJA_DEBUG_STR("ACK Read:\n");
										len = NETSTACK_RADIO.read(ackbuf, ACK_LEN + EXTRA_ACK_LEN);
									}
									if ((len == ACK_LEN + EXTRA_ACK_LEN || len == ACK_LEN) && seqno == ackbuf[2]) {
										//TODO get timing information
										success = RADIO_TX_OK;
										COOJA_DEBUG_STR("ACK ok\n");
									} else {
										success = RADIO_TX_NOACK;
										COOJA_DEBUG_STR("ACK not ok!\n");
										COOJA_DEBUG_PRINTF("%u: %x, %x, %x ?= %x, %x, %x, %x, %x\n", len, ackbuf[0], ackbuf[1],ackbuf[2],seqno,ackbuf[3],ackbuf[4],ackbuf[5],ackbuf[6]);
									}
								} else {
									COOJA_DEBUG_STR("No ack!\n");
								}
								waiting_for_radio_interrupt = 0;
							}
							we_are_sending = 0;
							off(keep_radio_on);
							COOJA_DEBUG_STR("end tx slot\n");
						}
					}

					if (success == RADIO_TX_NOACK) {
						p->transmissions++;
						if (p->transmissions == macMaxFrameRetries) {
							remove_packet_from_queue(cell->node_address);
					    n->BE_value=macMinBE;
							n->BW_value=0;
						}
						if((cell->link_options & LINK_OPTION_SHARED) & !is_broadcast){
                    unsigned int window = 1 << n->BE_value;
						        n->BW_value = random_rand()%window;
						        n->BE_value++;
							if(n->BE_value > macMaxBE){ n->BE_value = macMaxBE;}
						}
						ret = MAC_TX_NOACK;
					} else if (success == RADIO_TX_OK) {
						//TODO synchronize using ack_sfd_rtime or ack_sfd_time
						remove_packet_from_queue(cell->node_address);
						if(!read_packet_from_queue(cell->node_address)){
						   // if no more packets in the queue
						   n->BW_value = 0;
						   n->BE_value = macMinBE;
						}
						else{
						  // if queue is not empty
						   n->BW_value = 0;
						}
						ret = MAC_TX_OK;
					} else if (success == RADIO_TX_COLLISION) {
						p->transmissions++;
						if (p->transmissions == macMaxFrameRetries) {
							remove_packet_from_queue(cell->node_address);
							n->BE_value=macMinBE;
							n->BW_value=0;
						}
						if((cell->link_options & LINK_OPTION_SHARED) & !is_broadcast){
						        unsigned int window = 1 << n->BE_value;
						        n->BW_value = random_rand()%window;
						        n->BE_value++;
							if(n->BE_value > macMaxBE){ n->BE_value = macMaxBE;}
						}
						ret = MAC_TX_COLLISION;
					} else if (success == RADIO_TX_ERR) {
						p->transmissions++;
						if (p->transmissions == macMaxFrameRetries) {
							remove_packet_from_queue(cell->node_address);
							n->BE_value=macMinBE;
							n->BW_value=0;
						}
						if((cell->link_options & LINK_OPTION_SHARED) & !is_broadcast){
						         unsigned int window = 1 << n->BE_value;
						         n->BW_value = random_rand()%window;
                     n->BE_value++;
							if(n->BE_value > macMaxBE){ n->BE_value = macMaxBE;}
						}
						ret = MAC_TX_ERR;
					} else {
						// successful transmission
						remove_packet_from_queue(cell->node_address);
						if(!read_packet_from_queue(cell->node_address)){
						   // if no more packets in the queue
						   n->BW_value = 0;
						   n->BE_value = macMinBE;
						}
						else{
						  // if queue is not empty
						   n->BW_value = 0;
						}
						ret = MAC_TX_OK;
					}
					//XXX callback -- do we need to restore packet to packetbuf?
					mac_call_sent_callback(p->sent, p->ptr, ret, p->transmissions);
					}
					else { // packet to transmit but we cannot use shared slot due to backoff counter
					       n->BW_value--;
					}
				}
			} else if (cell->link_options & LINK_OPTION_RX) {
//				timeslot_rx(t, start, msg, MSG_LEN);
				if (cell->link_options & LINK_OPTION_TIME_KEEPING) {
					// TODO
				}
				{
					//TODO There are small timing variations visible in cooja, which needs tuning
					static uint8_t is_broadcast = 0, len, seqno, ret;
					uint16_t ack_sfd_time = 0;
					rtimer_clock_t ack_sfd_rtime = 0;

					is_broadcast = rimeaddr_cmp(cell->node_address, &rimeaddr_null);

					//wait before RX
					schedule_fixed(t, start, TsTxOffset - TsLongGT);
					COOJA_DEBUG_STR("schedule RX on guard time - TsLongGT");
					PT_YIELD(&mpt);
					//Start radio for at least guard time
					on();
					COOJA_DEBUG_STR("RX on -TsLongGT");
					uint8_t cca_status = 0;
					cca_status = (!NETSTACK_RADIO.channel_clear()
							|| NETSTACK_RADIO.pending_packet()
							|| NETSTACK_RADIO.receiving_packet());
					//Check if receiving within guard time
					schedule_fixed(t, start, TsTxOffset + TsLongGT);
					PT_YIELD(&mpt);
					COOJA_DEBUG_STR("RX on +TsLongGT");

					if (!(rx_end_time || cca_status || NETSTACK_RADIO.pending_packet()
							|| !NETSTACK_RADIO.channel_clear()
							|| NETSTACK_RADIO.receiving_packet())) {
						COOJA_DEBUG_STR("RX no packet in air\n");
						off(keep_radio_on);
						//no packets on air
						ret = 0;
					} else {
						if (rx_end_time == 0 && (!NETSTACK_RADIO.pending_packet())) {
							//wait until rx finishes
							schedule_fixed(t, start, TsTxOffset + wdDataDuration + 1);
							waiting_for_radio_interrupt = 1;
							COOJA_DEBUG_STR("Wait until RX is done");
							PT_YIELD(&mpt);
						}
						COOJA_DEBUG_STR("RX is finished");
						off(keep_radio_on);
						//wait until ack time
						extern volatile struct received_frame_s *last_rf;
						if(last_rf) { //received something and not out of memory
							COOJA_DEBUG_STR("last_rf != NULL");
						} else {
							COOJA_DEBUG_STR("last_rf = NULL");
						}
						if(is_broadcast) {
							COOJA_DEBUG_STR("RX is_broadcast cell");
						}
						extern volatile uint8_t last_acked;
						if (last_acked && !is_broadcast) {
							COOJA_DEBUG_STR("last_rf->acked");
							schedule_fixed(t, rx_end_time, TsTxAckDelay - delayTx);
							PT_YIELD(&mpt);
							COOJA_DEBUG_STR("send_ack()");
							void send_ack(void);
							send_ack();
							rx_end_time = 0;
						}
						//XXX return length instead? or status? or something?
						ret = 1;
					}
				}
			}
		}
		//TODO incorporate ack time -sync- correction
		uint16_t dt, duration, next_timeslot;
		next_timeslot = get_next_on_timeslot(timeslot);
		dt =
				next_timeslot ? next_timeslot - timeslot :
						current_slotframe->length - timeslot;
		duration = dt * TsSlotDuration;

		//XXX correct on timeslot boundaries
		static uint16_t correction = 5;
		if (!next_timeslot) {
			if (RTIMER_NOW() < start + duration - correction + 1) {
				COOJA_DEBUG_STR("New slot frame: no overflow");
				correction = 5;
				duration -= correction;
			} else {
				correction += 5;
				COOJA_DEBUG_STR("New slot frame: no correction!!");
			}
		}
//		COOJA_DEBUG_PRINTF("Schedule next ON slot now 0x%x, deadline 0x%x", RTIMER_NOW(),start);
		timeslot = next_timeslot;
		ieee154e_vars.asn += dt;
		schedule_fixed(t, start, duration);
		start += duration;
		leds_off(LEDS_GREEN);
		PT_YIELD(&mpt);
	}
	COOJA_DEBUG_STR("TSCH is OFF!!");
	PT_END(&mpt);
}
/*---------------------------------------------------------------------------*/
void
tsch_associate(void)
{
	/* TODO Synchronize
	 * If we are a master, start right away
	 * otherwise, wait for EBs to associate with a master
	 */
	COOJA_DEBUG_STR("tsch_associate\n");
	waiting_for_radio_interrupt = 0;
	we_are_sending = 0;
	working_on_queue = 0;
	//for now assume we are in sync
	ieee154e_vars.is_sync = 1;
	//something other than 0 for now
	ieee154e_vars.state = TSCH_ASSOCIATED;
	start = RTIMER_NOW();
	schedule_fixed(&t, start, TsSlotDuration);
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
	current_slotframe = &minimum_slotframe;
	ieee154e_vars.asn = 0;
	ieee154e_vars.captured_time = 0;
	ieee154e_vars.dsn = 0;
	ieee154e_vars.is_sync = 0;
	ieee154e_vars.state = 0;
	ieee154e_vars.sync_timeout = 0; //30sec/slotDuration - (asn-asn0)*slotDuration
	memb_init(&neighbor_memb);

	//schedule next wakeup? or leave for higher layer to decide? i.e, scan, ...
	tsch_associate();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tsch_tx_callback_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("tsch_tx_callback_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);


    PRINTF("tsch_tx_callback_process: calling mac tx callback\n");
  	COOJA_DEBUG_STR("tsch_tx_callback_process: calling mac tx callback\n");

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver tschrdc_driver = { "tschrdc", init, send_packet,
	send_list, packet_input, on, off, channel_check_interval, };
/*---------------------------------------------------------------------------*/
