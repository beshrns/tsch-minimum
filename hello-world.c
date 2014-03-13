/*---------------------------------------------------------------------------*/
/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#if RIME
///*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/rime.h"
//#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>
#include <string.h>
#include <random.h>
void leds_arch_set(unsigned char);

/*---------------------------------------------------------------------------*/
PROCESS(example_unicast_process, "Example unicast");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	static int count=0;
	printf("%d: %uB unicast message %s received from %d.%d\n",	++count, packetbuf_datalen(),(char *) packetbuf_dataptr(), from->u8[0], from->u8[1]);
}

static const struct unicast_callbacks unicast_callbacks = { recv_uc };
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(example_unicast_process, ev, data)
{
	PROCESS_EXITHANDLER(unicast_close(&uc);)
	PROCESS_BEGIN();

	unicast_open(&uc, 146, &unicast_callbacks);

	while(1) {
		static struct etimer et;
		static rimeaddr_t addr;
		static unsigned char leds=0;
		static char msg[15];

		etimer_set(&et, 2*CLOCK_SECOND + random_rand()%(CLOCK_SECOND));

		sprintf(msg, "%03d NXP: %03d\n", ++leds, leds);

		leds_arch_set( leds );
		packetbuf_copyfrom(msg, strlen(msg));

		addr.u8[0] = 3;
		addr.u8[1] = 0;

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		if(rimeaddr_cmp(&addr, &rimeaddr_node_addr)) {
			addr.u8[0] = 0xde;
			addr.u8[1] = 0xad;
		}
		printf("Sending to %02x.%02x: %s", addr.u8[0], addr.u8[1], msg);
		//does not work with rime!
		//packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 1);
		unicast_send(&uc, &addr);
	}

	PROCESS_END();

}

#else

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include <string.h>
#include "dev/leds.h"
#include "cooja-debug.h"
#include "tsch.h"
#include "net/rime.h"

#define PRINTF COOJA_DEBUG_STR

//void watchdog_periodic(void);
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
#include "net/packetbuf.h"
//#include "net/mac/framer-802154.c"
//extern const struct framer framer_802154;
//void* perpare_raw(rimeaddr_t addr, char* msg, uint8_t len) {
//  packetbuf_copyfrom(msg, len);
//  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &addr);
//  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
//  packetbuf_set_attr(PACKETBUF_ATTR_RELIABLE, 1);
////  packetbuf_set_attr(PACKETBUF_ATTR_ERELIABLE, 1);
//  framer_802154.create();
////  packetbuf_compact();
//  //set ack req in fcf
//  ((uint8_t*)packetbuf_hdrptr())[0] |= (1 << 5) | 1; //4 == FRAME802154_DATAFRAME
////  ((uint8_t*)packetbuf_dataptr())[0] = (1 << 5)| 4; //4 == FRAME802154_DATAFRAME
//
//  return packetbuf_hdrptr();
//}

#define MSG_LEN (127-10)
static char msg[127];
#include "net/netstack.h"

PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();
  COOJA_DEBUG_STR("COOJA_DEBUG_STR hello_world_process\n");
  //sprintf(msg, "Hello, world\n");

	if(rimeaddr_node_addr.u8[0] %2 == 0) {
		//memcpy(msg, perpare_raw(&rimeaddr_null, msg, MSG_LEN), 127);
		NETSTACK_RDC.send(NULL, msg);
	}
//	void tsch_associate(void);
//	tsch_associate();
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif /* RIME */
