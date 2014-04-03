#ifndef __TSCH_PARAMETERS_H__
#define __TSCH_PARAMETERS_H__
#include "net/rime/rimeaddr.h"

#define NACK_FLAG 0x8000
typedef uint32_t asn_t;

typedef struct {
	asn_t asn;                // current absolute slot number
	uint8_t state;              // state of the FSM
	uint8_t dsn;                // data sequence number
	uint16_t captured_time;       // last captures time
	uint16_t sync_timeout;        // how many slots left before looses sync
	uint8_t is_sync;             // TRUE iff mote synchronized to network
	uint8_t mac_ebsn;						//EB sequence number
	uint8_t join_priority;			//inherit from RPL - for PAN coordinator: 0 -- lower is better
//   OpenQueueEntry_t*  dataToSend;         // pointer to the data to send
//   OpenQueueEntry_t*  dataReceived;       // pointer to the data received
//   OpenQueueEntry_t*  ackToSend;          // pointer to the ack to send
//   OpenQueueEntry_t*  ackReceived;        // pointer to the ack received
} ieee154e_vars_t;

// Atomic durations
// expressed in 32kHz ticks:
//    - ticks = duration_in_seconds * 32768
//    - duration_in_seconds = ticks / 32768

//XXX check these numbers on real hw or cooja
//164*3 as PORT_TsSlotDuration causes 147.9448us drift every slotframe ==4.51 ticks
// 15000us
#define PORT_TsSlotDuration (164*3)
//   600us
#define PORT_maxTxDataPrepare (19)
//   500us
#define PORT_maxRxAckPrepare (16)
//   500us
#define PORT_maxRxDataPrepare (16)

#define PORT_maxTxAckPrepare (16)

// ~327us+129preample
#define PORT_delayTx (11)
//~50us delay + 129preample + ??
#define PORT_delayRx (9)

enum ieee154e_atomicdurations_enum {
	// time-slot related
	TsCCAOffset=98,										//3000us
	TsCCA=14,												//~500us
	TsRxTx=16,												//500us
	TsTxOffset = 131,                  //  4000us
	TsLongGT = 43,                  //  1300us
	TsTxAckDelay = 131,                  //  4000us
	TsShortGT = 16,                  //   500us
	TsSlotDuration = PORT_TsSlotDuration,  // 15000us
	// execution speed related
	maxTxDataPrepare = PORT_maxTxDataPrepare,
	maxRxAckPrepare = PORT_maxRxAckPrepare,
	maxRxDataPrepare = PORT_maxRxDataPrepare,
	maxTxAckPrepare = PORT_maxTxAckPrepare,
	// radio speed related
	delayTx = PORT_delayTx,         // between GO signal and SFD: radio fixed delay + 4Bytes preample + 1B SFD -- 1Byte time is 32us
	delayRx = PORT_delayRx,         // between GO signal and start listening
	// radio watchdog
	wdRadioTx = 33,                  //  1000us (needs to be >delayTx)
	wdDataDuration = 148,            //  4500us (measured 4280us with max payload)
	wdAckDuration = 25,                  //  750us (measured 1000us me: 440us)
};

enum ieee154e_states_enum {
	TSCH_OFF = 0, TSCH_ASSOCIATED = 1,
};

enum slotframe_operations_enum {
	ADD_SLOTFRAME = 0, DELETE_SLOTFRAME = 2, MODIFY_SLOTFRAME = 3,
};

enum link_operations_enum {
	ADD_LINK = 0, DELETE_LINK = 1, MODIFY_LINK = 2,
};

enum link_options_enum {
	LINK_OPTION_TX=1,
	LINK_OPTION_RX=2,
	LINK_OPTION_SHARED=4,
	LINK_OPTION_TIME_KEEPING=8,
};

enum link_type_enum {
	LINK_TYPE_NORMAL=0,
	LINK_TYPE_ADVERTISING=1,
};

enum cell_decision_enum {
	CELL_OFF=0,
	CELL_TX=1,
	CELL_TX_IDLE=2, //No packet to transmit
	CELL_TX_BACKOFF=3,  //csma backoff
	CELL_RX=4,
};

typedef struct {
	/* Unique identifier (local to specified slotframe) for the link */
	uint16_t link_handle;
	/* Relative number of slot in slotframe */
	//uint16_t timeslot;
	/* maybe 0 to 15 */
	uint8_t channel_offset;
	/*b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.*/
	uint8_t link_options;
	/* Type of link. NORMAL = 0. ADVERTISING = 1, and indicates
	the link may be used to send an Enhanced beacon. */
	uint8_t link_type;
	/* short address of neighbor */
	rimeaddr_t* node_address;
} cell_t;

typedef struct {
	/* Unique identifier */
	uint16_t slotframe_handle;
	uint16_t length;
	uint16_t on_size;
	cell_t ** cells;
} slotframe_t;
#define TSCH_MAX_PACKET_LEN 127

#endif /* __TSCH_PARAMETERS_H__ */
