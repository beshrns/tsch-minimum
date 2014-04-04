#ifndef PROJECT_H_
#define PROJECT_H_

#define DISABLE_COOJA_DEBUG 0
//#undef WITH_UIP6
//#define WITH_UIP6 0

//#undef UIP_CONF_LOGGING
//#define UIP_CONF_LOGGING 1
#undef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED 0

#undef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL 25

#undef CC2420_CONF_SFD_TIMESTAMPS
#define CC2420_CONF_SFD_TIMESTAMPS 1

#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     tschrdc_driver

#undef UIP_CONF_ND6_SEND_NA
#define UIP_CONF_ND6_SEND_NA 0

#undef UIP_CONF_ND6_SEND_RA
#define UIP_CONF_ND6_SEND_RA 0

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 4 /* should be a power of two */

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS	20

#endif /* PROJECT_H_ */

