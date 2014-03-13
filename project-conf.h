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

#endif /* PROJECT_H_ */
