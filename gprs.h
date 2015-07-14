/**
 * @file    gprs.h
 * @brief   Gprs interface for lwIP/ppp_new()
 *
 * lwipopts.h Requirement :
 *
 * - PPP_SUPPORT            1
 * - PPP_INPROC_OWNTHREAD   0
 * - GPRS_SUUPPORT          1
 *
 * Other requirement :
 * 
 *  - Architecture dependent sio implementation.
 *  - gprs_arch_modem_on() & gprs_arch_modem_off() to turn the modem on
 *  off.
 *  
 *  For other configuration options check gprs.h
 *
 * <pre>
 * Example to be include in your main after lwIP initialization :
 *
 * ...
 *  gprs_t * gprs;
 *
 *  ppp_init();				// Initialize the PPP sub systems
 *  gprs = gprs_new(3);		// Create a new gprs interface on serial port # 3
 *	gprs_open(gprs);		// Start the gprs connection
 * ...
 *</pre>
 *
 * @author  Mario Viara
 * @version 1.00
 * 
 * @copyright   Copyright Mario Viara 2015  - License Open Source (LGPL)
 * This is a free software and is opened for education, research and commercial
 * developments under license policy of following terms:
 * - This is a free software and there is NO WARRANTY.
 * - No restriction on use. You can use, modify and redistribute it for personal,
 *   non-profit or commercial product UNDER YOUR RESPONSIBILITY.
 * - Redistributions of source code must retain the above copyright notice.
 * 
 */
#ifndef GPRS_H
#define GPRS_H
#include "lwip/opt.h"
#include "lwip/sio.h"
#include "lwip/timers.h"
#include "lwip/tcpip.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppos.h"

#ifdef  __cplusplus
extern "C" {
#endif


/**
 * As default GPRS is not configured
 */
#ifndef GPRS_SUPPORT
#define GPRS_SUPPORT    0
#endif


/**
 * Include only if configured
 */
#if GPRS_SUPPORT

/**
 * GPRS debug options
 */
#ifndef GPRS_DEBUG
#define GPRS_DEBUG	LWIP_DBG_OFF
#endif


/**
 * Modem debug options
 */
#ifndef MODEM_DEBUG
#define	MODEM_DEBUG	LWIP_DBG_OFF
#endif



/**
 * Is set GPRS will run in own thread otherwise the function
 * gprs_input() must be called when data are available from the serial port.
 */
#ifndef GPRS_OWNTHREAD
#define GPRS_OWNTHREAD  1
#endif

/**
 * If multi threading is used define the platform specific thread options.
 */
#if	GPRS_OWNTHREAD
#ifndef GPRS_THREAD_NAME
#define	GPRS_THREAD_NAME	"GPRS"
#endif

#ifndef	GPRS_THREAD_STACKSIZE
#define	GPRS_THREAD_STACKSIZE	0
#endif

#ifndef	GPRS_THREAD_PRIO
#define	GPRS_THREAD_PRIO		1
#endif

#endif


/**
 * Buffer size for the reply buffer of the modem
 */ 
#ifndef GPRS_REPLY_BUFFER_SIZE
#define GPRS_REPLY_BUFFER_SIZE  128
#endif

/**
 * How much milliseconds the modem must be power off
 */
#ifndef GPRS_MODEM_OFF_TIME
#define GPRS_MODEM_OFF_TIME 1500
#endif

/**
 * Delay before using the modem after power on
 */
#ifndef GPRS_MODEM_ON_DELAY
#define GPRS_MODEM_ON_DELAY 2500
#endif

/**
 * Buffer size of serial interface
 */
#ifndef	GPRS_MODEM_BUFFER_SIZE
#define GPRS_MODEM_BUFFER_SIZE	1024
#endif

/**
 * Delay between command during init
 */
#ifndef GPRS_INIT_COMMAND_DELAY	
#define GPRS_INIT_COMMAND_DELAY	500
#endif

/**
 * Timeout in milliseconds for modem commands.
 */
#ifndef GPRS_COMMAND_TIMEOUT
#define GPRS_COMMAND_TIMEOUT  500
#endif


/**
 * Timeout in milliseconds for data connection
 */
#ifndef GPRS_DIAL_TIMEOUT
#define GPRS_DIAL_TIMEOUT   120000
#endif

/**
 * Timeout for network registration
 */
#define	GPRS_REGISTRATION_TIMEOUT	60000


/**
 * Timeout for modem alive
 */
#define	GPRS_ALIVE_TIMEOUT			10000
/**
 * Operator APN
 */
#ifndef GPRS_APN
#define GPRS_APN          "internet"
#endif

/**
 * If set the APN can be changed in runtime
*/
#ifndef GPRS_RUNTIME_APN
#define GPRS_RUNTIME_APN    0
#endif


/**
 * If required define the maximum size of the APN
 */
#if GPRS_RUNTIME_APN
#ifndef GPRS_APN_SIZE
#define GPRS_APN_SIZE        16
#endif
#endif


/**
 * MC55i require a minimum delay of 100 milliseconds between command
 * set to 0 to disable this feature.
 */
#ifndef	GPRS_COMMAND_DELAY
#define	GPRS_COMMAND_DELAY	1000
#endif



/**
 * Not registered
 *
 * - no SIM card available
 * - no PIN entered
 * - no valid PLMN entry found on the SIM
 */
#define GPRS_NETWORK_ERROR  0

/**
 * Registered in the home network
 */
#define GPRS_NETWORK_HOME       1

/**
 * Not registered searching for new operator
 */
#define GPRS_NETWORK_SEARCH     2

/**
 * Not registered registration denied
 */
#define GPRS_NETWORK_DENIED     3

/**
 * Not registered unknown state
 */
#define GPRS_NETWORK_UNKNOWN    4

/**
 * Registered in roaming network
 */
#define GPRS_NETWORK_ROAMING    5

#define GPRS_NETWORK_INVALID	0xFF

#define GSM_CSQ_INVALID			0xFF

/**
 * Modem command state
 */
typedef enum
{
	/**
	 * Not managed state
	 */
	MODEM_STATE_NONE,

	/**
	 * Check the echo of the sent command.
	 */
	MODEM_STATE_ECHO,

	/**
	 * Wait for EOL after echo
	 */
	MODEM_STATE_WAITEOL,

	/**
	 * check reply from the modem
	 */
	MODEM_STATE_REPLY
			
} modem_state_t;

/**
 * GPRS Modem state
 */
typedef enum
{
	GPRS_STATE_MODEM_OFF,
	GPRS_STATE_MODEM_ON,
	GPRS_STATE_MODEM_WAKEUP,
	GPRS_STATE_MODEM_ALIVE,
	GPRS_STATE_MODEM_IDENTIFY,
	GPRS_STATE_MODEM_IMEI,
	GPRS_STATE_GSM_NETWORK,
	GPRS_STATE_GPRS_NETWORK,
	GPRS_STATE_MODEM_INIT,
	GPRS_STATE_MODEM_DIAL,
	GPRS_STATE_CONNECTING,
	GPRS_STATE_CONNECTED,
	GPRS_STATE_DISCONNECTED,
} gprs_state_t;


/**
 * Structure to hold GPRS data
 */
typedef struct
{
	/**
	 * Sio device numebr
	 */
	u8_t            device;

	/**
	 * GSM signal quality
	 */
	u8_t			csq;
	
	/**
	 * CGREG flag if set the GPRS network registration status is checked
	 */
	u8_t			cgreg;

	/**
	 * GPRS network registration code
	 */
	u8_t			gprsNetwork;

	/**
	 * Set when the dial command have CONNECT as reply
	 */
	u8_t			connected;

	/**
	 * GSM network registration code
	 */
	u8_t			gsmNetwork;

	/**
	 * Set from the user to enable roaming
	 */
	u8_t			roaming;
	
	/**
	 * Time of the last change state
	 */
	u32_t			stateTime;

#if GPRS_COMMAND_DELAY > 0
	u8_t			delayedEol;

	/**
	 * Time of the last sent command.
	 */
	u32_t			commandTime;

	/**
	 * Command to be sent after the delay
	 */
	const char *	delayedCommand;
#endif

	/**
	 * Point to last sent command
	 */
	const char *	sentCommand;
	
	/**
	 * Sio device descriptor
	 */
	sio_fd_t        fd;

	/**
	 * Internal state machine
	 */
	gprs_state_t    state;

	/**
	 * Modem state machine
	 */
	modem_state_t	modem;

	/**
	 * Modem state machine number of processed char
	 */
	unsigned		modemCount;

	/**
	 * Internal machine sub state code
	 */
	int				subState;

	/**
	 * Buffer for reply from the modem
	 */
	char			replyBuffer[GPRS_REPLY_BUFFER_SIZE];

	/**
	 * Current pointer in the replyBuffer
	 */
	char	*		replyPtr;

	/**
	 * Runtime APN
	 */
#if GPRS_RUNTIME_APN
    char            apn[GPRS_APN_SIZE];
#endif
	
	/**
	 * Modem IMEI
	 */
	char			imei[15+1];
	
	/**
	 * PPP structure
	 */
	ppp_pcb         *pcb;

	/**
	 * PP network interface
	 */
	struct netif	pppif;

	
#if GPRS_OWNTHREAD
	/**
	 * GPRS thread
	 */
	sys_thread_t    thread;

	/**
	 * Serial recv buffer
	 */
	u8_t			recvBuffer[GPRS_MODEM_BUFFER_SIZE];

	/**
	 * Recv buffer length
	 */
	u32_t			recvLen;

	/**
	 * sempahore to process received data
	 */
	sys_sem_t		recvSem;
#endif

} gprs_t;


#if GPRS_OWNTHREAD == 0
/**
 * Called when data are available from the serial port
 *
 * @param gprs- Gprs descriptor.
 * @param data- Pointer to the data
 * @param length - Length of the data.
 */
void gprs_input(gprs_t * gprs,u8_t * data,u32_t length);
#endif

/**
 * Create a GPRS Control modem
 *
 * @param device Device number for sio functions.
 *
 * @return A pointer to GPRS control or 0 on error.
 */
gprs_t *   gprs_new(u8_t device);

/**
 * Open a new GPRS connection
 *
 * @param gprs - GPRS control returned from a call of gprs_new()
 */
void gprs_open(gprs_t * gprs);

/**
 * Close a GPRS connection
 * 
 * @param gprs - GPRS control returned from a call of gprs_new()
 */
void gprs_close(gprs_t * gprs);


/**
 * Disconnect the  GPRS
 * 
 * @param gprs - GPRS control returned from a call of gprs_new()
 */
void gprs_disconnect(gprs_t * gprs);


/**
 * Connect the  GPRS
 * 
 * @param gprs - GPRS control returned from a call of gprs_new()
 */
void gprs_connect(gprs_t * gprs);

/**
 * Return the current network status
 *
 * @return Current network status
 */
int gprs_get_network(gprs_t *gprs);

/**
 * Get the roaming flag
 *
 * If set connection are made also in roaming.
 *
 * @return roaming flag
 */
u8_t gprs_get_roaming(gprs_t * gprs);

/**
 * Set the roaming flag.
 *
 * If not 0 connection are also made in roaming
 *
 * @param roaming - Roaming flag.
 */
void gprs_set_roaming(gprs_t * gprs,u8_t roaming);

/**
 * Get the GSM modem IMEI.
 *
 * @return IMEI
 */
const char * gprs_get_imei(gprs_t * gprs);

/**
 * Get the GSM signal quality before connection is initiated.
 *
 * @return csq (0 - 31)
 */
u8_t gprs_get_csq(gprs_t * gprs);

#if GPRS_RUNTIME_APN

/**
 * Set the APN
 *
 * @param gprs - GPRS control
 * @param name - Name of the new apn
 *
 * @see GPRS_APN_SIZE
 */
void gprs_set_apn(gprs_t *gprs,const char *name);

/**
 * Get the current APN
 *
 * @return The current APN
 */
const char * gprs_get_apn(gprs_t * gprs);

#endif

/**
 * System depending function to turn on the modem
 *
 * @param device - Sio device number.
 */
#ifndef gprs_arch_modem_on
void gprs_arch_modem_on(u8_t device);
#endif

/**
 * System depending function to turo off the modem
 *
 * @param device - Sio device number.
 */

#ifndef gprs_arch_modem_off
void gprs_arch_modem_off(u8_t device);
#endif

#endif  /* GPRS_SUPPPORT */


#ifdef  __cplusplus
}
#endif

#endif  /* GPRS_H */
