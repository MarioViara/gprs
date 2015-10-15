/**
 * @file    gprs.c
 * @brief   Gprs interface for lwIP/ppp_new()
 *
 *  
 * @author  Mario Viara
 * @version 1.00
 * 
 * @copyright   Copyright Mario Viara 2014  - License Open Source (LGPL)
 * This is a free software and is opened for education, research and commercial
 * developments under license policy of following terms:
 * - This is a free software and there is NO WARRANTY.
 * - No restriction on use. You can use, modify and redistribute it for personal,
 *   non-profit or commercial product UNDER YOUR RESPONSIBILITY.
 * - Redistributions of source code must retain the above copyright notice.
 * 
 */
#include <string.h>
#include "gprs.h"

#include "lwip/pbuf.h"
#include "lwip/dns.h"

/**
 * Compile only if enabled
 */
#if GPRS_SUPPORT

#define TIMEOUT(x)      sys_timeout(x,gprs_timeout,gprs)
#define UNTIMEOUT()     sys_untimeout(gprs_timeout,gprs)
#define TIMEOUT1(x)     sys_timeout(x,gprs_command_delayed,gprs)
#define UNTIMEOUT1()    sys_untimeout(gprs_command_delayed,gprs)

static void gprs_input_internal(gprs_t * gprs,u8_t * data,u32_t length);
static void gprs_do_start(gprs_t * gprs);


/**
 * Safe strcpy
 */
static void sstrcpy(char *dest,const char * source,size_t size)
{
	size_t length = strlen(source);
	size_t i;

	if (length + 1 > size)
	{
		LWIP_DEBUGF(GPRS_DEBUG,("gprs: sstrcpy() String too short size=%u required=%u\n",size,length+1));
		LWIP_ASSERT("gprs: sstrcpy()",length + 1 <= size);
		length = size - 1;

	}

	for (i = 0 ; i < length ; i++)
	{
		*dest ++ = *source++;
	}

	*dest = 0;


}

/**
 * Safe strcat
 */
static void sstrcat(char *dest,const char*source,size_t size)
{
	size_t length = strlen(dest);

	if (length < size)
	{
		sstrcpy(dest+length,source,size - length);
	}
	else
	{
		LWIP_DEBUGF(GPRS_DEBUG,("gprs: sstrcat() String overflow short size=%u length=%u",size,length));
		LWIP_ASSERT("gprs: sstrcat()",length < size);

	}
}
/**
 * Called when a gprs timeout expire
 *
 * @param arg - Gprs connection
 */
static void gprs_timeout(void * arg);


/**
 * Set the new state of the modem machine.
 */
static void modem_set_state(gprs_t * gprs,modem_state_t state)
{
	gprs->modem = state;

//    LWIP_DEBUGF(GPRS_DEBUG,("gprs: modem_set_state: %d\n",gprs->modem));
    gprs->modemCount = 0;

}

/**
 * No reply received
 */
#define MODEM_REPLY_NONE	0

/**
 * OK reply received
 */
#define	MODEM_REPLY_OK		1

/**
 * Error reply received
 */
#define	MODEM_REPLY_ERROR	2

/**
 * Check reply from the modem.
 *
 * @return  0 - No valid reply received
 * @return  1 - OK received
 * @return 2 - Error received.
 */
static int modem_check_reply(gprs_t * gprs,const u8_t *data,u32_t length,void (*callback)(gprs_t * gprs,const char *reply))
{
	u8_t c;
	int reply = MODEM_REPLY_NONE;


	while (length-- > 0 && reply == MODEM_REPLY_NONE)
	{
		c = *data++;

		switch (gprs->modem)
		{
			default:
			case MODEM_STATE_NONE:
				break;

			case MODEM_STATE_ECHO:
				if (gprs->sentCommand[gprs->modemCount] == c)
				{
					if (gprs->sentCommand[++gprs->modemCount] < 32)
					{

						modem_set_state(gprs,MODEM_STATE_WAITEOL);
					}
				}
				else
				{
					gprs->modemCount = 0;
				}
				break;

			case MODEM_STATE_WAITEOL:
				if (c < 32)
				{
					//LWIP_DEBUGF(GPRS_DEBUG,("Modem_check_reply: echo received\n"));
					modem_set_state(gprs,MODEM_STATE_REPLY);
				}
				break;

			case MODEM_STATE_REPLY:
				if (c < 32)
				{
					if (gprs->modemCount > 0)
					{
						gprs->replyBuffer[gprs->modemCount] = 0;
						LWIP_DEBUGF(MODEM_DEBUG,("gprs: Modem reply '%s'\n",gprs->replyBuffer));
						gprs->modemCount = 0;
						if (!strcmp(gprs->replyBuffer,"OK"))
						{
#if GPRS_COMMAND_DELAY > 0
							gprs->commandTime  = sys_now();
#endif
							//LWIP_DEBUGF(MODEM_DEBUG,("modem_check_reply: OK received\n"));
							reply = MODEM_REPLY_OK;
						}
						else if(!strncmp(gprs->replyBuffer,"ERROR",5))
						{
#if GPRS_COMMAND_DELAY > 0
							gprs->commandTime  = sys_now();
#endif
							LWIP_DEBUGF(MODEM_DEBUG,("gprs: Modem reply ERROR ERROR received\n"));
							reply = MODEM_REPLY_ERROR;
						}
						else
						{

							if (callback != 0)
							{
								(*callback)(gprs,gprs->replyBuffer);
							}
						}

					}
				}
				else
				{
					if (gprs->modemCount < sizeof(gprs->replyBuffer) - 1)
					{
						gprs->replyBuffer[gprs->modemCount++] = c;
					}
					else
					{
						LWIP_DEBUGF(GPRS_DEBUG,("gprs: GPRS_REPLY_BUFFER_SIZE too short"));
					}

				}
				break;
		}
	}

	return reply;
}

/**
 * Check if we have received a OK from the modem.
 *
 * @return 1 if OK is received.
 */
static int modem_check_ok_reply(gprs_t * gprs,const u8_t * data,u32_t length)
{
	return modem_check_reply(gprs,data,length,0) == MODEM_REPLY_OK;

}

/**
 * Output data to the serial port
 *
 * @param data - Data to be sent
 * @param length - Length of ata
 */
static void gprs_serial_output(gprs_t * gprs,u8_t * data,u32_t length)
{
	u32_t sent;

	sent = sio_write(gprs->fd,data,length);

    if (sent != length)
    {
        LWIP_DEBUGF(MODEM_DEBUG,("gprs: send: failed lenth=%u\n",(unsigned)length));
    }

#if GPRS_SERIAL_STAT
    gprs->sentBytes += length;
#endif
}

/**
 * Set the state of the gprs machine.
 *
 * @param newState - New state.
 */
static void gprs_set_state(gprs_t * gprs,gprs_state_t newState)
{
    u32_t now;
    
    if (gprs->state != newState)
    {
        now = sys_now();
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: new state %d elapsed %lu\n",newState,(unsigned long)(now - gprs->stateTime)));


        gprs->stateTime     = now;
        gprs->state         = newState;
        gprs->subState      = 0;
    }


}

/**
 * Return the number of milliseconds elapsed form the start of the current state.
 *
 * @return elapsed milliseconds.
 */
static u32_t gprs_state_elapsed(gprs_t * gprs)
{
	return sys_now() - gprs->stateTime;

}


/**
 * PPP callback
 */
static void gprs_callback(ppp_pcb *pcb,int err_code,void *ctx)
{
    gprs_t * gprs = (gprs_t *)ctx;

    (void)pcb;

    if (err_code)
    {
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: Callback: disconnected (%d)\n",err_code));

        switch (err_code)
        {
            case PPPERR_USER:
                gprs_set_state(gprs, GPRS_STATE_DISCONNECTED);
                break;

            default:
                gprs_do_start(gprs);
                break;
        }

    }
    else
    {
//        UNTIMEOUT();
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: gprs_callback: connected\n"));

        //LWIP_DEBUGF(GPRS_DEBUG,("IP   %s\n",ip_ntoa(&gprs->pcb->addrs.our_ipaddr)));
        //LWIP_DEBUGF(GPRS_DEBUG,("MASK %s\n",ip_ntoa(&gprs->pcb->addrs.netmask)));
        //LWIP_DEBUGF(GPRS_DEBUG,("DNS1 %s\n",ip_ntoa(&gprs->pcb->addrs.dns1)));
        //LWIP_DEBUGF(GPRS_DEBUG,("DNS2 %s\n",ip_ntoa(&gprs->pcb->addrs.dns2)));
        gprs_set_state(gprs, GPRS_STATE_CONNECTED);
        ppp_set_default(gprs->pcb);

        /*
         * Update DNS if configured
         */
#if LWIP_DNS
        //dns_setserver(0,&gprs->pcb->addrs.dns1);
        //dns_setserver(1,&gprs->pcb->addrs.dns2);

#endif
    }

}

#if GPRS_OWNTHREAD
/**
 * Callback function called in the lwIP thread to process incoming data from the serial port
 */
static void gprs_input_callback(void *arg)
{
    gprs_t * gprs = (gprs_t *)arg;

    /* LWIP_DEBUGF(GPRS_DEBUG,("gprs_input_callback: %u\n",p->len));*/
    gprs_input_internal(gprs,(u8_t *)gprs->recvBuffer,gprs->recvLen);
    sys_sem_signal(&gprs->recvSem);
}


/**
 * Dedicated thread to read data from the serial port.
 */
static void gprs_thread(void *arg)
{
    gprs_t * gprs = (gprs_t *)arg;


    LWIP_DEBUGF(GPRS_DEBUG,("gprs: Thread: Started on device %d size %u\n",gprs->device,GPRS_MODEM_BUFFER_SIZE));


    for (;;)
    {
        gprs->recvLen  = sio_read(gprs->fd,(u8_t *)gprs->recvBuffer,GPRS_MODEM_BUFFER_SIZE);

        if (gprs->recvLen > 0)
        {
#if GPRS_SERIAL_STAT
        	gprs->rcvdBtes += gprs->recvLen;
#endif

            if (tcpip_callback(gprs_input_callback,(void *)gprs) != ERR_OK)
            {
                LWIP_DEBUGF(GPRS_DEBUG,("gprs: Thread tcpip_callback() failed\n"));
            }
            else
            {
                sys_sem_wait(&gprs->recvSem);
            }

        }

    }
}
#else
/**
 * Must be called from the user when data are available from the serial port.
 *
 */
void gprs_input(gprs_t * gprs,u8_t * data,u32_t length)
{
    gprs_input_internal(gprs,data,length);
}

#endif


/**
 * PPOS output function
 */
static u32_t gprs_pppos_output(ppp_pcb *pcb,u8_t * data,u32_t length,void *ctx)
{
	gprs_t * gprs = (gprs_t *)ctx;

	(void)pcb;

	gprs_serial_output(gprs,data,length);

	return length;

}
/**
 * Create a new gprs connection
 *
 * @param device - Serial device number.
 */
gprs_t * gprs_new(u8_t device)
{
    gprs_t * gprs;

    gprs = (gprs_t *) mem_malloc(sizeof(gprs_t));

    if (gprs != 0)
    {
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: new on device %d size %u\n",device,(unsigned)sizeof(gprs_t)));
        gprs->device = device;
        gprs->fd = sio_open(device);
        gprs->stateTime = sys_now();

#if GPRS_COMMAND_DELAY > 0
        gprs->commandTime  = gprs->stateTime;
#endif

        gprs->imei[0] = 0;

        gprs->csq = GSM_CSQ_INVALID;


        if (gprs->fd != 0)
        {
            gprs->pcb = pppos_create(&gprs->pppif,gprs_pppos_output,gprs_callback,(void *)gprs);
            
            if (gprs->pcb)
                
            {
                gprs_set_state(gprs,GPRS_STATE_DISCONNECTED);
                
#if GPRS_RUNTIME_APN
                gprs_set_apn(gprs,GPRS_APN);
#endif

#if GPRS_OWNTHREAD
                sys_sem_new(&gprs->recvSem,0);
                gprs->thread = sys_thread_new(GPRS_THREAD_NAME,gprs_thread,(void *)gprs,GPRS_THREAD_STACKSIZE,GPRS_THREAD_PRIO);
#endif
            }
            else
            {
                LWIP_DEBUGF(GPRS_DEBUG,("gprs:  pppos__create failed\n"));
                mem_free(gprs);
                gprs = (gprs_t *)0;
            }
        }
        else
        {
            LWIP_DEBUGF(GPRS_DEBUG,("gprs: _new failed sio_open(%d)\n",device));
            mem_free(gprs);
            gprs = (gprs_t *)0;
        }
    }
    else
    {
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: malloc failed\n"));
    }

    return gprs;
}


/**
 * Send a string to the modem.
 *
 * @param cmd - String to be sent
 * @param eol - If not 0 is sent as line delimiter.
 */
static void gprs_raw_send(gprs_t* gprs,const char * cmd,u8_t eol)
{

    u32_t length;


    length = strlen(cmd);

    LWIP_DEBUGF(MODEM_DEBUG,("gprs: Sent: '%s' (eol=%d)\n",cmd,eol));
    
    gprs_serial_output(gprs,(u8_t *)cmd,length);


    if (eol)
    {
        gprs_serial_output(gprs,&eol,1);
    }

}

#if GPRS_COMMAND_DELAY
/**
 * Callback function for delayed command.
 *
 * Called when the delay between 2 command expire.
 */
static void gprs_command_delayed(void *arg)
{
	//LWIP_DEBUGF(GPRS_DEBUG,("gprs_command_delayed: timeout\n"));
	gprs_t * gprs = (gprs_t *)arg;
	gprs_raw_send(gprs,gprs->delayedCommand,gprs->delayedEol);
	modem_set_state(gprs,MODEM_STATE_ECHO);

}
#endif

/**
 * Start a new gprs connection
 */
static void gprs_do_start(gprs_t * gprs)
{
    UNTIMEOUT();

    LWIP_DEBUGF(GPRS_DEBUG,("gprs: set modem off\n"));

    gprs_arch_modem_off(gprs->device);
    gprs_set_state(gprs,GPRS_STATE_MODEM_OFF);
    TIMEOUT(GPRS_MODEM_OFF_TIME);
}

/**
 * Send a new command to the modem.
 *
 * @param cmd - Pointer to the command
 * @param eol - Optional EOL char
 * @param timeout - Timeout of the operation in milliseconds.
 */
static void gprs_command_timeout(gprs_t * gprs,const char * cmd,u8_t eol,u32_t timeout)
{
#if GPRS_COMMAND_DELAY
	u32_t delay = sys_now() - gprs->commandTime;
#endif

	gprs->sentCommand = cmd;

#if GPRS_COMMAND_DELAY
	if (delay < GPRS_COMMAND_DELAY)
	{
		delay = GPRS_COMMAND_DELAY - delay;
		timeout += delay;
		TIMEOUT(timeout);
		modem_set_state(gprs,MODEM_STATE_NONE);
		//LWIP_DEBUGF(GPRS_DEBUG,("gprs_command_timeout: Delayed %lu milliseconds\n",delay));
		TIMEOUT1(delay);
		gprs->delayedCommand = cmd;
		gprs->delayedEol = eol;

	}
	else
	{
		TIMEOUT(timeout);
		modem_set_state(gprs,MODEM_STATE_ECHO);
		gprs_raw_send(gprs,cmd,eol);
	}
#else
	TIMEOUT(timeout);
	modem_set_state(gprs,MODEM_STATE_ECHO);
	gprs_raw_send(gprs,cmd,eol);

#endif
}

/**
 * Send a command to the modem using the default timeout
 *
 * @param cmd - Pointer to the command
 * @param eol - Optional EOL char
 */
static void gprs_command(gprs_t * gprs,const char *cmd,u8_t eol)
{
	gprs_command_timeout(gprs,cmd,eol,GPRS_COMMAND_TIMEOUT);
}


/**
 * Return the next token from a modem reply.
 *
 * @return 0 If no token are available or a pointer to the token.
 */
static const char * modem_next_token(gprs_t * gprs)
{
    char * ptr = gprs->replyPtr;
    char * result;
    
    /**
     * Skip ' ' and ,
     */
    while (*ptr == ' ' || * ptr == ',')
    {
        ptr++;
    }

    result = ptr;
    
    while (*ptr && *ptr != ' ' && *ptr != ',')
    {
        ptr++;
    }

    if (*ptr)
    {
        *ptr = 0;
        ptr++;
    }

    gprs->replyPtr = ptr;

    //LWIP_DEBUGF(MODEM_DEBUG,("modem_next_token = '%s'\n",result));
    
    return *result ? result : 0;
}

/**
 * Set the command line received from the modem and return a
 * pointer to the fist one.
 *
 *
 * @return 0 or a pointer to the first token.
 */
static const char * modem_first_token(gprs_t * gprs,const char * reply)
{
    gprs->replyPtr = (char *)reply;
    
    return modem_next_token(gprs);
}


/**
 * Process reply from the identify command.
 *
 * This function identify the supported modem and set flag used during the connection.
 *
 * Supported devices :
 *
 *  - SIM800 xxx	Require gprs network registration flag.
 */
static void do_modem_identify_reply(gprs_t * gprs,const char * reply)
{
    LWIP_DEBUGF(GPRS_DEBUG,("gprs: Identify: '%s'\n",reply));

    if (!strncmp(reply,"SIM800",6))
    {
        gprs->cgreg = 1;
    }
    
	}

/**
 * Send the modem identify command.
 */
static void do_modem_identify(gprs_t * gprs)
{
	UNTIMEOUT();

    gprs->cgreg = 0;
	gprs_set_state(gprs,GPRS_STATE_MODEM_IDENTIFY);
	gprs_command(gprs,"ATI",'\r');
}


/**
 * Process reply from the IMEI command
 */
static void do_modem_imei_reply(gprs_t * gprs,const char * reply)
{
    const char * result;

    result = modem_first_token(gprs,reply);

    if (result)
    {

   		sstrcpy(gprs->imei,result,sizeof(gprs->imei));
   		LWIP_DEBUGF(GPRS_DEBUG,("gprs: IMEI='%s'\n",result));
    }
}

/**
 * Send the get IMEI command.
 */
static void do_modem_imei(gprs_t * gprs)
{
	UNTIMEOUT();

	gprs->imei[0] = 0;
	gprs_set_state(gprs,GPRS_STATE_MODEM_IMEI);
	gprs_command(gprs,"AT+GSN",'\r');
}


/**
 * Decode a network status reply from the modem.
 */
static u8_t do_network_decode(gprs_t *gprs, char * reply)
{
    int network = -1;
    const char * result;
    
    result = modem_first_token(gprs,reply);

    if (result)
    {
        result = modem_next_token(gprs);
        if (result)
        {
            result = modem_next_token(gprs);
            if (result)
            {
                network = atoi(result);
            }
        }
    }

    LWIP_DEBUGF(MODEM_DEBUG,("gprs: Decoded network=%d\n",network));
    
    return network;
}

/**
 * Decode a signal quality reply from the modem.
 */
static u8_t do_signal_quality_decode(gprs_t *gprs, char * reply)
{
    int csq = -1;
    const char * result;

    result = modem_first_token(gprs,reply);

    if (result)
    {
        result = modem_next_token(gprs);
        if (result)
        {
            csq = atoi(result);
        }
    }

    LWIP_DEBUGF(MODEM_DEBUG,("gprs: Decoded signal quality=%d\n",csq));

    return csq;
            
    
}

/**
 * Process reply from the gsm registration command.
 */
static void do_gsm_network_reply(gprs_t *gprs,const char *reply)
{
    int network = do_network_decode(gprs,(char *)reply);

    if (network != gprs->gsmNetwork)
    {
        gprs->gsmNetwork = network;
    }
}

/**
 * Process reply from the gsm signal quality command.
 */
static void do_gsm_signal_quality_reply(gprs_t *gprs,const char *reply)
{
	gprs->csq = do_signal_quality_decode(gprs,(char *)reply);
}

/**
 * Send the gprs registration command
 *
 */
static void do_gprs_network(gprs_t * gprs)
{
    UNTIMEOUT();

    gprs_set_state(gprs,GPRS_STATE_GPRS_NETWORK);

    if (gprs->subState++ == 0)
    {
        gprs->gprsNetwork = GPRS_NETWORK_INVALID;

    }

    if (gprs_state_elapsed(gprs) > GPRS_REGISTRATION_TIMEOUT)
    {
    	LWIP_DEBUGF(GPRS_DEBUG,("gprs: GPRS Network registration timeout (%lu)\n",(unsigned long)GPRS_REGISTRATION_TIMEOUT));
    	gprs_do_start(gprs);
    }
    else
    {
    	gprs_command(gprs,"AT+CGREG?",'\r');
    }
}

/**
 * Process reply from the gprs registration command
 */
static void do_gprs_network_reply(gprs_t *gprs,const char *reply)
{
    int network = do_network_decode(gprs,(char *)reply);

    if (network != gprs->gprsNetwork)
    {
        gprs->gprsNetwork = network;
    }
}

/**
 * Process the modem reply for the dial command.
 */
static void do_modem_dial_reply(gprs_t * gprs,const char *reply)
{
	if (!strncmp(reply,"CONNECT",7))
	{
		gprs->connected =1;
	}
}

/**
 * Send the dial command to the modem.
 */
static void do_modem_dial(gprs_t * gprs)
{

	UNTIMEOUT();

    gprs_set_state(gprs,GPRS_STATE_MODEM_DIAL);
    gprs->connected = 0;
    gprs_command_timeout(gprs,"ATD*99***1#",'\r',GPRS_DIAL_TIMEOUT);
}

/**
 * Send the command to initialize the gprs of the modem.
 */
static void do_modem_init(gprs_t * gprs)
{
	UNTIMEOUT();

    gprs_set_state(gprs,GPRS_STATE_MODEM_INIT);
    sstrcpy(gprs->replyBuffer,"AT+CGDCONT=1,\"IP\",\"",sizeof(gprs->replyBuffer));

#if	GPRS_RUNTIME_APN
    sstrcat(gprs->replyBuffer,gprs->apn,sizeof(gprs->replyBuffer));
#else
    sstrcat(gprs->replyBuffer,GPRS_APN,sizeof(gprs->replyBuffer));
#endif

    sstrcat(gprs->replyBuffer,"\"",sizeof(gprs->replyBuffer));

    gprs_command(gprs,gprs->replyBuffer,'\r');

}

static void do_gprs_network_check(gprs_t *gprs,u8_t *data,u32_t length)
{
    if (modem_check_reply(gprs,data,length,do_gprs_network_reply) == MODEM_REPLY_OK)
    {
    	LWIP_DEBUGF(GPRS_DEBUG,("gprs: GPRS Network is %d\n",gprs->gprsNetwork));

        if (gprs->gprsNetwork == GPRS_NETWORK_HOME || (gprs->gprsNetwork == GPRS_NETWORK_ROAMING && gprs->roaming))
        {
        	do_modem_init(gprs);
        }
        else
        {
            do_gprs_network(gprs);
        }
    }
}


/**
 * Send alternatively the signal quality and the gsm network registration commands.
 */
static void do_gsm_network(gprs_t *gprs)
{
	UNTIMEOUT();

    gprs_set_state(gprs,GPRS_STATE_GSM_NETWORK);

    if (gprs->subState++ == 0)
    {
        gprs->gsmNetwork = GPRS_NETWORK_INVALID;
    }

    if (gprs_state_elapsed(gprs) > GPRS_REGISTRATION_TIMEOUT)
    {
    		LWIP_DEBUGF(GPRS_DEBUG,("gprs: GSM Network registration timeout (%lu)\n",(unsigned long)GPRS_REGISTRATION_TIMEOUT));
    		gprs_do_start(gprs);
    }
    else
    {
    	if (gprs->subState & 0x01)
    	{
    		gprs_command(gprs,"AT+CSQ",'\r');
    	}
    	else
    	{
    		gprs_command(gprs,"AT+CREG?",'\r');
    	}
    }
}


static void do_gsm_network_check(gprs_t *gprs,u8_t *data,u32_t length)
{
	if (gprs->subState & 0x01)
	{
		if (modem_check_reply(gprs,data,length,do_gsm_signal_quality_reply) == MODEM_REPLY_OK)
		{
			LWIP_DEBUGF(GPRS_DEBUG,("gprs: GSM Signal Quality is %d\n",gprs->csq));

			do_gsm_network(gprs);
		}
	}
	else
	{
		if (modem_check_reply(gprs,data,length,do_gsm_network_reply) == MODEM_REPLY_OK)
		{
			LWIP_DEBUGF(GPRS_DEBUG,("gprs: GSM Network is %d\n",gprs->gsmNetwork));

			if (gprs->gsmNetwork == GPRS_NETWORK_HOME || (gprs->gsmNetwork == GPRS_NETWORK_ROAMING && gprs->roaming))
			{
				if (gprs->cgreg)
				{
					do_gprs_network(gprs);
				}
				else
				{
					do_modem_init(gprs);

				}
			}
			else
			{
				do_gsm_network(gprs);
			}
		}
	}
}


/**
 * Send the modem alive command.
 */
static void do_modem_alive(gprs_t * gprs)
{
	UNTIMEOUT();

	gprs_set_state(gprs,GPRS_STATE_MODEM_ALIVE);

    if (gprs_state_elapsed(gprs) > GPRS_ALIVE_TIMEOUT)
    {
   		LWIP_DEBUGF(GPRS_DEBUG,("gprs: Modem alive timeout (%lu)\n",(unsigned long)GPRS_ALIVE_TIMEOUT));
		gprs_do_start(gprs);
	}
	else
	{
		gprs_command(gprs,"AT",'\r');
	}
}

/**
 * Send a sequence of command to the modem after power up
 *
 * The reply are not checked and the command only set the modem in the default state.
 */
static void do_modem_wakeup(gprs_t * gprs)
{
    static const char * inits[] = {"AT","AT","AT&F","ATE1"};
    
    gprs_set_state(gprs,GPRS_STATE_MODEM_WAKEUP);

    if (gprs->subState < (int)(sizeof(inits)/sizeof(char *)))
    {
        TIMEOUT(GPRS_INIT_COMMAND_DELAY);
        gprs_raw_send(gprs,inits[gprs->subState++],'\r');
    }
    else
    {
    	do_modem_alive(gprs);
    }
            
}
         

/**
 * Start the PPP connection
 */
static void gprs_do_connect(gprs_t * gprs)
{
	UNTIMEOUT();
	gprs_set_state(gprs,GPRS_STATE_CONNECTING);
	ppp_connect(gprs->pcb,0);
}

/**
 * Handle timeout
 */
static void gprs_timeout(void * arg)
{
    gprs_t * gprs = (gprs_t *)arg;


    switch (gprs->state)
    {
        default:
            LWIP_DEBUGF(GPRS_DEBUG,("gprs: Timeout state %d\n",gprs->state));
            gprs_do_start(gprs);
            break;

        case GPRS_STATE_MODEM_OFF:
            gprs_arch_modem_on(gprs->device);
            LWIP_DEBUGF(GPRS_DEBUG,("gprs: set modem on\n"));
            TIMEOUT(GPRS_MODEM_ON_DELAY);
            gprs_set_state(gprs,GPRS_STATE_MODEM_ON);
            break;

        case GPRS_STATE_MODEM_ON:
        case GPRS_STATE_MODEM_WAKEUP:
            do_modem_wakeup(gprs);
            break;

        case GPRS_STATE_MODEM_ALIVE:
        	do_modem_alive(gprs);
        	break;



    }
}


/**
 * This function process data from the serial interface and must run
 * in the LWIP thread context.
 *
 * @param gprs - Pointer to the gprs structur7e.
 * @param data- Data pointer
 * @param length - Length of the data.
 */

static void gprs_input_internal(gprs_t * gprs,u8_t * data,u32_t length)
{

    if (length > 0)
    {

#if 0
        LWIP_DEBUGF(GPRS_DEBUG,("gprs: Got %lu bytes state=%d\n",(unsigned long)length,gprs->state));
#endif

        switch (gprs->state)
        {
            default:
                LWIP_DEBUGF(GPRS_DEBUG,("gprs: Ignored %lu bytes\n",(unsigned long)length));
                break;

            case GPRS_STATE_MODEM_DIAL:
            	modem_check_reply(gprs,data,length,do_modem_dial_reply);
            	if (gprs->connected)
            	{
            		gprs_do_connect(gprs);
            	}
            	break;

            case GPRS_STATE_GPRS_NETWORK:
                do_gprs_network_check(gprs,data,length);
                break;
                    
            case GPRS_STATE_GSM_NETWORK:
                do_gsm_network_check(gprs,data,length);
                break;

            case GPRS_STATE_MODEM_IMEI:
                if (modem_check_reply(gprs,data,length,do_modem_imei_reply) == MODEM_REPLY_OK)
                {
                    do_gsm_network(gprs);
                }
            	break;

            case GPRS_STATE_MODEM_IDENTIFY:
                if (modem_check_reply(gprs,data,length,do_modem_identify_reply) == MODEM_REPLY_OK)
                {
                    do_modem_imei(gprs);
                }
            	break;

            case GPRS_STATE_MODEM_INIT:
            	if (modem_check_ok_reply(gprs,data,length))
            	{
            		do_modem_dial(gprs);
            	}
            	break;
            case GPRS_STATE_MODEM_ALIVE:
            	if (modem_check_ok_reply(gprs,data,length))
            	{
            		do_modem_identify(gprs);
            	}
            	break;

            case GPRS_STATE_CONNECTING:
            case GPRS_STATE_CONNECTED:
                pppos_input(gprs->pcb,data,length);
                break;

        }
    }
}


/**
 * Open the GPRS connection
 */
void gprs_open(gprs_t * gprs)
{
    gprs_do_start(gprs);
}


#if GPRS_RUNTIME_APN

void gprs_set_apn(gprs_t * gprs,const char *name)
{
	sstrcpy(gprs->apn,name,sizeof(gprs->apn));

    LWIP_DEBUGF(GPRS_DEBUG,("gprs: set_apn='%s'\n",gprs->apn));
}


const char * gprs_get_apn(gprs_t * gprs)
{
    return gprs->apn;
}

#endif


const char * gprs_get_imei(gprs_t * gprs)
{
	return gprs == NULL ? "" : gprs->imei;
}


u8_t gprs_get_csq(gprs_t * gprs)
{
	return gprs == NULL ? GSM_CSQ_INVALID : gprs->csq;
}


void gprs_set_roaming(gprs_t * gprs,u8_t roaming)
{
	gprs->roaming = roaming;
}


#if GPRS_SERIAL_STAT
void gprs_get_stat(gprs_t * gprs,u32_t * sent,u32_t * rcvd)
{
	*sent = gprs->sentBytes;
	*rcvd = gprs->rcvdBtes;
}
#endif
#endif  /* GPRS_SUPPPORT */
