/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                          APPLICATION CODE
*
*                                            CYCLONE V SOC
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JBL
*********************************************************************************************************
* Note(s)       : none.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <app_cfg.h>
#include  <lib_mem.h>

#include  <bsp.h>
#include  <bsp_int.h>
#include  <bsp_os.h>
#include  <cpu_cache.h>

#include  <cpu.h>
#include  <cpu_core.h>

#include  <os.h>


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

CPU_STK AppTaskStartStk[APP_START_TASK_STACK_SIZE];
CPU_STK WatchDogTaskStk[WATCH_DOG_TASK_STACK_SIZE];
CPU_STK TraceOutTaskStk[TRACE_OUT_STACK_SIZE];

OS_EVENT * m_traceNotify;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  WatchDogTask              (void        *p_arg);
static  void  TraceOutTask              (void        *p_arg);

/*
*********************************************************************************************************
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : (1) It is assumed that your code will call main() once you have performed all necessary
*                   initialisation.
*********************************************************************************************************
*/

int main ()
{
    INT8U os_err;

    BSP_WatchDog_Reset();                                       /* Reset the watchdog as soon as possible.              */

                                                                /* Scatter loading is complete. Now the caches can be activated.*/
    BSP_BranchPredictorEn();                                    /* Enable branch prediction.                            */
    BSP_L2C310Config();                                         /* Configure the L2 cache controller.                   */
//    BSP_CachesEn();                                             /* Enable L1 I&D caches + L2 unified cache.             */

    void GICinit(void);
    GICinit();

    CPU_Init();

    Mem_Init();

    BSP_Init();


    OSInit();


    os_err = OSTaskCreateExt((void (*)(void *)) AppTaskStart,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&AppTaskStartStk[APP_START_TASK_STACK_SIZE - 1],
                             (INT8U           ) OS_APP_TASK_START_PRIO,
                             (INT16U          ) OS_APP_TASK_START_ID,
                             (OS_STK        * )&AppTaskStartStk[0],
                             (INT32U          ) APP_START_TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
    OSTaskNameSet(OS_APP_TASK_START_PRIO, (INT8U *)(void *)"AppStart", &os_err);

    os_err = OSTaskCreateExt((void (*)(void *)) WatchDogTask,   /* Create the watchdog task.                            */
                             (void          * ) 0,
                             (OS_STK        * )&WatchDogTaskStk[WATCH_DOG_TASK_STACK_SIZE - 1],
                             (INT8U           ) OS_WATCH_DOG_TASK_PRIO,
                             (INT16U          ) OS_WATCH_DOG_TASK_ID,
                             (OS_STK        * )&WatchDogTaskStk[0],
                             (INT32U          ) WATCH_DOG_TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
    OSTaskNameSet(OS_WATCH_DOG_TASK_PRIO, (INT8U *)(void *)"WatchDog", &os_err);

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    CPU_IntEn();

    OSStart();
}

/*
*********************************************************************************************************
*                                           App_TaskStart()
*
* Description : Startup task example code.
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
* Notes       : (1) The ticker MUST be initialised AFTER multitasking has started.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    BSP_OS_TmrTickInit(OS_TICKS_PER_SEC);                       /* Configure and enable OS tick interrupt.              */

	int lwip_app_main(void);
	lwip_app_main();

	for(;;) {
        OSTimeDlyHMSM(0, 0, 0, 500);
        //BSP_LED_On(0);
        //OSTimeDlyHMSM(0, 0, 0, 500);
        //BSP_LED_Off(0);
    }
}

/*
*********************************************************************************************************
*                                            WatchDogTask()
*
* Description : Periodically reset the platform watchdog.
*
* Arguments   : none.
*
* Returns     : none.
*********************************************************************************************************
*/


void  WatchDogTask (void *p_arg)
{
    while (DEF_TRUE) {
        BSP_WatchDog_Reset();                                   /* Reset the watchdog.                                  */

        OSTimeDlyHMSM(0, 0, 0, 500);
        BSP_LED_Flash(0);
    }
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
void OSStartTrace(void)
{
	INT8U os_err;
    m_traceNotify = OSMboxCreate(NULL);
    OSEventNameSet(m_traceNotify, "TraceNotify", &os_err);

    os_err = OSTaskCreateExt((void (*)(void *)) TraceOutTask,   /* Create the trace out task.                            */
                             (void          * ) 0,
                             (OS_STK        * )&TraceOutTaskStk[TRACE_OUT_STACK_SIZE - 1],
                             (INT8U           ) TRACE_OUT_TASK_PRIO,
                             (INT16U          ) TRACE_OUT_TASK_ID,
                             (OS_STK        * )&TraceOutTaskStk[0],
                             (INT32U          ) TRACE_OUT_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
    OSTaskNameSet(TRACE_OUT_TASK_PRIO, (INT8U *)(void *)"TraceOut", &os_err);
}

#include "alt_16550_uart.h"
#include <stdarg.h>
#include <string.h>	// memcpy
ALT_16550_HANDLE_t G_UARThndl;

typedef struct TraceBuff_t
{
	char buff[4096];
	int  p_in, p_out;	// data in, data out
	uint32_t cnt;	// count in buffer
} TraceBuff;
TraceBuff m_tbuff;

int TracePuts(const char * msg, int len)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL();
	if (len > sizeof(m_tbuff.buff) - m_tbuff.cnt)
		len = sizeof(m_tbuff.buff) - m_tbuff.cnt;
	const char * p = msg;
	int ret_len = len;
	if (len + m_tbuff.p_in >= sizeof(m_tbuff.buff))
	{
		int this_len = sizeof(m_tbuff.buff) - m_tbuff.p_in;
		memcpy(m_tbuff.buff+m_tbuff.p_in, p, this_len);
		len -= this_len;
		p += this_len;
		m_tbuff.p_in = 0;
		m_tbuff.cnt += this_len;
	}
	if (len)
	{
		memcpy(m_tbuff.buff+m_tbuff.p_in, p, len);
		m_tbuff.p_in += len;
		m_tbuff.cnt += len;
	}
	OS_EXIT_CRITICAL();
	OSMboxPost(m_traceNotify, (void *)1);

	return ret_len;
}

int TracePrintf(const char * fmt, ...)
{
	char buff[128];
	int len = snprintf(buff, sizeof(buff), "%8d ", OSTimeGet());

	va_list args;
	va_start(args, fmt);
	len += vsnprintf(buff+len, sizeof(buff)-len, fmt, args);
	va_end(args);
	if (len > sizeof(buff)) while (1) ;

	return TracePuts(buff, len);
}

#define UART_FIFO_SIZE		128
void TraceOutTask(void *p_arg)
{
	INT8U os_err;
	uint32_t fifo_size;
	alt_16550_fifo_size_get_tx(&G_UARThndl, &fifo_size);
	if (fifo_size > UART_FIFO_SIZE) while (1) ;

	while (1)
	{
		OSMboxPend(m_traceNotify, 0, &os_err);
		while (m_tbuff.cnt > 0)
		{
			uint32_t uu;
			while (1) {
				alt_16550_fifo_level_get_tx(&G_UARThndl, &uu);
				if (uu >= 1) OSTimeDly(uu);
				else break;
			};
			uint32_t cnt = m_tbuff.cnt;
			uint32_t n = (cnt > fifo_size) ? fifo_size : cnt;
			if (n + m_tbuff.p_out > sizeof(m_tbuff.buff))
				n = sizeof(m_tbuff.buff) - m_tbuff.p_out;
			uint32_t fifo_n = n;
			char buff[UART_FIFO_SIZE];
			const char * p = m_tbuff.buff+m_tbuff.p_out;
			for (int i = 0; i < fifo_n; i++)
			{
				char ch = *p++;
				if (ch == '\n')	// we should insert '\r'
				{
					if ((fifo_n == fifo_size) && (i == fifo_n - 1))
					{
						// if full FIFO write and last char, we skip it this round
						n--; fifo_n--; break;
					}
					else
					{
						buff[i++] = '\r';
						if (fifo_n == fifo_size)
							n--;		// full FIFO write, decrease a written char
						else
							fifo_n++;	// not full FIFO write, increase write length
					}
				}
				buff[i] = ch;
			}
			alt_16550_fifo_write(&G_UARThndl, buff, fifo_n);

			OS_CPU_SR cpu_sr;
			OS_ENTER_CRITICAL();
			m_tbuff.cnt -= n;
			m_tbuff.p_out += n;
			if (m_tbuff.p_out >= sizeof(m_tbuff.buff))
				m_tbuff.p_out -= sizeof(m_tbuff.buff);
			OS_EXIT_CRITICAL();
		}
	}
}
