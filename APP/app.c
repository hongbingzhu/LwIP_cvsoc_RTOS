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

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  WatchDogTask              (void        *p_arg);

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
