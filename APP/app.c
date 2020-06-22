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

#define OS_APP_TASK_START_PRIO		5
#define	APP_START_TASK_STACK_SIZE	4096u
CPU_STK AppTaskStartStk[APP_START_TASK_STACK_SIZE];

#define OS_WATCH_DOG_TASK_PRIO		2
#define WATCH_DOG_TASK_STACK_SIZE	1024u
CPU_STK WatchDogTaskStk[WATCH_DOG_TASK_STACK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  WatchDogTask              (void        *p_arg);

CPU_BOOLEAN  AppInit_TCPIP (void);


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
                             (INT16U          ) OS_APP_TASK_START_PRIO,
                             (OS_STK        * )&AppTaskStartStk[0],
                             (INT32U          ) APP_START_TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));
    OSTaskNameSet(OS_APP_TASK_START_PRIO, (INT8U *)(void *)"AppStart", &os_err);

    os_err = OSTaskCreateExt((void (*)(void *)) WatchDogTask,   /* Create the watchdog task.                            */
                             (void          * ) 0,
                             (OS_STK        * )&WatchDogTaskStk[WATCH_DOG_TASK_STACK_SIZE - 1],
                             (INT8U           ) OS_WATCH_DOG_TASK_PRIO,
                             (INT16U          ) OS_WATCH_DOG_TASK_PRIO,
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

    //AppInit_TCPIP();                                            /* Initialize uC/TCPIP                                  */

    for(;;) {
    	int lwip_app_main(void);
    	lwip_app_main();
        //OSTimeDlyHMSM(0, 0, 0, 500);
        //BSP_LED_On(0);
        //OSTimeDlyHMSM(0, 0, 0, 500);
        //BSP_LED_Off(0);
    }

}

#if 0
/*
*********************************************************************************************************
*                                           AppInit_TCPIP()
*
* Description : TCP-IP startup code.
*
* Arguments   : none.
*
* Returns     : none.
*********************************************************************************************************
*/

CPU_BOOLEAN  AppInit_TCPIP (void)
{
    NET_IF_NBR      if_nbr;
    NET_ERR         err_net;
#ifdef NET_IPv4_MODULE_EN
    NET_IPv4_ADDR   addr_ipv4;
    NET_IPv4_ADDR   msk_ipv4;
    NET_IPv4_ADDR   gateway_ipv4;
#endif
#ifdef NET_IPv6_MODULE_EN
    NET_IPv6_ADDR   ipv6_addr;
#endif


                                                                /* -------- INITIALIZE NETWORK TASKS & OBJECTS -------- */
    err_net = Net_Init(&NetRxTaskCfg,                           /* See Note #6.                                         */
                       &NetTxDeallocTaskCfg,
                       &NetTmrTaskCfg);
    if (err_net != NET_ERR_NONE) {
        return (DEF_FAIL);
    }

    /* -------------- ADD ETHERNET INTERFACE -------------- */
    /* See Note #7.                                         */
    if_nbr = NetIF_Add((void    *)&NetIF_API_Ether,  /* See Note #7b.                                        */
            (void    *)&NetDev_API_HPS_EMAC,         /* Device driver API,    See Note #7c.                  */
            (void    *)&NetDev_BSP_BoardDev_Nbr,     /* BSP API,              See Note #7d.                  */
            (void    *)&NetDev_Cfg_Ether_1,          /* Device configuration, See Note #7e.                  */
            (void    *)&NetPhy_API_ksz9021r,         /* PHY driver API,       See Note #7f.                  */
            (void    *)&NetPhy_Cfg_Ether_1,          /* PHY configuration,    See Note #7g.                  */
            &err_net);
    if (err_net != NET_IF_ERR_NONE) {
        return (DEF_FAIL);
    }


                                                                /* ------------- START ETHERNET INTERFACE ------------- */
    NetIF_Start(if_nbr, &err_net);                              /* See Note #8.                                         */
    if (err_net != NET_IF_ERR_NONE) {
        return (DEF_FAIL);
    }

#ifdef NET_IPv4_MODULE_EN
                                                                /* --------- CONFIGURE IPV4 STATIC ADDRESSES ---------- */
                                                                /* See Note #9                                          */
    NetASCII_Str_to_IP("10.10.110.2",                           /* Convert Host IPv4 string address to 32 bits address.*/
                       &addr_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetASCII_Str_to_IP("255.255.255.0",                         /* Convert IPv4 mask string to 32 bits address.         */
                       &msk_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetASCII_Str_to_IP("10.10.110.1",                            /* Convert Gateway string address to 32 bits address.   */
                       &gateway_ipv4,
                       NET_IPv4_ADDR_SIZE,
                      &err_net);
    NetIPv4_CfgAddrAdd(if_nbr,                                  /* Add a statically-configured IPv4 host address,   ... */
                       addr_ipv4,                               /* ... subnet mask, & default gateway to the        ... */
                       msk_ipv4,                                /* ... interface. See Note #10.                         */
                       gateway_ipv4,
                      &err_net);
    if (err_net != NET_IPv4_ERR_NONE) {
        return (DEF_FAIL);
    }
#endif

#ifdef NET_IPv6_MODULE_EN
                                                                /* ----- CONFIGURE IPV6 STATIC LINK LOCAL ADDRESS ----- */
                                                                /* See Note #9.                                         */
    NetASCII_Str_to_IP("fe80::1111:1112",                       /* Convert IPv6 string address to 128 bits address.     */
                       &ipv6_addr,
                        NET_IPv6_ADDR_SIZE,
                       &err_net);
    NetIPv6_CfgAddrAdd(if_nbr,                                  /* Add a statically-configured IPv6 host address to ... */
                      &ipv6_addr,                               /* ... the interface. See Note 11.                      */
                       64,
                      &err_net);
    if (err_net != NET_IPv6_ERR_NONE) {
        return (DEF_FAIL);
    }
#endif

    return (DEF_OK);
}
#endif

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
