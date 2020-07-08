/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2010; Micrium, Inc.; Weston, FL
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
*                                      APPLICATION CONFIGURATION
*
*                                            ZYNQ 7000 EPP
*                                               on the
*                                       ZC702 development board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : JBL
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                INCLUDE
*********************************************************************************************************
*/

#include  <stdio.h>


#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

#define OS_WATCH_DOG_TASK_PRIO		2
#define OS_WATCH_DOG_TASK_ID		2
#define WATCH_DOG_TASK_STACK_SIZE	1024u

#define OS_APP_TASK_START_PRIO		5
#define OS_APP_TASK_START_ID		5
#define APP_START_TASK_STACK_SIZE	4096u

#define LWIP_TSK_PRIO				10
#define LWIP_TSK_ID					10
#define LWIP_STK_SIZE				4096u

#define TRACE_OUT_TASK_PRIO			60
#define TRACE_OUT_TASK_ID			60
#define TRACE_OUT_STACK_SIZE		2048u

#endif /* __APP_CFG_H__ */
