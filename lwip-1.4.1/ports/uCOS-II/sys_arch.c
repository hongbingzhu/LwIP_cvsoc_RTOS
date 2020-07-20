/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#define SYS_ARCH_GLOBALS

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#if !NO_SYS
#include "ucos_ii.h"
#include "arch/sys_arch.h"
#include <stdlib.h>
#include <string.h>

#ifndef OS_NO_ERR
#define OS_NO_ERR   OS_ERR_NONE     // ucos-iii v.s. ucos-ii
#endif

/*----------------------------------------------------------------------------*/
/*                      VARIABLES                                             */
/*----------------------------------------------------------------------------*/

static OS_MEM *pQueueMem, *pStackMem;

const void *const pvNullPointer = (mem_ptr_t *)0xffffffff;
INT8U     pcQueueMemoryPool[MAX_QUEUES * sizeof(TQ_DESCR)] __attribute__((aligned(4)));
OS_STK    LwIP_Task_Stk[LWIP_TASK_MAX * LWIP_STK_SIZE] __attribute__((aligned(4)));

INT8U     LwIP_task_priopity_stask[LWIP_TASK_MAX];

/*----------------------------------------------------------------------------*/
/*                      PROTOTYPES                                            */
/*----------------------------------------------------------------------------*/


/*------------------- Creates an empty mailbox. ------------------------------*/
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	/* prarmeter "size" can be ignored in your implementation. */
	u8_t        ucErr;
	PQ_DESCR    pQDesc;
	pQDesc = OSMemGet(pQueueMem, &ucErr);

	LWIP_ASSERT("OSMemGet ", ucErr == OS_NO_ERR);
	if (ucErr == OS_NO_ERR) {
		if (size > MAX_QUEUE_ENTRIES)
			size = MAX_QUEUE_ENTRIES;

		pQDesc->pQ = OSQCreate(&(pQDesc->pvQEntries[0]), size);
		OSEventNameSet(pQDesc->pQ, "LWIP quie", &ucErr);
		LWIP_ASSERT("OSQCreate ", pQDesc->pQ != NULL);

		if (pQDesc->pQ != NULL) {
			*mbox = pQDesc;
			return 0;
		} else {
			ucErr = OSMemPut(pQueueMem, pQDesc);
			*mbox = NULL;
			return ucErr;
		}
	}
	else {
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
	u8_t        ucErr;
	sys_mbox_t  m_box = *mbox;
	LWIP_ASSERT("sys_mbox_free ", m_box != SYS_MBOX_NULL);

	OSQFlush(m_box->pQ);

	(void)OSQDel(m_box->pQ, OS_DEL_NO_PEND, &ucErr);
	LWIP_ASSERT("OSQDel ", ucErr == OS_NO_ERR);

	ucErr = OSMemPut(pQueueMem, m_box);
	LWIP_ASSERT("OSMemPut ", ucErr == OS_NO_ERR);
	*mbox = NULL;
}

/*----------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	u8_t  i = 0;
	sys_mbox_t m_box = *mbox;
	if (msg == NULL) msg = (void *)&pvNullPointer;
	/* try 10 times */
	while ((i < 10) && ((OSQPost(m_box->pQ, msg)) != OS_NO_ERR)) {
		i++;
		OSTimeDly(5);
	}
	LWIP_ASSERT("sys_mbox_post error!\n", i != 10);
}

/* Try to post the "msg" to the mailbox. */
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	sys_mbox_t m_box = *mbox;
	if (msg == NULL) msg = (void *)&pvNullPointer;

	if ((OSQPost(m_box->pQ, msg)) != OS_NO_ERR) {
		return ERR_MEM;
	}
	return ERR_OK;
}

/*----------------------------------------------------------------------------*/
/*
  Blocks the thread until a message arrives in the mailbox, but does
  not block the thread longer than "timeout" milliseconds (similar to
  the sys_arch_sem_wait() function). The "msg" argument is a result
  parameter that is set by the function (i.e., by doing "*msg =
  ptr"). The "msg" parameter maybe NULL to indicate that the message
  should be dropped.

  The return values are the same as for the sys_arch_sem_wait() function:
  Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
  timeout.

  Note that a function with a similar name, sys_mbox_fetch(), is
  implemented by lwIP.
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	u8_t    ucErr;
	u32_t   ucos_timeout, timeout_new;
	void    *temp;
	sys_mbox_t m_box = *mbox;
	/* convert to OS time tick */
	if (timeout != 0) {
		ucos_timeout = (timeout * OS_TICKS_PER_SEC) / 1000;
		if (ucos_timeout < 1)
			ucos_timeout = 1;
	} else ucos_timeout = 0;

	timeout = OSTimeGet();

	temp = OSQPend(m_box->pQ, ucos_timeout, &ucErr);

	if (msg != NULL) {
		if (temp == (void *)&pvNullPointer)
			*msg = NULL;
		else
			*msg = temp;
	}

	if (ucErr == OS_ERR_TIMEOUT)
		timeout = SYS_ARCH_TIMEOUT;
	else {
		LWIP_ASSERT("OSQPend ", ucErr == OS_NO_ERR);

		timeout_new = OSTimeGet() - timeout;	/* do NOT worry about u32 overflow */
		/* convert to millisecond */
		timeout = timeout_new * 1000 / OS_TICKS_PER_SEC + 1;
	}
	return timeout;
}

/*
 * Check if an mbox is valid/allocated:
 * @param sys_mbox_t *mbox pointer mail box
 * @return 1 for valid, 0 for invalid
 */
int sys_mbox_valid(sys_mbox_t *mbox)
{
	sys_mbox_t  m_box = *mbox;
	u8_t        ucErr;
	int         ret;
	OS_Q_DATA   q_data;
	memset(&q_data, 0, sizeof(OS_Q_DATA));
	ucErr = OSQQuery(m_box->pQ, &q_data);
	ret = (ucErr < 2 && (q_data.OSNMsgs < q_data.OSQSize)) ? 1 : 0;
	return ret;
}

/* Set an mbox invalid so that sys_mbox_valid returns 0 */
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{

}

/*
 * Creates and returns a new semaphore. The "count" argument specifies
 * the initial state of the semaphore. TBD finish and test
 */
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	u8_t err;

	*sem = OSSemCreate((u16_t)count);
	if (*sem == NULL) {
		return -1;
	}

	OSEventNameSet(*sem, "LWIP Sem", &err);
	LWIP_ASSERT("OSSemCreate ", *sem != NULL);
	return 0;
}

/*
  Blocks the thread while waiting for the semaphore to be
  signaled. If the "timeout" argument is non-zero, the thread should
  only be blocked for the specified time (measured in
  milliseconds).

  If the timeout argument is non-zero, the return value is the number of
  milliseconds spent waiting for the semaphore to be signaled. If the
  semaphore wasn't signaled within the specified time, the return value is
  SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
  (i.e., it was already signaled), the function may return zero.

  Notice that lwIP implements a function with a similar name,
  sys_sem_wait(), that uses the sys_arch_sem_wait() function.
*/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	u8_t ucErr;
	u32_t ucos_timeout, timeout_new;

	if (timeout != 0) {
		ucos_timeout = (timeout * OS_TICKS_PER_SEC) / 1000; // convert to OS time tick
		if (ucos_timeout < 1)
			ucos_timeout = 1;
	}
	else ucos_timeout = 0;

	timeout = OSTimeGet();

	OSSemPend(*sem, ucos_timeout, &ucErr);
	/*  only when timeout! */
	if (ucErr == OS_ERR_TIMEOUT)
		timeout = SYS_ARCH_TIMEOUT;
	else {
		/* for pbuf_free, may be called from an ISR */
		timeout_new = OSTimeGet() - timeout;	/* do NOT worry about u32 overflow */
		/* convert to milisecond */
		timeout = timeout_new * 1000 / OS_TICKS_PER_SEC + 1;
	}
	return timeout;
}

/* Signals a semaphore */
void sys_sem_signal(sys_sem_t *sem)
{
	OSSemPost(*sem);
}

/* Deallocates a semaphore */
void sys_sem_free(sys_sem_t *sem)
{
	u8_t     ucErr;
	OSSemDel(*sem, OS_DEL_ALWAYS, &ucErr);
	LWIP_ASSERT("OSSemDel ", ucErr == OS_NO_ERR);
	*sem = NULL;
}

int sys_sem_valid(sys_sem_t *sem)
{
	OS_SEM_DATA  sem_data;
	return (OSSemQuery(*sem, &sem_data) == OS_NO_ERR) ? 1 : 0;
}

/* Set a semaphore invalid so that sys_sem_valid returns 0 */
void sys_sem_set_invalid(sys_sem_t *sem)
{
}

/*----------------------------------------------------------------------------*/
/*            memory interface                                                */
/*----------------------------------------------------------------------------*/
#if 0
typedef struct mem_leaks {
	void *location;
	INT32U size;
	struct mem_leaks *next;
} MEM_LEAKS;

struct mem_leaks *MemoryLeaks = NULL;
int       MaxUserMemory  = 0;
int       CurrUserMemory = 0;

/**
 * Allocate memory: determine the smallest pool that is big enough
 * to contain an element of 'size' and get an element from that pool.
 *
 * @param size the size in bytes of the memory needed
 * @return a pointer to the allocated memory or NULL if the pool is empty
 */
__weak void *mem_malloc(mem_size_t length)
{
	if (!length) {
		LWIP_ASSERT("mem_malloc: Illegal Length\n", length);
		return NULL;
	}
	OS_CPU_SR  cpu_sr = 0;
	OS_ENTER_CRITICAL();
	INT8U *p = NULL;
	/* Create leaks entry */
	struct mem_leaks *mem_leak = malloc(sizeof(MEM_LEAKS));
	LWIP_ASSERT("mem_malloc: no free memory for allocation leaks", mem_leak);
	if (!mem_leak) {
		OS_EXIT_CRITICAL();
		return NULL;
	}
	mem_leak->next = (struct mem_leaks *)NULL;
	/* allocation block memory */
	p = malloc(length);
	LWIP_ASSERT("mem_malloc: no free memory for allocation data", p);
	if (!p) {
		OS_EXIT_CRITICAL();
		return NULL;
	}
	memset(p, 0, length);
	mem_leak->location = (struct mem_leaks *)p;
	mem_leak->size = length;
	CurrUserMemory += length;
	if (CurrUserMemory > MaxUserMemory)
		MaxUserMemory = CurrUserMemory;
	/* Create next leaks entry */
	struct mem_leaks *EndMemoryLeaks = NULL;
	if (MemoryLeaks == NULL)
		MemoryLeaks = mem_leak;
	else {
		for (EndMemoryLeaks = MemoryLeaks; EndMemoryLeaks->next; EndMemoryLeaks = EndMemoryLeaks->next);
		EndMemoryLeaks->next = mem_leak;
	}
	OS_EXIT_CRITICAL();
	return (void *)p;
}

/**
 * Free memory previously allocated by mem_malloc. Loads the pool number
 * and calls memp_free with that pool number to put the element back into
 * its pool
 *
 * @param rmem the memory element to free
 */
__weak void mem_free(void *rmem)
{
	if (!rmem || MemoryLeaks == NULL) return;
	OS_CPU_SR  cpu_sr = 0;
	OS_ENTER_CRITICAL();
	struct mem_leaks *EndMemoryLeaks = NULL, *PrevMemoryLeaks = NULL;
	for (EndMemoryLeaks = MemoryLeaks; EndMemoryLeaks; EndMemoryLeaks = EndMemoryLeaks->next) {
		if (EndMemoryLeaks->location == rmem) {
			CurrUserMemory -= EndMemoryLeaks->size;
			if (CurrUserMemory < 0)
				CurrUserMemory = 0;
			free(rmem);
			if (PrevMemoryLeaks)
				PrevMemoryLeaks->next = EndMemoryLeaks->next;
			else
				MemoryLeaks = EndMemoryLeaks->next;
			free(EndMemoryLeaks);
			break;
		}
		PrevMemoryLeaks = EndMemoryLeaks;
	}
	OS_EXIT_CRITICAL();
}

/**
 * Zero the heap and initialize start, end and lowest-free
 */
__weak void mem_init(void)
{

}

/**
 * Shrink memory returned by mem_malloc().
 *
 * @param rmem pointer to memory allocated by mem_malloc the is to be shrinked
 * @param newsize required size after shrinking (needs to be smaller than or
 *                equal to the previous size)
 * @return for compatibility reasons: is always == rmem, at the moment
 *         or NULL if newsize is > old size, in which case rmem is NOT touched
 *         or freed!
 */
__weak void *mem_trim(void *rmem, mem_size_t newsize)
{
	if (!newsize) {
		LWIP_ASSERT("mem_trim: Illegal Length\n", newsize);
		return NULL;
	}
	OS_CPU_SR  cpu_sr = 0;
	OS_ENTER_CRITICAL();
	INT8U *p = rmem;
	/* search memory in list leaks */
	struct mem_leaks *CurMemoryLeaks = NULL;
	for (CurMemoryLeaks = MemoryLeaks; CurMemoryLeaks; CurMemoryLeaks = CurMemoryLeaks->next) {
		if (CurMemoryLeaks->location == rmem) {
			p = realloc(rmem, newsize);
			LWIP_ASSERT("mem_trim: no free memory for allocation data", p);
			if (!p) {
				OS_EXIT_CRITICAL();
				return NULL;
			}
			CurMemoryLeaks->location = p;
			if (CurMemoryLeaks->size > newsize)
				CurrUserMemory -= (CurMemoryLeaks->size - newsize);
			else {
				INT32U delta = (newsize - CurMemoryLeaks->size);
				CurrUserMemory += delta;
				memset(&p[CurMemoryLeaks->size], 0, delta);
			}
			CurMemoryLeaks->size = newsize;
			break;
		}
	}
	OS_EXIT_CRITICAL();
	return (void *)p;
}
#endif

/* Initialize sys arch */
void sys_init(void)
{
	u8_t ucErr;
	memset(LwIP_task_priopity_stask, 0, sizeof(LwIP_task_priopity_stask));
	/* init mem used by sys_mbox_t, use ucosII functions */
	pQueueMem = OSMemCreate((void *)pcQueueMemoryPool, MAX_QUEUES, sizeof(TQ_DESCR), &ucErr);
	OSMemNameSet(pQueueMem, "LwIP Queue", &ucErr);
	LWIP_ASSERT("sys_init: failed OSMemCreate Queue", ucErr == OS_NO_ERR);
	pStackMem = OSMemCreate((void *)LwIP_Task_Stk, LWIP_TASK_MAX, LWIP_STK_SIZE * sizeof(OS_STK), &ucErr);
	OSMemNameSet(pQueueMem, "LwIP Stack", &ucErr);
	LWIP_ASSERT("sys_init: failed OSMemCreate Stack", ucErr == OS_NO_ERR);
}

/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
// TBD
/*-----------------------------------------------------------------------------------*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
	u8_t ubPrio = LWIP_TASK_START_PRIO;
	u8_t ucErr;
	arg = arg;
	int i;
	if (prio) {
		ubPrio += (prio - 1);
		for (i = 0; i < LWIP_TASK_MAX; ++i)
			if (LwIP_task_priopity_stask[i] == ubPrio)
				break;
		if (i == LWIP_TASK_MAX) {
			for (i = 0; i < LWIP_TASK_MAX; ++i)
				if (LwIP_task_priopity_stask[i] == 0) {
					LwIP_task_priopity_stask[i] = ubPrio;
					break;
				}
			if (i == LWIP_TASK_MAX) {
				LWIP_ASSERT("sys_thread_new: there is no space for priority", 0);
				return (-1);
			}
		} else
			prio = 0;
	}
	/* Search for a suitable priority */
	if (!prio) {
		ubPrio = LWIP_TASK_START_PRIO;
		while (ubPrio < (LWIP_TASK_START_PRIO + LWIP_TASK_MAX)) {
			for (i = 0; i < LWIP_TASK_MAX; ++i)
				if (LwIP_task_priopity_stask[i] == ubPrio) {
					++ubPrio;
					break;
				}
			if (i == LWIP_TASK_MAX)
				break;
		}
		if (ubPrio < (LWIP_TASK_START_PRIO + LWIP_TASK_MAX))
			for (i = 0; i < LWIP_TASK_MAX; ++i)
				if (LwIP_task_priopity_stask[i] == 0) {
					LwIP_task_priopity_stask[i] = ubPrio;
					break;
				}
		if (ubPrio >= (LWIP_TASK_START_PRIO + LWIP_TASK_MAX) || i == LWIP_TASK_MAX) {
			LWIP_ASSERT("sys_thread_new: there is no free priority", 0);
			return (-1);
		}
	}
	if (stacksize > LWIP_STK_SIZE || !stacksize)
		stacksize = LWIP_STK_SIZE;
	/* get Stack from pool */
	OS_STK *task_stk = OSMemGet(pStackMem, &ucErr);
	if (ucErr != OS_NO_ERR) {
		LWIP_ASSERT("sys_thread_new: impossible to get a stack", 0);
		return (-1);
	}
	INT16U task_id = ubPrio - LWIP_TASK_START_PRIO + LWIP_TSK_ID;
#if (OS_TASK_STAT_EN == 0)
	OSTaskCreate(thread, (void *)arg, &task_stk[stacksize - 1], ubPrio);
#else
	OSTaskCreateExt(thread, (void *)arg, &task_stk[stacksize - 1], ubPrio, task_id,
					&task_stk[0], stacksize, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
#endif
	OSTaskNameSet(ubPrio, (u8_t *)name, &ucErr);

	return task_id;
}

/**
 * Sleep for some ms. Timeouts are NOT processed while sleeping.
 *
 * @param ms number of milliseconds to sleep
 */
void sys_msleep(u32_t ms)
{
	OSTimeDly(ms);
}

/*--------------------------------------------------------------------------------------------------*/
#endif // !NO_SYS
/* EOF */
