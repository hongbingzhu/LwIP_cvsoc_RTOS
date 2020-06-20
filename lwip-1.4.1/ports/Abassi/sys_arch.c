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

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#ifndef LWIP_SPINLOCK							/* Index when using H/W spinlocks					*/
  #define LWIP_SPINLOCK		((OX_SPINLOCK_BASE)+5)
#endif

#ifndef SYS_MBX_HOLD
  #define SYS_MBX_HOLD		16					/* Maximum number of "freed" mailboxes				*/
#endif
#ifndef SYS_MTX_HOLD
  #define SYS_MTX_HOLD		16					/* Maximum number of "freed" mutexes				*/
#endif
#ifndef SYS_SEM_HOLD
  #define SYS_SEM_HOLD		16					/* Maximum number of "freed" semaphores				*/
#endif
#ifndef LWIP_MAX_TASK
  #define LWIP_MAX_TASK		16					/* Maximum # of tasks created with sys_thread_new()	*/
#endif

struct _MboxParking {							/* Structure to hold freed mailboxes & related info	*/
	sys_mbox_t Mbox;							/* Mailbox that was "freed"							*/
	int        Size;							/* Size of the freed mailbox						*/
};
struct _TaskFctArg {							/* Structure to pass the task argument in function	*/
	lwip_thread_fn Fct;							/* Function attached to the task					*/
	void          *Arg;							/* Task argument to pass to the function			*/
};

static struct _MboxParking g_MboxParking[SYS_MBX_HOLD];
static sys_mutex_t         g_MtxParking[SYS_MTX_HOLD];
static sys_sem_t           g_SemParking[SYS_SEM_HOLD];
static struct _TaskFctArg  g_TaskFctArg[LWIP_MAX_TASK];
static MTX_t              *g_LWIPmutex;
#if ((OX_N_CORE) > 1)	
  static int               g_ParkSpin;			/* To protect access to the parking lots			*/
#endif

static void TaskTrampoline(void);				/* Used to set argument as lwip_thread_fn() needs	*/

/*--------------------------------------------------------------------------------------------------*/
/*
  Creates an empty mailbox.
*/
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
int        ii;									/* Loop counter										*/
int        IsrState;							/* State of ISRs (enable/disable) before disabling	*/
sys_mbox_t MailBox;

	MailBox = (sys_mbox_t)NULL;					/* Assume there is matching "freed" mailboxes		*/

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_MBX_HOLD ; ii++) {		/* Check if the same mailbox size was freed			*/
		if ((g_MboxParking[ii].Mbox != (sys_mbox_t)NULL)
		&&  (g_MboxParking[ii].Size == size)) {	/* If yes, then take the freed one instead of new	*/
			MailBox                = g_MboxParking[ii].Mbox;
			g_MboxParking[ii].Mbox = (sys_mbox_t)NULL;
			MailBox->RdIdx         = 0;			/* Make sure the mailbox is empty					*/
			MailBox->WrtIdx        = MailBox->Size;	/* Should defintitely be but let's play safe	*/
			break;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif
	OSeint(IsrState);

	if (MailBox == (sys_mbox_t)NULL) {			/* No such mailbox was freed						*/
		MailBox = (sys_mbox_t)MBXopen(NULL, size);	/* Get a new one from Abassi					*/
	}

	*mbox = MailBox;

  #if SYS_STATS
	if (MailBox == (sys_mbox_t)NULL) {
		lwip_stats.sys.mbox.err++;
	}
	else {
		lwip_stats.sys.mbox.used++;
	}
  #endif

	return((MailBox == NULL)? (ERR_MEM) : (ERR_OK));
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
int        ii;									/* Loop counter										*/
int        IsrState;							/* State of ISRs (enable/disable) before disabling	*/
sys_mbox_t MailBox;								/* New mailbox (either MBXopen or from parking lot)	*/

  #if SYS_STATS
	lwip_stats.sys.mbox.used--;
  #endif

	MailBox = *mbox;

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_MBX_HOLD ; ii++) {		/* Find an unused entry to memo that mailbox		*/
		if (g_MboxParking[ii].Mbox == (sys_mbox_t)NULL) {
			g_MboxParking[ii].Mbox = MailBox;
			g_MboxParking[ii].Size = MailBox->Size;
			mbox                   = (sys_mbox_t *)NULL;
		  #if ((OX_N_CORE) > 1)					/* The spinlock is not needed on a single core		*/
			COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
		  #endif
			OSeint(IsrState);
			return;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif										/* Keep the ISRs disabled here						*/
												/* No more room to hold deleted mailbox				*/
	for(;;);									/* If not trapped, it would create a memory leak	*/
}

/*--------------------------------------------------------------------------------------------------*/
/*
   Posts the "msg" to the mailbox.
*/
void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
	MBXput(*mbox, (intptr_t)data, -1);
	return;
}


/*--------------------------------------------------------------------------------------------------*/
/*
   Try to post the "msg" to the mailbox.
*/
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
err_t result;

	result = ERR_OK;							/* Assume the mailbox is not full					*/
	if (0 != MBXput(*mbox, (intptr_t)msg, 0)) {
		result = ERR_MEM;						/* Mailbox is full, return info abour the error		*/
	  #if SYS_STATS
		lwip_stats.sys.mbox.err++;
	  #endif
	}

   return(result);
}

/*--------------------------------------------------------------------------------------------------*/
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
u32_t Elapse;

	Elapse = (u32_t)G_OStimCnt;					/* Current Abassi internal time						*/
	if (0 == MBXget(*mbox, (intptr_t *)msg, (timeout == 0) ? -1 : OS_MS_TO_TICK(timeout))) {
		Elapse = (((u32_t)G_OStimCnt-Elapse)*1000)
		       / OS_TICK_PER_SEC;				/* Compute elapsed time and convert in milliseconds	*/
	}
	else {
		Elapse = SYS_ARCH_TIMEOUT;				/* Did not get it, report the error					*/
		*msg   = NULL;							/* Make sure there is an invalid meesage there		*/
	}
	return(Elapse);
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Similar to sys_arch_mbox_fetch, but if message is not ready immediately, we'll
  return with SYS_MBOX_EMPTY.  On success, 0 is returned.
*/
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
u32_t RetVal;

	RetVal = ERR_OK;
	if (0 != MBXget(*mbox, (intptr_t *)msg, 0)) {
		*msg   = NULL;							/* Make sure there is an invalid meesage there		*/
		RetVal = SYS_MBOX_EMPTY;				/* Report the mailbox is empty						*/
	}
	return(RetVal);
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Creates and returns a new semaphore. The "count" argument specifies
  the initial state of the semaphore.
*/
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
int       ii;									/* Loop counter										*/
int       IsrState;								/* State of ISRs (enable/disable) before disabling	*/
sys_sem_t Sema;									/* New semaphore (either SEMopen or parking lot)	*/

	Sema = (sys_sem_t)NULL;

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_SEM_HOLD ; ii++) {		/* Check if a semaphore was freed					*/
		if (g_SemParking[ii] != (sys_sem_t)NULL) {/* If yes, then take the freed one instead of new	*/
			Sema             = g_SemParking[ii];
			g_SemParking[ii] = (sys_sem_t)NULL;
			break;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif
	OSeint(IsrState);

	if (Sema == (sys_sem_t)NULL) {				/* No semaphore was freed							*/
		Sema = (sys_sem_t)SEMopen(NULL);		/* Get a new one from Abassi						*/
	}
	Sema->Value = (int)count;					/* Set the initial count as requested				*/

	*sem = Sema;

  #if SYS_STATS
	if (Sema == (sys_sem_t)NULL) {
		lwip_stats.sys.sem.err++;
	}
	else {
		lwip_stats.sys.sem.used++;
	}
  #endif
		
	return((Sema == NULL)? (ERR_MEM) : (ERR_OK));
}

/*--------------------------------------------------------------------------------------------------*/
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
u32_t Elapse;

	Elapse = (u32_t)G_OStimCnt;					/* Current Abassi internal time						*/
	if(0 == SEMwait((SEM_t *)*sem, (timeout == 0) ? -1 : OS_MS_TO_TICK((int)timeout))) {
		Elapse = (((u32_t)G_OStimCnt-Elapse)*1000)
		       / OS_TICK_PER_SEC;				/* Compute elapsed time and convert in milliseconds	*/
	}
	else {
		Elapse = SYS_ARCH_TIMEOUT;				/* Did not get it, report the error					*/
	}
	return(Elapse);
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Signals a semaphore
*/
void sys_sem_signal(sys_sem_t *sem)
{
	SEMpost((SEM_t *)*sem);
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Deallocates a semaphore
*/
void sys_sem_free(sys_sem_t *sem)
{
int ii;											/* Loop counter										*/
int IsrState;									/* State of ISRs (enable/disable) before disabling	*/

  #if SYS_STATS
	lwip_stats.sys.sem.used--;
  #endif

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_SEM_HOLD ; ii++) {		/* Find an unused entry to memo that semaphore		*/
		if (g_SemParking[ii] == (sys_sem_t)NULL) {
			g_SemParking[ii] = *sem;
		  #if ((OX_N_CORE) > 1)					/* The spinlock is not needed on a single core		*/
			COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
		  #endif
			OSeint(IsrState);
			return;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif										/* Keep the ISRs disabled here						*/
												/* No more room to hold deleted semaphores			*/
	for(;;);									/* If not trapped, it would create a memory leak	*/
}


/*--------------------------------------------------------------------------------------------------*/
/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
int         ii;									/* Loop counter										*/
int         IsrState;							/* State of ISRs (enable/disable) before disabling	*/
sys_mutex_t Mtx;								/* New mutex (either MTXopen or from parking lot)	*/

	Mtx = (sys_sem_t)NULL;

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_MTX_HOLD ; ii++) {		/* Check if a mutex was freed						*/
		if (g_MtxParking[ii] != (sys_mutex_t)NULL) {/* If yes, then take the freed instead of new	*/
			Mtx             = g_MtxParking[ii];
			g_MtxParking[ii] = (sys_mutex_t)NULL;
			break;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif
	OSeint(IsrState);

	if (Mtx == (sys_mutex_t)NULL) {				/* No mutex was freed								*/
		Mtx = MTXopen(NULL);					/* Get a new one from Abassi						*/
	}

	*mutex = Mtx;

  #if SYS_STATS
	if (Mtx == (sys_mutex_t)NULL) {
		lwip_stats.sys.mutex.err++;
	}
	else {
		lwip_stats.sys.mutex.used++;
	}
  #endif
		
	return((Mtx == NULL)? (ERR_MEM) : (ERR_OK));
}

/*--------------------------------------------------------------------------------------------------*/
/** Lock a mutex
 * @param mutex the mutex to lock */
void sys_mutex_lock(sys_mutex_t *mutex)
{
	MTXlock((MTX_t *)*mutex, -1);
	return;
}

/*--------------------------------------------------------------------------------------------------*/
/** Unlock a mutex
 * @param mutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *mutex)
{
	MTXunlock((MTX_t *)*mutex);
	return;
}

/*--------------------------------------------------------------------------------------------------*/
/** Delete a mutex
 * @param mutex the mutex to delete */
void sys_mutex_free(sys_mutex_t *mutex) 
{
int ii;											/* Loop counter										*/
int IsrState;									/* State of ISRs (enable/disable) before disabling	*/

  #if SYS_STATS
	lwip_stats.sys.mutex.used--;
  #endif

	(*mutex)->Value = 1;						/* Make sure the mutex is at its initial state		*/

	IsrState = OSdint();						/* Make sure only 1 task plays with the Parking		*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<SYS_SEM_HOLD ; ii++) {		/* Find an unused entry to memo that semaphore		*/
		if (g_MtxParking[ii] == (sys_mutex_t)NULL) {
			g_MtxParking[ii] = *mutex;
		  #if ((OX_N_CORE) > 1)					/* The spinlock is not needed on a single core		*/
			COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
		  #endif
			OSeint(IsrState);
			return;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif										/* Keep the ISRs disabled here						*/
												/* No more room to hold deleted mutex				*/
	for(;;);									/* If not trapped, it would create a memory leak	*/
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Initialize sys arch
*/
void sys_init(void)
{
int ii;											/* Loop counter										*/
	
	for (ii=0 ; ii<SYS_MBX_HOLD ; ii++) {		/* No mailboxes freed yet							*/
		g_MboxParking[ii].Mbox = (sys_mbox_t)NULL;
	}
	for (ii=0 ; ii<SYS_MTX_HOLD ; ii++) {		/* No mutexes freed yet								*/
		g_MtxParking[ii] = (sys_mutex_t)NULL;
	}
	for (ii=0 ; ii<SYS_SEM_HOLD ; ii++) {		/* No semaphores freed yet							*/
		g_SemParking[ii] = (sys_sem_t)NULL;
	}
	for (ii=0 ; ii<LWIP_MAX_TASK ; ii++) {
		g_TaskFctArg[ii].Fct = (lwip_thread_fn)NULL;
	}
	g_LWIPmutex = MTXopen("LWIP Park");			/* Get mutex to protect the parking array accesses	*/

  #if ((OX_N_CORE) > 1)	
	g_ParkSpin = 0;
  #endif

  #if SYS_STATS
	memset(&lwip_stats, 0, sizeof(lwip_stats));
  #endif

	return;
}

/*--------------------------------------------------------------------------------------------------*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name,lwip_thread_fn thread,void *arg,int stacksize,int prio)
{
int          ii;								/* Loop counter										*/
int          IsrState;							/* State of ISRs (enable/disable) before disabling	*/
sys_thread_t Task;								/* Created task decriptor							*/

												/* Create thread in suspended mode to set argument	*/
	Task = (sys_thread_t)TSKcreate(name, prio, stacksize, &TaskTrampoline, 0);

	IsrState = OSdint();						/* Make sure only 1 task plays with g_TaskFctArg	*/
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	CORElock(LWIP_SPINLOCK, &g_ParkSpin, 0, 1+COREgetID());
  #endif										/* If the parking lot hasn't been initialized		*/
	for (ii=0 ; ii<LWIP_MAX_TASK ; ii++) {
		if (g_TaskFctArg[ii].Fct == (lwip_thread_fn)NULL) {
			g_TaskFctArg[ii].Fct = thread;
			g_TaskFctArg[ii].Arg = arg;
			break;
		}
	}
  #if ((OX_N_CORE) > 1)							/* The spinlock is not needed on a single core		*/
	COREunlock(LWIP_SPINLOCK, &g_ParkSpin, 0);
  #endif
	OSeint(IsrState);

	if (ii >= LWIP_MAX_TASK) {
		OSdint();								/* No more room to hold Fct pointer and argument	*/
		for(;;);								/* If not trapped, it would corrupt the memory		*/
	}
	TSKsetArg(Task, &g_TaskFctArg[ii]);			/* Set the task function and argument				*/
  #ifdef LWIP_ABASSI_BMP_CORE_2_USE
	TSKsetCore(Task, LWIP_ABASSI_BMP_CORE_2_USE);
  #endif
	TSKresume(Task);							/* It can now be running							*/

	return(Task);
}

/*--------------------------------------------------------------------------------------------------*/
/* Intermediate function used to pass the task argument in the function argument					*/
/*--------------------------------------------------------------------------------------------------*/

void TaskTrampoline(void)
{
struct _TaskFctArg *MyFctArg;					/* My function and argument							*/

	MyFctArg = (void *)TSKgetArg();				/* Get the Fct pointer and function argument		*/
	for(;;) {
		MyFctArg->Fct(MyFctArg->Arg);			/* Call the task function							*/

		TSKselfSusp();							/* If returning, suspend the task as documented		*/
	}
}

/*--------------------------------------------------------------------------------------------------*/
/*
  This optional function does a "fast" critical region protection and returns
  the previous protection level. This function is only called during very short
  critical regions. An embedded system which supports ISR-based drivers might
  want to implement this function by disabling interrupts. Task-based systems
  might want to implement this by using a mutex or disabling tasking. This
  function should support recursive calls from the same task or interrupt. In
  other words, sys_arch_protect() could be called while already protected. In
  that case the return value indicates that it is already protected.

  sys_arch_protect() is only required if your port is supporting an operating
  system.
*/

/* NOTE: the normal way would be to disable the interrupts and in the case of multi-core to also 	*/
/*       use a spinlock to protect the resource access across the different cores. But 				*/
/*       sys_arch_protect() and sys_arch_unprotect() are used to protect the lwIP memory management	*/
/*		 which can use malloc().																	*/
/*       Because some libraries can be protected against re-entrance by a mmutex, if the interrupts	*/
/*       are disable when the task blocks on the mutex re-entrance protection would trigger a bad	*/
/*       situation.  The blocking means a task switch will happen when the interrupts are disable.	*/
/*		 This would most likely mean the application would become frozen. That's why a mutex is		*/
/*       used instead of enabling/disabling ISRs													*/

sys_prot_t sys_arch_protect(void)
{
	MTXlock(g_LWIPmutex, -1);

	return((sys_prot_t)0);
}

/*--------------------------------------------------------------------------------------------------*/
/*
  This optional function does a "fast" set of critical region protection to the
  value specified by pval. See the documentation for sys_arch_protect() for
  more information. This function is only required if your port is supporting
  an operating system.
*/

void sys_arch_unprotect(sys_prot_t pval)
{

	pval = pval;								/* To remove compiler warning						*/

	MTXunlock(g_LWIPmutex);

	return;
}

/*--------------------------------------------------------------------------------------------------*/
/*
 * Prints an assertion messages and aborts execution.
 */
void sys_assert( const char *msg )
{
	msg = msg;
  #if 0
	printf(msg);
	printf("\n\r");
  #endif
	OSdint();
	for(;;);
}

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
#ifdef __UABASSI_H__
/*--------------------------------------------------------------------------------------------------*/
/* uAbassi does not support the mAbassi mailboxes													*/
/* This implement a crude mailbox system with the same API as Abassi / mAbassi						*/


MBX_t *MBXopen(const char *Name, int Size)
{
MBX_t *MailBox;

	MailBox = malloc(sizeof(*MailBox));
	if (MailBox == NULL) {
		return(NULL);
	}
	memset(MailBox, 0, sizeof(*MailBox));

	MailBox->SemRd   = SEMopen(NULL);
	MailBox->SemWrt  = SEMopen(NULL);
	MailBox->Mtx     = MTXopen(NULL);
	MailBox->Buffer  = malloc(Size*sizeof(MailBox->Buffer[0]));

	if ((MailBox->SemRd  == (SEM_t *)NULL)
	||  (MailBox->SemWrt == (SEM_t *)NULL)
	||  (MailBox->Mtx    == (MTX_t *)NULL)
	||  (MailBox->Buffer == (intptr_t *)NULL)) {
		free(MailBox->SemRd);
		free(MailBox->SemWrt);
		free(MailBox->Mtx);
		free(MailBox->Buffer);
		free(MailBox);
		MailBox = (MBX_t *)NULL;
	}
	MailBox->SemWrt->Value = Size;
	MailBox->Size          = Size;
	memset(MailBox->Buffer , 0, Size*sizeof(MailBox->Buffer[0]));

	return(MailBox);
}

/*--------------------------------------------------------------------------------------------------*/

int MBXget(MBX_t *Mbx, intptr_t *Msg, int Tout)
{
int ii;
int RetVal;

	RetVal = 0;
	if (0 != SEMwait(Mbx->SemRd, Tout)) {		/* Wait for a non-empty mailbox						*/
		RetVal = -1;							/* Report the mailbox is empty						*/
	}
	else {
		ii = Mbx->RdIdx - 1;					/* Check for mailbox contents						*/
		if (ii < 0) {							/* Wrap around of circular buffer index				*/
			ii = Mbx->Size-1;
		}
		*Msg        = Mbx->Buffer[ii];			/* Grab the message									*/
		Mbx->RdIdx  = ii;						/* New read index									*/

		SEMpost(Mbx->SemWrt);					/* One more spot available in the mailbox			*/
	}
	return(RetVal);
}

/*--------------------------------------------------------------------------------------------------*/

int MBXput(MBX_t *Mbx, intptr_t Msg, int Tout)
{
int ii;
int RetVal;

	RetVal == MTXlock(Mbx->Mtx, Tout);			/* Protect against multiple writers					*/
	if (RetVal != 0) {
		return(-1);
	}

	RetVal = SEMwait(Mbx->SemWrt, Tout);		/* Wait for room in the mailbox						*/
	if (RetVal != 0) {							/* When timeout on no room, release the mutex		*/
		MTXunlock(Mbx->Mtx);
		return(-1);
	}

	ii = Mbx->WrtIdx - 1;						/* Check for mailbox contents						*/
	if (ii < 0) {								/* Wrap around of circular buffer index				*/
		ii = Mbx->Size-1;
	}
	Mbx->Buffer[ii] = Msg;						/* Insert the message in the mailbox				*/
	Mbx->WrtIdx     = ii;						/* New write index									*/

	MTXunlock(Mbx->Mtx);						/* Release the protection							*/

	SEMpost(Mbx->SemRd);						/* One more message available in the mailbox		*/
												/* Done outside protection in case read blocks us	*/
	return(0);									/* If blocked, mutex would remain locked			*/
}

/*--------------------------------------------------------------------------------------------------*/
#endif
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/

/* EOF */
