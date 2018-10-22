/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== task.c ========
 *
 * Task Management functions
 *
 */

#include <netmain.h>
#include <_stack.h>
#include <_oskern.h>

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>

/*
 * Low Resource Flag
 * We can only process a low resource condition when
 * we are not in kernel mode, so we use a flag and
 * then process it on a call to llExit()
 */
static uint32_t _TaskFlagLowResource = 0;

/* signal that system resources are low */
void NotifyLowResource(void)
{
    _TaskFlagLowResource = 1;
}

/*
 * Hook ID
 * The Hook ID is used to set our environment pointer for
 * slot zero. It must be valid!! (Check in SetEnv function)
 */
static uint32_t    hookOK = 0;

/* key used to store file descriptor environment for all threads */
static pthread_key_t key;

extern void ti_ndk_config_Global_stackThread(uintptr_t arg0, uintptr_t arg1);

/*-------------------------------------------------------------------- */
/* llEnter / llExit - Kernel Mode gate functions */
/*-------------------------------------------------------------------- */
/* NOTE: There are two versions of the llEnter()/llExit() pairing. */
/* The default version uses task priority to implement exclusion. The */
/* second version uses a semaphore. The priority based version is */
/* faster, but not as safe if care is not taken when setting task */
/* priorities. */

/* NOTE: Whether or not to use a semaphore for exclusion is independent */
/*       of the choice to use a semaphore for scheduling in NETCTRL. */
/*       The concepts are unrelated. */

/* To enable the semaphore version, set the below #define to 1 */
/* Note that when _NDK_EXTERN_CONFIG is defined, this USE_LL_SEMAPHORE */
/* is defined outside the module. */
/*-------------------------------------------------------------------- */
#ifndef _NDK_EXTERN_CONFIG
#define USE_LL_SEMAPHORE 0
#endif

static void  *hSemGate = 0;
static volatile int InKernel = 0;

#if USE_LL_SEMAPHORE
/*-------------------------------------------------------------------- */
/* llEnter() */
/* Enter the IP stack */
/*-------------------------------------------------------------------- */
void llEnter()
{
    /* If we don't have our semaphore yet, allocate one with "1" entry */
    if( !hSemGate )
    {
        if( !(hSemGate = SemCreate(1)) )
        {
            DbgPrintf(DBG_ERROR,"llEnter (sem): Could not create llEnter() semaphore");
            return;
        }
    }

    /* Wait on entering the stack. */
    SemPend(hSemGate, SEM_FOREVER);

    /* If case something goes wrong with the semaphore, track */
    /* problems entering the stack */
    if( InKernel )
        DbgPrintf(DBG_ERROR,"llEnter (sem): Illegal call to llEnter()");

    InKernel=1;
}
/*-------------------------------------------------------------------- */
/* llExit() */
/* Release the IP stack */
/*-------------------------------------------------------------------- */
void llExit()
{
    /* Handle the low resource condition */
    if( _TaskFlagLowResource )
    {
        ExecLowResource();
        _TaskFlagLowResource = 0;
    }

    /* The "InKernel" flag traps calls to llExit() before calling */
    /* llEnter(). */
    if( !InKernel )
    {
        DbgPrintf(DBG_ERROR,"llExit (sem): Illegal call to llExit()");
        return;
    }
    InKernel=0;

    /* Signal that we've exited the stack. */
    SemPost(hSemGate);
}
#else
static int OldPriority = 0;
/*-------------------------------------------------------------------- */
/* llEnter() */
/* Enter the IP stack */
/*-------------------------------------------------------------------- */
void llEnter()
{
    int tmpOldPriority;

    /* If we don't have our semaphore yet, allocate one with "0" entry */
    if( !hSemGate )
    {
        if( !(hSemGate = SemCreate(0)) )
        {
            DbgPrintf(DBG_ERROR,
                    "llEnter: Could not create llEnter() semaphore");
            return;
        }
    }

    /* Since this task is running (assume for now it has a priority LESS */
    /* THAN the kernel level - we know that nothing is running at the */
    /* kernel level. */

    /* We try to use priority to block other calls from coming into */
    /* llEnter(). However, if we are re-entered, we'll fall back to */
    /* using the semaphore */

    /* Set this task's priority at kernel level */
    tmpOldPriority = TaskSetPri( TaskSelf(), OS_TASKPRIKERN );

    /* Verify this call was legal */
    if( tmpOldPriority >= OS_TASKPRIKERN )
    {
        if( InKernel )
        {
            DbgPrintf(DBG_ERROR,
                    "llEnter: Illegal reentrant call to llEnter()");
        }
        else
        {
            DbgPrintf(DBG_ERROR,"llEnter: Illegal priority call to llEnter()");
        }
        return;
    }

    /* Verify there has been no "reentrance". */
    /* If there was, use the semaphore. */
    if( ++InKernel > 1 )
    {
        /* Wait on entering the stack. */
        SemPend(hSemGate, SEM_FOREVER);
    }

    /* Store the priority of the task that owns kernel mode */
    OldPriority = tmpOldPriority;
}
/*-------------------------------------------------------------------- */
/* llExit() */
/* Release the IP stack */
/*-------------------------------------------------------------------- */
void llExit()
{
    /* Handle the low resource condition */
    if( _TaskFlagLowResource )
    {
        ExecLowResource();
        _TaskFlagLowResource = 0;
    }

    /* Verify we were at kernel level */
    if( !InKernel )
    {
        DbgPrintf(DBG_ERROR,"llExit: Illegal call to llExit()");
        return;
    }

    /* If a task is waiting at llEnter(), signal it */
    if( --InKernel > 0 )
    {
        /* Signal that we've exited the stack. */
        SemPost(hSemGate);
    }

    /* Restore this task's priority */
    TaskSetPri( TaskSelf(), OldPriority );
}
#endif

/*-------------------------------------------------------------------- */
/* TaskCleanupHook() - Called automatically at Task exit (user shouldn't */
/*                     call) */
/*-------------------------------------------------------------------- */
void TaskCleanupHook(void *arg)
{
    /* Delete mem allocated for ti_ndk_os_TaskArgs struct */
    if (arg) {
        free(arg);
    }
}

/*-------------------------------------------------------------------- */
/* TaskInit() - Called from NETCTRL init */
/*-------------------------------------------------------------------- */
void _TaskInit()
{
    /* all NDK threads will share the same key */
    pthread_key_create(&key, NULL);

    hookOK = 1;
}

/*-------------------------------------------------------------------- */
/* TaskShutdown() - Called from NETCTRL shutdown */
/*-------------------------------------------------------------------- */
void _TaskShutdown()
{
}

/*-------------------------------------------------------------------- */
/* TaskBlock() */
/* Block a task */
/*-------------------------------------------------------------------- */
void TaskBlock(void *h)
{
}

/*
 * ======== ti_ndk_os_threadStub ========
 * Wrapper function used by TaskCreate(). This function call user's thread
 * function and calls fdOpenSession() and fdCloseSession() for the user.
 */
static void *ti_ndk_os_threadStub(void *threadArgs)
{
    ti_ndk_os_TaskArgs *args = (ti_ndk_os_TaskArgs *)threadArgs;

    /* push clean up handler to free allocated memory once user thread exits */
    pthread_cleanup_push(&TaskCleanupHook, threadArgs);

    fdOpenSession((void *)pthread_self());

    /* call user thread function */
    args->arg0(args->arg1, args->arg2);

    fdCloseSession((void *)pthread_self());

    /* User thread has exited, pop (run) the clean up handler function */
    pthread_cleanup_pop(1);

    return (NULL);
}

/*-------------------------------------------------------------------- */
/* TaskCreate() */
/* Create a task */
/*-------------------------------------------------------------------- */
/* ARGSUSED */
void *TaskCreate(void (*pFun)(), char *Name, int Priority, uint32_t StackSize,
                   uintptr_t Arg1, uintptr_t Arg2, uintptr_t argReserved)
{
    int status;
    pthread_attr_t pthreadAttrs;
    struct sched_param schedParams;
    pthread_t task;
    ti_ndk_os_TaskArgs *taskArgs;

    taskArgs = (ti_ndk_os_TaskArgs *)malloc(sizeof(ti_ndk_os_TaskArgs));
    if (!taskArgs) {
        DbgPrintf(DBG_WARN, "TaskCreate: couldn't allocate pthread fxn args\n");
        return (NULL);
    }
    taskArgs->arg0 = pFun;
    taskArgs->arg1 = Arg1;
    taskArgs->arg2 = Arg2;

    pthread_attr_init(&pthreadAttrs);

    schedParams.sched_priority = Priority;
    status = pthread_attr_setschedparam(&pthreadAttrs, &schedParams);
    if (status != 0) {
        DbgPrintf(DBG_WARN,
                "TaskCreate: pthread_attr_setschedparam failed (%d)\n", status);
        return (NULL);
    }

    status = pthread_attr_setstacksize(&pthreadAttrs, StackSize);
    if (status != 0) {
        DbgPrintf(DBG_WARN,
                "TaskCreate: pthread_attr_setstacksize failed (%d)\n", status);
        return (NULL);
    }

    /* use PTHREAD_CREATE_DETACHED as we don't support TaskJoin in NDK OSAL */
    status = pthread_attr_setdetachstate(&pthreadAttrs,
            PTHREAD_CREATE_DETACHED);
    if (status != 0) {
        DbgPrintf(DBG_WARN,
                "TaskCreate: pthread_attr_setdetachstate failed (%d)\n",
                status);
        return (NULL);
    }

    status = pthread_create(&task, &pthreadAttrs,
            ti_ndk_os_threadStub, (void *)taskArgs);
    if (status != 0) {
        DbgPrintf(DBG_WARN, "TaskCreate: pthread_create failed (%d)\n", status);
        return (NULL);
    }

    pthread_attr_destroy(&pthreadAttrs);
    if (status != 0) {
        DbgPrintf(DBG_WARN,
                "TaskCreate: pthread_attr_destroy failed (%d)\n",
                status);
        return (NULL);
    }

    return ((void *)task);
}

/*-------------------------------------------------------------------- */
/* TaskDestroy() */
/* Destroy a task */
/*-------------------------------------------------------------------- */
void TaskDestroy( void *h )
{
    if (h == TaskSelf()) {
        TaskExit();
    }
    else {
        pthread_cancel((pthread_t)h);
    }
}

/*-------------------------------------------------------------------- */
/* TaskSetEnv */
/* Sets the task's Environment pointer */
/*-------------------------------------------------------------------- */
void TaskSetEnv( void *h, int Slot, void *hEnv )
{
    if( Slot ) {
        return;
    }
    if( !hookOK )
    {
        DbgPrintf(DBG_ERROR,"TaskSetEnv: FATAL: NDK_hookCreate() must be set in BIOS Task module config (!hookOK)");
        return;
    }

    pthread_setspecific(key, hEnv);
}

/*-------------------------------------------------------------------- */
/* TaskGetEnv */
/* Gets the task's Environment pointer */
/*-------------------------------------------------------------------- */
void *TaskGetEnv( void *h, int Slot )
{
    void *env = NULL;

    if( Slot ) {
        return (0);
    }
    if( !hookOK )
    {
        DbgPrintf(DBG_ERROR,"TaskGetEnv: FATAL: NDK_hookCreate() must be set in DSP/BIOS Task module config (!hookOK)");
        return (0);
    }
    env = pthread_getspecific(key);
    return (env);
}

/*-------------------------------------------------------------------- */
/* TaskExit() */
/* Exits and destroys a task */
/*-------------------------------------------------------------------- */
void TaskExit()
{
    pthread_exit(NULL);
}

/*-------------------------------------------------------------------- */
/* TaskGetPri() */
/* Get a task's priority */
/*-------------------------------------------------------------------- */
int TaskGetPri(void *h)
{
    struct sched_param param;
    int policy;
    int status;
    int retval;

    status = pthread_getschedparam((pthread_t)h, &policy, &param);
    if (status != 0) {
        DbgPrintf(DBG_WARN,
                "TaskGetPri: could not get thread priority (%d)\n", status);
        retval = OS_TASKGETPRIFAIL;
    }
    else {
        retval = param.sched_priority;
    }

    return (retval);
}

/*-------------------------------------------------------------------- */
/* TaskSetPri() */
/* Set a task's priority */
/*-------------------------------------------------------------------- */
int TaskSetPri(void *h, int priority)
{
    int status;
    int oldPri;
    struct sched_param param;
    param.sched_priority = priority;

    /* save the old thread priority */
    oldPri = TaskGetPri(h);
    if (oldPri == OS_TASKGETPRIFAIL) {
        oldPri = OS_TASKSETPRIFAIL;
    }
    else {
        /* set the thread priority */
        status = pthread_setschedparam((pthread_t)h, SCHED_OTHER, &param);
        if (status != 0) {
            DbgPrintf(DBG_WARN,
                    "TaskSetPri: could not set thread priority (%d)\n", status);
            oldPri = OS_TASKSETPRIFAIL;
        }
    }
    return (oldPri);
}

/*-------------------------------------------------------------------- */
/* TaskSelf() */
/* Return handle of task itself  */
/*-------------------------------------------------------------------- */
void *TaskSelf()
{
    return ((void *)pthread_self());
}

/*-------------------------------------------------------------------- */
/* TaskSleep() */
/* Put a task into sleep  */
/*-------------------------------------------------------------------- */
void TaskSleep(uint32_t delay)
{
    if (delay >= 1000) {
        sleep(delay / 1000);
    }
    else {
        usleep(delay * 1000);
    }
}

/*-------------------------------------------------------------------- */
/* TaskYield() */
/* Yield task  */
/*-------------------------------------------------------------------- */
void TaskYield()
{
     sched_yield();
}

/*
 * ======== ti_ndk_config_Global_startupFxn ========
 * Called to start up the NDK in Linux.
 * In Linux, this should be called from main()
 */

void ti_ndk_config_Global_startupFxn()
{
    void *ndkThread;

    ndkThread = TaskCreate(ti_ndk_config_Global_stackThread, NULL, 5, PTHREAD_STACK_MIN, 0, 0, 0);
    if (!ndkThread) {
        DbgPrintf(DBG_ERROR,"Error: could not create NDK stack thread");
    }
}
