/*
 * Copyright (c) 2012-2017, Texas Instruments Incorporated
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
 * ======== semaphore.c ========
 *
 * Semaphore management using BIOS6
 *
 */
// TODO: this (netmain) undefines __USE_UNIX98 somewhere ...
#include <netmain.h>
#include <_stack.h>
#include <_oskern.h>

#include <stdlib.h>
#include <time.h>
#include <errno.h>

#include <semaphore.h>

/* needed for PTHREAD_PRIO_NONE */
#ifndef __USE_UNIX98
#define __USE_UNIX98
#endif
#include <pthread.h>

#define TI_NDK_OS_SEM_MAGIC_KEY 0xCAB0CAB0
#define TI_NDK_OS_BINARY_SEM_MAGIC_KEY 0xABCDABCD

/* binary semaphore structure */
typedef struct ti_ndk_os_binary_sem {
    pthread_mutex_t mutex;
    pthread_cond_t cvar;
    int v; // TODO: rename this to 'count'
} ti_ndk_os_binary_sem;

/*
 *  ======== relTimeToAbs ========
 *  Convert relative time to absolute time
 */
static void relTimeToAbs(int msecs, struct timespec *abstime)
{
    long remMs = 0;
    long secs = 0;

    /* Get the time since the beginning of the Epoch */
#if defined(__linux__)
    /*
     * Fix for select() returning immediately (Telnet console closed instantly
     * after opening due to this, too).
     *
     * Linux sem_timedwait() uses real time, not monotonic! (worse, it seems
     * the clock used is system dependent).  On my system, a timespec with time
     * computed from the monotonic clock fails for Linux (works in BIOS). This
     * is b/c for real time on Linux, time since the epoch is returned, which
     * will be a large value for tv_sec.  But for monotonic, tv_sec will be much
     * smaller.  This caused sem_timedwait to return ETIMEDOUT, which now makes
     * sense because the small value of seconds from the monotonic clock is
     * sometime in the 70s relative to the epoch, and so timed out a long time
     * ago.
     * So for Linux, had to use the real time clock.
     *
     * Note that once the following issue is fixed in BIOS, REALTIME can be
     * used for both cases:
     *
     * SYSBIOS-321 Posix timeouts should be based on CLOCK_REALTIME
     */
    clock_gettime(CLOCK_REALTIME, abstime);
#else
    clock_gettime(CLOCK_MONOTONIC, abstime);
#endif

    secs = msecs / 1000;
    remMs = msecs % 1000;

    abstime->tv_sec += secs;
    abstime->tv_nsec += remMs * 1000000;

    if (abstime->tv_nsec >= 1000000000) {
        abstime->tv_sec++;
        abstime->tv_nsec -= 1000000000;
    }
}

/*
 *  ======== SemCreate ========
 *  Create a semaphore.
 */
void *SemCreate(int Count)
{
    uint32_t *ptr;
    sem_t *sem;
    
    /* allocate enough space for the magic key and the semaphore */
    ptr = (uint32_t *)malloc(sizeof(uint32_t) + sizeof(sem_t));
    if (!ptr) {
        DbgPrintf(DBG_ERROR,"SemCreate(): could not create semaphore");
    }
    else {
        /* store magic # to ensure SemPend/Post/Delete are called correctly */
        *ptr = TI_NDK_OS_SEM_MAGIC_KEY;

        sem = (sem_t *)(ptr + 1);

        memset(sem, 0, sizeof(sem_t));

        /* create the new semaphore */
        // TODO: Count is a signed int, but sem_init param is unsigned int ...
        /*
         * pass zero for 'pshared' as this is a don't care for BIOS. What about
         * for Linux, though? (TODO)
         */
        sem_init(sem, 0, Count);
    }
    return ((void *)sem);
}

/*
 *  ======== SemCreateBinary ========
 *  Create a binary semaphore.
 */
void *SemCreateBinary(int Count)
{
    int retc = 0;
    uint32_t *ptr;
    pthread_mutexattr_t mtxAttrs;
    pthread_condattr_t cndAttrs;
    ti_ndk_os_binary_sem *sem;

    /* allocate enough space for the magic key and the binary sem */
    ptr = (uint32_t *)malloc(sizeof(uint32_t) + sizeof(ti_ndk_os_binary_sem));

    if (!ptr) {
        DbgPrintf(DBG_ERROR, "SemCreateBinary(): could not create semaphore\n");
        return (NULL);
    }

    /* store magic # to ensure SemPend/PostBinary are called correctly */
    *ptr = TI_NDK_OS_BINARY_SEM_MAGIC_KEY;

    sem = (ti_ndk_os_binary_sem *)(ptr + 1);

    retc = pthread_mutexattr_init(&mtxAttrs);
    if (retc) {
        DbgPrintf(DBG_ERROR,
                "SemCreateBinary(): couldn't initialize mtxAttrs (%d)\n", retc);
        free(ptr);
        return (NULL);
    }

    retc = pthread_condattr_init(&cndAttrs);
    if (retc) {
        DbgPrintf(DBG_ERROR,
                "SemCreateBinary(): couldn't initialize cndAttrs (%d)\n", retc);
        free(ptr);
        return (NULL);
    }

    pthread_mutexattr_settype(&mtxAttrs, PTHREAD_MUTEX_NORMAL);
    pthread_mutexattr_setprotocol(&mtxAttrs, PTHREAD_PRIO_NONE);

    // TODO: check ret val
    pthread_mutex_init(&sem->mutex, &mtxAttrs);
    pthread_cond_init(&sem->cvar, &cndAttrs);

    pthread_mutexattr_destroy(&mtxAttrs);
    pthread_condattr_destroy(&cndAttrs);

    sem->v = Count;

    return ((void *)sem);
}

/*
 *  ======== SemDeleteBinary ========
 *  Delete a binary semaphore.
 */
void SemDeleteBinary(void *hSem)
{
    uint32_t *ptr = (uint32_t *)hSem;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemDeleteBinary(): error: semaphore is NULL\n");
        return;
    }

    /* verify this is an NDK binary semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_BINARY_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_WARN,
                "SemDeleteBinary(): error: deleting an invalid semaphore!\n");
        return;
    }
    pthread_cond_destroy(&((ti_ndk_os_binary_sem *)hSem)->cvar);
    pthread_mutex_destroy(&((ti_ndk_os_binary_sem *)hSem)->mutex);
    free(ptr);
}


/*
 *  ======== SemDelete ========
 *  Delete a semaphore.
 */
void SemDelete(void *hSem)
{
    uint32_t *ptr = (uint32_t *)hSem;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemDelete(): error: semaphore is NULL\n");
        return;
    }

    /* verify this is a 'regular' semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_WARN,
            "SemDelete(): error: deleting an invalid semaphore!\n");
        return;
    }
    sem_destroy((sem_t *)hSem);
    free(ptr);
}

/*
 *  ======== SemCount ========
 *  Get the current semaphore count.
 */
int SemCount(void *hSem)
{
    uint32_t *ptr = (uint32_t *)hSem;
    int value = 0;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemCount(): error: semaphore is NULL\n");
        return (0);
    }

    ptr = ptr - 1;
    if (ptr && *ptr == TI_NDK_OS_BINARY_SEM_MAGIC_KEY) {
        value = ((ti_ndk_os_binary_sem *)hSem)->v;
    }
    else if (ptr && *ptr == TI_NDK_OS_SEM_MAGIC_KEY) {
        /* sem_getvalue() always returns 0 so no need to check return value */
        sem_getvalue((sem_t *)hSem, &value);
    }
    else {
        DbgPrintf(DBG_WARN, "SemCount(): error: invalid semaphore\n");
    }
    return (value);
}

/*
 *  ======== SemPendBinary ========
 *  Wait for a binary semaphore.
 */
int SemPendBinary(void *hSem, uint32_t Timeout)
{
    uint32_t *ptr = (uint32_t *)hSem;
    ti_ndk_os_binary_sem *sem;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemPendBinary(): error: semaphore is NULL\n");
        return (0);
    }

    /* verify this is an NDK binary semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_BINARY_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_WARN,
                "SemPendBinary(): error: pending on an invalid semaphore!\n");
        return (0);
    }

    sem = (ti_ndk_os_binary_sem *)hSem;

    pthread_mutex_lock(&sem->mutex);
    while (!sem->v) {
        pthread_cond_wait(&sem->cvar, &sem->mutex);
    }
    sem->v = 0;
    pthread_mutex_unlock(&sem->mutex);
    return (1);
}

/*
 *  ======== SemPend ========
 *  Wait for a semaphore.
 */
int SemPend(void *hSem, uint32_t Timeout)
{
    int retval;
    struct timespec ts;
    uint32_t *ptr = (uint32_t *)hSem;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemPend(): error: semaphore is NULL\n");
        return (0);
    }

    /* verify this is a 'regular' semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_WARN,
            "SemPend(): error: pending on invalid semaphore\n");
        return (0);
    }

    if (Timeout == SEM_FOREVER) {
        retval = sem_wait((sem_t *)hSem);
    }
    else {
        /* sem_timedwait() requires an absolute time */
        relTimeToAbs(Timeout, &ts);
        retval = sem_timedwait((sem_t *)hSem, &ts);
    }

    if (retval == -1) {
        retval = 1;
    }

    return ((retval == 0) ? 1 : 0);
}

/*
 *  ======== SemPostBinary ========
 *  Signal a binary semaphore.
 */
void SemPostBinary(void *hSem)
{
    uint32_t *ptr = (uint32_t *)hSem;
    ti_ndk_os_binary_sem *sem;

    if (!ptr) {
        // TODO: change the severity back to WARN here (after you test Janet's fix)
        DbgPrintf(DBG_ERROR, "SemPostBinary(): error: semaphore is NULL\n");
        return;
    }

    /* verify this is an NDK binary semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_BINARY_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_ERROR,
                "SemPostBinary(): error: posting an invalid semaphore!\n");
        return;
    }

    sem = (ti_ndk_os_binary_sem *)hSem;

    if (pthread_mutex_trylock(&sem->mutex)) {
        /* scheduler currently being signaled by another event */
        return;
    }
    else if (sem->v == 1) {
        /* binary sem has already been posted */
        pthread_mutex_unlock(&sem->mutex);
        return;
    }
    sem->v = 1;
    pthread_cond_signal(&sem->cvar);
    pthread_mutex_unlock(&sem->mutex);
}

/*
 *  ======== SemPost ========
 *  Signal a semaphore.
 */
void SemPost(void *hSem)
{
    uint32_t *ptr = (uint32_t *)hSem;

    if (!ptr) {
        // TODO: change the severity back to WARN here (after you test Janet's fix)
        DbgPrintf(DBG_ERROR, "SemPost(): error: semaphore is NULL\n");
        return;
    }

    /* verify this is a 'regular' semaphore */
    ptr = ptr - 1;
    if (!ptr || *ptr != TI_NDK_OS_SEM_MAGIC_KEY) {
        DbgPrintf(DBG_ERROR,
            "SemPost(): error: posting an invalid semaphore!\n");
        return;
    }
    sem_post((sem_t *)hSem);
}

/*
 *  ======== SemReset ========
 *  Reset a semaphore's count.
 *  TODO: is this implementation right? It's assuming that the caller wants
 *  to *drain* the sem count ... however, they may want to bring it *up* (in
 *  the case where 'Count > SemCount(hSem)'. May need to SemPost() until sem's)
 *  count == Count ...?)
 */
void SemReset(void *hSem, int Count)
{
    int retval = 0;
    int actualCount;
    uint32_t *ptr = (uint32_t *)hSem;

    if (!ptr) {
        DbgPrintf(DBG_WARN, "SemReset(): error: semaphore is NULL\n");
        return;
    }

    ptr = ptr - 1;
    if (ptr && *ptr == TI_NDK_OS_BINARY_SEM_MAGIC_KEY) {
        /* setting count to 0 is only option for binary semaphore */
        ((ti_ndk_os_binary_sem *)hSem)->v = 0;
    }
    else if (ptr && *ptr == TI_NDK_OS_SEM_MAGIC_KEY) {
        actualCount = SemCount(hSem);

        /* SemReset should bring the sem's count down to the user's desired value */
        while ((actualCount > Count) && (retval == 0)) {
            /* drain the count without blocking */
            retval = sem_trywait((sem_t *)hSem);
        }
    }
    else {
        DbgPrintf(DBG_WARN, "SemReset(): error: invalid semaphore\n");
    }
}
