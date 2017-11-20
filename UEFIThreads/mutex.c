/*
 * Copyright (c) 2008-2014 Travis Geiselbrecht
 * Copyright (c) 2012-2012 Shantanu Gupta
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file
 * @brief  Mutex functions
 *
 * @defgroup mutex Mutex
 * @{
 */

#include "private/kernel/mutex.h"
#include "private/kernel/thread.h"

/**
 * @brief  Initialize a mutex_t
 */
VOID mutex_init(mutex_t *m)
{
    *m = (mutex_t)MUTEX_INITIAL_VALUE(*m);
}

/**
 * @brief  Destroy a mutex_t
 *
 * This function frees any resources that were allocated
 * in mutex_init().  The mutex_t object itself is not freed.
 */
VOID mutex_destroy(mutex_t *m)
{
    ASSERT(m->magic == MUTEX_MAGIC);

    if (unlikely(m->holder != 0 && get_current_thread() != m->holder)) {
        DEBUG((EFI_D_ERROR, "mutex_destroy: thread %p (%s) tried to release mutex %p it doesn't own. owned by %p (%s)\n",
              get_current_thread(), get_current_thread()->name, m, m->holder, m->holder->name));
        ASSERT(FALSE);
    }

    THREAD_LOCK(state);
    m->magic = 0;
    m->count = 0;
    wait_queue_destroy(&m->wait, TRUE);
    THREAD_UNLOCK(state);
}

/**
 * @brief  Mutex wait with timeout
 *
 * This function waits up to \a timeout ms for the mutex to become available.
 * Timeout may be zero, in which case this function returns immediately if
 * the mutex is not free.
 *
 * @return  EFI_SUCCESS on success, EFI_TIMEOUT on timeout,
 * other values on error
 */
EFI_STATUS mutex_acquire_timeout(mutex_t *m, THREAD_TIME_MS timeout)
{
    ASSERT(m->magic == MUTEX_MAGIC);

    if (unlikely(get_current_thread() == m->holder)) {
        DEBUG((EFI_D_ERROR, "mutex_acquire_timeout: thread %p (%s) tried to acquire mutex %p it already owns.\n",
              get_current_thread(), get_current_thread()->name, m));
        ASSERT(FALSE);
    }

    THREAD_LOCK(state);

    EFI_STATUS ret = EFI_SUCCESS;
    if (unlikely(++m->count > 1)) {
        ret = wait_queue_block(&m->wait, timeout);
        if (unlikely(ret != EFI_SUCCESS)) {
            /* if the acquisition timed out, back out the acquire and exit */
            if (likely(ret == EFI_TIMEOUT)) {
                /*
                 * race: the mutex may have been destroyed after the timeout,
                 * but before we got scheduled again which makes messing with the
                 * count variable dangerous.
                 */
                m->count--;
            }
            /* if there was a general error, it may have been destroyed out from
             * underneath us, so just exit (which is really an invalid state anyway)
             */
            goto err;
        }
    }

    m->holder = get_current_thread();

err:
    THREAD_UNLOCK(state);
    return ret;
}

/**
 * @brief  Release mutex
 */
EFI_STATUS mutex_release(mutex_t *m)
{
    ASSERT(m->magic == MUTEX_MAGIC);

    if (unlikely(get_current_thread() != m->holder)) {
        DEBUG((EFI_D_ERROR, "mutex_release: thread %p (%s) tried to release mutex %p it doesn't own. owned by %p (%s)\n",
              get_current_thread(), get_current_thread()->name, m, m->holder, m->holder ? m->holder->name : "none"));
        ASSERT(FALSE);
    }

    THREAD_LOCK(state);

    m->holder = 0;

    if (unlikely(--m->count >= 1)) {
        /* release a thread */
        wait_queue_wake_one(&m->wait, TRUE, EFI_SUCCESS);
    }

    THREAD_UNLOCK(state);
    return EFI_SUCCESS;
}

