/* semaphore.c
 *
 * Copyright 2012 Christopher Anderson <chris@nullcode.org>
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "private/kernel/semaphore.h"
#include "private/kernel/thread.h"

VOID sem_init(semaphore_t *sem, UINTN value)
{
    *sem = (semaphore_t)SEMAPHORE_INITIAL_VALUE(*sem, value);
}

VOID sem_destroy(semaphore_t *sem)
{
    THREAD_LOCK(state);
    sem->count = 0;
    wait_queue_destroy(&sem->wait, TRUE);
    THREAD_UNLOCK(state);
}

INTN sem_post(semaphore_t *sem, BOOLEAN resched)
{
    INTN ret = 0;

    THREAD_LOCK(state);

    /*
     * If the count is or was negative then a thread is waiting for a resource, otherwise
     * it's safe to just increase the count available with no downsides
     */
    if (unlikely(++sem->count <= 0))
        ret = wait_queue_wake_one(&sem->wait, resched, EFI_SUCCESS);

    THREAD_UNLOCK(state);

    return ret;
}

EFI_STATUS sem_wait(semaphore_t *sem)
{
    EFI_STATUS ret = EFI_SUCCESS;
    THREAD_LOCK(state);

    /*
     * If there are no resources available then we need to
     * sit in the wait queue until sem_post adds some.
     */
    if (unlikely(--sem->count < 0))
        ret = wait_queue_block(&sem->wait, INFINITE_TIME);

    THREAD_UNLOCK(state);
    return ret;
}

EFI_STATUS sem_trywait(semaphore_t *sem)
{
    EFI_STATUS ret = EFI_SUCCESS;
    THREAD_LOCK(state);

    if (unlikely(sem->count <= 0))
        ret = EFI_NOT_READY;
    else
        sem->count--;

    THREAD_UNLOCK(state);
    return ret;
}

EFI_STATUS sem_timedwait(semaphore_t *sem, THREAD_TIME_MS timeout)
{
    EFI_STATUS ret = EFI_SUCCESS;
    THREAD_LOCK(state);

    if (unlikely(--sem->count < 0)) {
        ret = wait_queue_block(&sem->wait, timeout);
        if (EFI_ERROR(ret)) {
            if (ret == EFI_TIMEOUT) {
                sem->count++;
            }
        }
    }

    THREAD_UNLOCK(state);
    return ret;
}
