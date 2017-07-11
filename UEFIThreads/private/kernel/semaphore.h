/* semaphore.h
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

#ifndef __KERNEL_SEMAPHORE_H
#define __KERNEL_SEMAPHORE_H

#include "private/kernel/thread.h"
#include "private/kernel/mutex.h"

#define SEMAPHORE_MAGIC (0x73656D61) // 'sema'

typedef struct semaphore {
    INTN magic;
    INTN count;
    wait_queue_t wait;
} semaphore_t;

#define SEMAPHORE_INITIAL_VALUE(s, _count) \
{ \
    .magic = SEMAPHORE_MAGIC, \
    .count = _count, \
    .wait = WAIT_QUEUE_INITIAL_VALUE((s).wait), \
}

VOID sem_init(semaphore_t *, UINTN);
VOID sem_destroy(semaphore_t *);
INTN sem_post(semaphore_t *, BOOLEAN resched);
EFI_STATUS sem_wait(semaphore_t *);
EFI_STATUS sem_trywait(semaphore_t *);
EFI_STATUS sem_timedwait(semaphore_t *, THREAD_TIME_MS);

#endif
