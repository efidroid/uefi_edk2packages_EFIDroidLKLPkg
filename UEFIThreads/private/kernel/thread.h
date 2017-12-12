/*
 * Copyright (c) 2008-2015 Travis Geiselbrecht
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
#ifndef __KERNEL_THREAD_H
#define __KERNEL_THREAD_H

#define SMP_MAX_CPUS 1

#include <Protocol/UefiThread.h>

#include "private/kernel/spinlock.h"
#include "private/UContext.h"
#include "private/list.h"
#include "private/wait.h"
#include "private/compat.h"

/* debug-enable runtime checks */
//#define THREAD_STACK_HIGHWATER 1
//#define THREAD_STACK_BOUNDS_CHECK 1
//#ifndef THREAD_STACK_PADDING_SIZE
//#define THREAD_STACK_PADDING_SIZE 256
//#endif

enum thread_state {
    THREAD_SUSPENDED = 0,
    THREAD_READY,
    THREAD_RUNNING,
    THREAD_BLOCKED,
    THREAD_SLEEPING,
    THREAD_DEATH,
};

typedef struct {
  struct list_node node;

  TLS_DESTRUCTOR destructor;
} tls_item_t;

typedef struct {
    struct list_node node;

    tls_item_t *tls;
    VOID *data;
} tls_value_t;

#define THREAD_FLAG_DETACHED                  (1<<0)
#define THREAD_FLAG_FREE_STACK                (1<<1)
#define THREAD_FLAG_FREE_STRUCT               (1<<2)
#define THREAD_FLAG_REAL_TIME                 (1<<3)
#define THREAD_FLAG_IDLE                      (1<<4)
#define THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK  (1<<5)

#define THREAD_MAGIC (0x74687264) // 'thrd'

typedef struct thread {
    INTN magic;
    struct list_node thread_list_node;

    /* active bits */
    struct list_node queue_node;
    INTN priority;
    enum thread_state state;
    INTN remaining_quantum;
    UINTN flags;
#if WITH_SMP
    INTN curr_cpu;
    INTN pinned_cpu; /* only run on pinned_cpu if >= 0 */
#endif
    SPINLOCK_THREAD_DATA SpinlockThreadData;

    /* if blocked, a pointer to the wait queue */
    struct wait_queue *blocking_wait_queue;
    EFI_STATUS wait_queue_block_ret;

    UCONTEXT context;

    /* stack stuff */
    VOID *stack;
    UINTN stack_size;

    /* entry point */
    THREAD_START_ROUTINE entry;
    VOID *arg;

    /* return code */
    INTN retcode;
    struct wait_queue retcode_wait_queue;

    /* thread local storage */
    struct list_node tls_values;

    CHAR8 name[32];
} thread_t;

#if WITH_SMP
#define thread_curr_cpu(t) ((t)->curr_cpu)
#define thread_pinned_cpu(t) ((t)->pinned_cpu)
#define thread_set_curr_cpu(t,c) ((t)->curr_cpu = (c))
#define thread_set_pinned_cpu(t, c) ((t)->pinned_cpu = (c))
#else
#define thread_curr_cpu(t) (0)
#define thread_pinned_cpu(t) (-1)
#define thread_set_curr_cpu(t,c) do {} while(0)
#define thread_set_pinned_cpu(t, c) do {} while(0)
#endif

/* functions */
VOID thread_init_early(VOID);
VOID thread_init(VOID);
VOID thread_create_idle(VOID);
VOID thread_set_name(thread_t *t, CONST CHAR8 *name);
VOID thread_set_priority(INTN priority);
thread_t *thread_create(CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, UINTN stack_size);
thread_t *thread_create_etc(thread_t *t, CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, VOID *stack, UINTN stack_size);
EFI_STATUS thread_resume(thread_t *);
VOID thread_exit(INTN retcode) NORETURN;
VOID thread_sleep(THREAD_TIME_MS delay);
EFI_STATUS thread_detach(thread_t *t);
EFI_STATUS thread_join(thread_t *t, INTN *retcode, THREAD_TIME_MS timeout);
EFI_STATUS thread_detach_and_resume(thread_t *t);
EFI_STATUS thread_set_real_time(thread_t *t);

VOID dump_thread(thread_t *t);
VOID arch_dump_thread(thread_t *t);
VOID dump_all_threads(VOID);

/* scheduler routines */
VOID thread_yield(VOID); /* give up the cpu voluntarily */
VOID thread_preempt(VOID); /* get preempted (inserted into head of run queue) */
VOID thread_block(VOID); /* block on something and reschedule */
VOID thread_unblock(thread_t *t, BOOLEAN resched); /* go back in the run queue */

/* called on every timer tick for the scheduler to do quantum expiration */
enum handler_return thread_timer_tick(VOID);

/* the current thread */
extern struct thread *_current_thread;

STATIC inline thread_t *get_current_thread(VOID)
{
    return _current_thread;
}

STATIC inline VOID set_current_thread(thread_t *t)
{
    _current_thread = t;
}

/* scheduler lock */
extern SPINLOCK thread_lock;

#define THREAD_LOCK(state) SPINLOCK_STATE state; spin_lock_irqsave(&thread_lock, state)
#define THREAD_UNLOCK(state) spin_unlock_irqrestore(&thread_lock, state)

STATIC inline BOOLEAN thread_lock_held(VOID)
{
    return spin_lock_held(&thread_lock);
}

/* thread local storage */
EFI_STATUS tls_create(UINTN *pkey, TLS_DESTRUCTOR destructor);
EFI_STATUS tls_delete(UINTN key);
EFI_STATUS tls_set(UINTN key, VOID *data);
VOID* tls_get(UINTN key);

#endif
