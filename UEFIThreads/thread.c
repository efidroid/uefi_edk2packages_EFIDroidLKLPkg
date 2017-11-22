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

/**
 * @file
 * @brief  Kernel threading
 *
 * This file is the core kernel threading interface.
 *
 * @defgroup thread Threads
 * @{
 */

#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include "private/list.h"
#include "private/kernel/thread.h"
#include "private/kernel/dpc.h"

#define STACK_DEBUG_BYTE (0x99)
#define STACK_DEBUG_WORD (0x99999999)

#define DEBUG_THREAD_CONTEXT_SWITCH 0

typedef struct {
    EFI_EVENT Event;
    thread_t *thread;
} TIMER_CONTEXT;

/* global thread list */
STATIC struct list_node thread_list;

/* master thread spinlock */
SPINLOCK thread_lock = SPIN_LOCK_INITIAL_VALUE;

/* the run queue */
STATIC struct list_node run_queue[NUM_PRIORITIES];
STATIC UINT32 run_queue_bitmap;

/* the idle thread(s) (statically allocated) */
#if WITH_SMP
STATIC thread_t _idle_threads[SMP_MAX_CPUS];
#define idle_thread(cpu) (&_idle_threads[cpu])
#else
STATIC thread_t _idle_thread;
#define idle_thread(cpu) (&_idle_thread)
#endif

struct thread *_current_thread = NULL;

STATIC UINTN tls_next_key = 1;

/* local routines */
STATIC VOID thread_resched(VOID);
STATIC VOID idle_thread_routine(VOID) NORETURN;

STATIC VOID dpc_heap_free_cb(VOID *arg)
{
  FreePool(arg);
}

/* run queue manipulation */
STATIC VOID insert_in_run_queue_head(thread_t *t)
{
    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state == THREAD_READY);
    ASSERT(!list_in_list(&t->queue_node));
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    list_add_head(&run_queue[t->priority], &t->queue_node);
    run_queue_bitmap |= (1<<t->priority);
}

STATIC VOID insert_in_run_queue_tail(thread_t *t)
{
    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state == THREAD_READY);
    ASSERT(!list_in_list(&t->queue_node));
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    list_add_tail(&run_queue[t->priority], &t->queue_node);
    run_queue_bitmap |= (1<<t->priority);
}

STATIC VOID init_thread_struct(thread_t *t, CONST CHAR8 *name)
{
    SetMem(t, sizeof(thread_t), 0);
    t->magic = THREAD_MAGIC;
    thread_set_pinned_cpu(t, -1);
    AsciiStrCpyS(t->name, sizeof(t->name), name);
}

STATIC VOID initial_thread_func(VOID)
{
    INTN ret;

    /* release the thread lock that was implicitly held across the reschedule */
    spin_unlock(&thread_lock);
    gBS->RestoreTPL(TPL_CALLBACK);

    thread_t *ct = get_current_thread();
    ret = ct->entry(ct->arg);

    thread_exit(ret);
}

STATIC tls_entry_t * internal_tls_get(thread_t *t, UINTN key) {
    tls_entry_t *tls_entry;
    list_for_every_entry(&t->tls_list, tls_entry, tls_entry_t, node) {
        if (tls_entry->key == key)
            return tls_entry;
    }

    return NULL;
}

EFI_STATUS tls_create(UINTN *pkey, TLS_DESTRUCTOR destructor) {
    if (pkey==NULL)
        return EFI_INVALID_PARAMETER;

    THREAD_LOCK(state);
    UINTN key = tls_next_key++;

    thread_t *t;
    list_for_every_entry(&thread_list, t, thread_t, thread_list_node) {
        tls_entry_t *tls_entry = AllocatePool(sizeof(tls_entry_t));
        if (tls_entry == NULL) {
            THREAD_UNLOCK(state);
            return EFI_OUT_OF_RESOURCES;
        }

        tls_entry->key = key;
        list_add_tail(&t->tls_list, &tls_entry->node);

        tls_entry->data = NULL;
        tls_entry->destructor = destructor;
    }

    THREAD_UNLOCK(state);

    *pkey = key;

    return EFI_SUCCESS;
}

EFI_STATUS tls_delete(UINTN key) {
    thread_t *t;

    THREAD_LOCK(state);
    list_for_every_entry(&thread_list, t, thread_t, thread_list_node) {
        tls_entry_t *tls_entry = internal_tls_get(t, key);
        if (tls_entry == NULL) {
            THREAD_UNLOCK(state);
            return EFI_NOT_FOUND;
        }
        list_delete(&tls_entry->node);

        if (tls_entry->data && tls_entry->destructor)
            tls_entry->destructor(tls_entry->data);
        FreePool(tls_entry);
    }
    THREAD_UNLOCK(state);

    return EFI_SUCCESS;
}

EFI_STATUS tls_set(UINTN key, VOID *data) {
    thread_t *current_thread = get_current_thread();

    tls_entry_t *tls_entry = internal_tls_get(current_thread, key);
    if (tls_entry == NULL) {
        return EFI_NOT_FOUND;
    }
    tls_entry->data = data;

    return EFI_SUCCESS;
}

VOID* tls_get(UINTN key) {
    thread_t *current_thread = get_current_thread();

    tls_entry_t *tls_entry = internal_tls_get(current_thread, key);
    if (tls_entry == NULL) {
        return NULL;
    }

    return tls_entry->data;
}

/**
 * @brief  Create a new thread
 *
 * This function creates a new thread.  The thread is initially suspended, so you
 * need to call thread_resume() to execute it.
 *
 * @param  name        Name of thread
 * @param  entry       Entry point of thread
 * @param  arg         Arbitrary argument passed to entry()
 * @param  priority    Execution priority for the thread.
 * @param  stack_size  Stack size for the thread.
 *
 * Thread priority is an integer from 0 (lowest) to 31 (highest).  Some standard
 * prioritys are defined in <kernel/thread.h>:
 *
 *  HIGHEST_PRIORITY
 *  DPC_PRIORITY
 *  HIGH_PRIORITY
 *  DEFAULT_PRIORITY
 *  LOW_PRIORITY
 *  IDLE_PRIORITY
 *  LOWEST_PRIORITY
 *
 * Stack size is typically set to DEFAULT_STACK_SIZE
 *
 * @return  Pointer to thread object, or NULL on failure.
 */
thread_t *thread_create_etc(thread_t *t, CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, VOID *stack, UINTN stack_size)
{
    UINTN flags = 0;
    tls_entry_t *tls_entry;

    if (!t) {
        t = AllocatePool(sizeof(thread_t));
        if (!t)
            return NULL;
        flags |= THREAD_FLAG_FREE_STRUCT;
    }

    init_thread_struct(t, name);

    t->entry = entry;
    t->arg = arg;
    t->priority = priority;
    t->state = THREAD_SUSPENDED;
    t->blocking_wait_queue = NULL;
    t->wait_queue_block_ret = EFI_SUCCESS;
    t->SpinlockThreadData.OldTpl = 0;
    t->SpinlockThreadData.OldTplIsValid = FALSE;
    thread_set_curr_cpu(t, -1);

    t->retcode = 0;
    wait_queue_init(&t->retcode_wait_queue);

    /* create the stack */
    if (!stack) {
#if THREAD_STACK_BOUNDS_CHECK
        stack_size += THREAD_STACK_PADDING_SIZE;
        flags |= THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK;
#endif
        t->stack = AllocatePool(stack_size);
        if (!t->stack) {
            if (flags & THREAD_FLAG_FREE_STRUCT)
                FreePool(t);
            return NULL;
        }
        flags |= THREAD_FLAG_FREE_STACK;
#if THREAD_STACK_BOUNDS_CHECK
        SetMem(t->stack, THREAD_STACK_PADDING_SIZE, STACK_DEBUG_BYTE);
#endif
    } else {
        t->stack = stack;
    }
#if THREAD_STACK_HIGHWATER
    if (flags & THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK) {
        SetMem(t->stack + THREAD_STACK_PADDING_SIZE,
               stack_size - THREAD_STACK_PADDING_SIZE, STACK_DEBUG_BYTE);
    } else {
        SetMem(t->stack, stack_size, STACK_DEBUG_BYTE);
    }
#endif

    t->stack_size = stack_size;

    /* save whether or not we need to free the thread struct and/or stack */
    t->flags = flags;

    /* inheirit thread local storage from the parent */
    list_initialize(&t->tls_list);
    thread_t *current_thread = get_current_thread();

    list_for_every_entry(&current_thread->tls_list, tls_entry, tls_entry_t, node) {
        tls_entry_t *new_tls_entry = AllocatePool(sizeof(tls_entry_t));
        ASSERT(new_tls_entry);
        CopyMem(new_tls_entry, tls_entry, sizeof(*new_tls_entry));
        new_tls_entry->data = NULL;

        list_add_tail(&t->tls_list, &new_tls_entry->node);
    }

    // init context
    GetContext(&t->context);

    // set entrypoint
    MakeContext(&t->context, initial_thread_func, t->stack, t->stack_size);

    /* add it to the global thread list */
    THREAD_LOCK(state);
    list_add_head(&thread_list, &t->thread_list_node);
    THREAD_UNLOCK(state);

    return t;
}

thread_t *thread_create(CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, UINTN stack_size)
{
    return thread_create_etc(NULL, name, entry, arg, priority, NULL, stack_size);
}

/**
 * @brief Flag a thread as real time
 *
 * @param t Thread to flag
 *
 * @return EFI_SUCCESS on success
 */
EFI_STATUS thread_set_real_time(thread_t *t)
{
    if (!t)
        return EFI_INVALID_PARAMETER;

    ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);
    t->flags |= THREAD_FLAG_REAL_TIME;
    THREAD_UNLOCK(state);

    return EFI_SUCCESS;
}

BOOLEAN thread_is_realtime(thread_t *t)
{
    return (t->flags & THREAD_FLAG_REAL_TIME) && t->priority > DEFAULT_PRIORITY;
}

STATIC BOOLEAN thread_is_idle(thread_t *t)
{
    return !!(t->flags & THREAD_FLAG_IDLE);
}

STATIC BOOLEAN thread_is_real_time_or_idle(thread_t *t)
{
    return !!(t->flags & (THREAD_FLAG_REAL_TIME | THREAD_FLAG_IDLE));
}

/**
 * @brief  Make a suspended thread executable.
 *
 * This function is typically called to start a thread which has just been
 * created with thread_create()
 *
 * @param t  Thread to resume
 *
 * @return EFI_SUCCESS on success, ERR_NOT_SUSPENDED if thread was not suspended.
 */
EFI_STATUS thread_resume(thread_t *t)
{
    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state != THREAD_DEATH);

    BOOLEAN resched = FALSE;
    BOOLEAN ints_disabled = arch_ints_disabled();
    THREAD_LOCK(state);
    if (t->state == THREAD_SUSPENDED) {
        t->state = THREAD_READY;
        insert_in_run_queue_head(t);
        if (!ints_disabled) /* HACK, don't resced into bootstrap thread before idle thread is set up */
            resched = TRUE;
    }

    THREAD_UNLOCK(state);

    if (resched)
        thread_yield();

    return EFI_SUCCESS;
}

EFI_STATUS thread_detach_and_resume(thread_t *t)
{
    EFI_STATUS err;
    err = thread_detach(t);
    if (err != EFI_SUCCESS)
        return err;
    return thread_resume(t);
}

EFI_STATUS thread_join(thread_t *t, INTN *retcode, THREAD_TIME_MS timeout)
{
    ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);

    if (t->flags & THREAD_FLAG_DETACHED) {
        /* the thread is detached, go ahead and exit */
        THREAD_UNLOCK(state);
        return EFI_UNSUPPORTED;
    }

    /* wait for the thread to die */
    if (t->state != THREAD_DEATH) {
        EFI_STATUS err = wait_queue_block(&t->retcode_wait_queue, timeout);
        if (err != EFI_SUCCESS) {
            THREAD_UNLOCK(state);
            return err;
        }
    }

    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state == THREAD_DEATH);
    ASSERT(t->blocking_wait_queue == NULL);
    ASSERT(!list_in_list(&t->queue_node));

    /* save the return code */
    if (retcode)
        *retcode = t->retcode;

    /* remove it from the master thread list */
    list_delete(&t->thread_list_node);

    /* clear the structure's magic */
    t->magic = 0;

    THREAD_UNLOCK(state);

    /* free its stack and the thread structure itself */
    if (t->flags & THREAD_FLAG_FREE_STACK && t->stack)
        FreePool(t->stack);

    if (t->flags & THREAD_FLAG_FREE_STRUCT)
        FreePool(t);

    return EFI_SUCCESS;
}

EFI_STATUS thread_detach(thread_t *t)
{
    ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);

    /* if another thread is blocked inside thread_join() on this thread,
     * wake them up with a specific return code */
    wait_queue_wake_all(&t->retcode_wait_queue, FALSE, EFI_UNSUPPORTED);

    /* if it's already dead, then just do what join would have and exit */
    if (t->state == THREAD_DEATH) {
        t->flags &= ~THREAD_FLAG_DETACHED; /* makes sure thread_join continues */
        THREAD_UNLOCK(state);
        return thread_join(t, NULL, 0);
    } else {
        t->flags |= THREAD_FLAG_DETACHED;
        THREAD_UNLOCK(state);
        return EFI_SUCCESS;
    }
}

/**
 * @brief  Terminate the current thread
 *
 * Current thread exits with the specified return code.
 *
 * This function does not return.
 */
VOID thread_exit(INTN retcode)
{
    thread_t *current_thread = get_current_thread();

    ASSERT(current_thread->magic == THREAD_MAGIC);
    ASSERT(current_thread->state == THREAD_RUNNING);
    ASSERT(!thread_is_idle(current_thread));

//  DEBUG((EFI_D_INFO, "thread_exit: current %p\n", current_thread));

    THREAD_LOCK(state);

    /* enter the dead state */
    current_thread->state = THREAD_DEATH;
    current_thread->retcode = retcode;

    /* if we're detached, then do our teardown here */
    if (current_thread->flags & THREAD_FLAG_DETACHED) {
        /* remove it from the master thread list */
        list_delete(&current_thread->thread_list_node);

        /* clear the structure's magic */
        current_thread->magic = 0;

        /* free its stack and the thread structure itself */
        if (current_thread->flags & THREAD_FLAG_FREE_STACK && current_thread->stack) {
            dpc_queue(dpc_heap_free_cb, current_thread->stack, DPC_FLAG_NORESCHED);

            /* make sure its not going to get a bounds check performed on the half-freed stack */
            current_thread->flags &= ~THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK;
        }

        if (current_thread->flags & THREAD_FLAG_FREE_STRUCT)
            dpc_queue(dpc_heap_free_cb, current_thread, DPC_FLAG_NORESCHED);
    } else {
        /* signal if anyone is waiting */
        wait_queue_wake_all(&current_thread->retcode_wait_queue, FALSE, 0);
    }

    /* reschedule */
    thread_resched();

    DEBUG((EFI_D_ERROR, "somehow fell through thread_exit()\n"));
    ASSERT(FALSE);
    for(;;);
}

STATIC VOID idle_thread_routine(VOID)
{
    for (;;) {
        // this causes UEFI to dispatch events
        EfiGetCurrentTpl();

        thread_yield();
    }
}

STATIC thread_t *get_top_thread(INTN cpu)
{
    thread_t *newthread;
    UINT32 local_run_queue_bitmap = run_queue_bitmap;

    while (local_run_queue_bitmap) {
        /* find the first (remaining) queue with a thread in it */
        UINTN next_queue = sizeof(run_queue_bitmap) * 8 - 1 - __builtin_clz(local_run_queue_bitmap);

        list_for_every_entry(&run_queue[next_queue], newthread, thread_t, queue_node) {
#if WITH_SMP
            if (newthread->pinned_cpu < 0 || newthread->pinned_cpu == cpu)
#endif
            {
                list_delete(&newthread->queue_node);

                if (list_is_empty(&run_queue[next_queue]))
                    run_queue_bitmap &= ~(1<<next_queue);

                return newthread;
            }
        }

        local_run_queue_bitmap &= ~(1<<next_queue);
    }
    /* no threads to run, select the idle thread for this cpu */
    return idle_thread(cpu);
}

/**
 * @brief  Cause another thread to be executed.
 *
 * Internal reschedule routine. The current thread needs to already be in whatever
 * state and queues it needs to be in. This routine simply picks the next thread and
 * switches to it.
 *
 * This is probably not the function you're looking for. See
 * thread_yield() instead.
 */
VOID thread_resched(VOID)
{
    thread_t *oldthread;
    thread_t *newthread;

    thread_t *current_thread = get_current_thread();
    UINTN cpu = arch_curr_cpu_num();

    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));
    ASSERT(current_thread->state != THREAD_RUNNING);
    ASSERT(current_thread->SpinlockThreadData.OldTplIsValid);
    ASSERT(current_thread->SpinlockThreadData.OldTpl<=TPL_CALLBACK);

    newthread = get_top_thread(cpu);

    ASSERT(newthread);

    newthread->state = THREAD_RUNNING;

    oldthread = current_thread;

    if (newthread == oldthread)
        return;

    /* set up quantum for the new thread if it was consumed */
    if (newthread->remaining_quantum <= 0) {
        newthread->remaining_quantum = 5; // XXX make this smarter
    }

    /* mark the cpu ownership of the threads */
    thread_set_curr_cpu(oldthread, -1);
    thread_set_curr_cpu(newthread, cpu);

#if WITH_SMP
    if (thread_is_idle(newthread)) {
        mp_set_cpu_idle(cpu);
    } else {
        mp_set_cpu_busy(cpu);
    }

    if (thread_is_realtime(newthread)) {
        mp_set_cpu_realtime(cpu);
    } else {
        mp_set_cpu_non_realtime(cpu);
    }
#endif

    /* do the switch */
    set_current_thread(newthread);

#if DEBUG_THREAD_CONTEXT_SWITCH
    DEBUG((EFI_D_INFO, "arch_context_switch: cpu %d, old %p (%a, pri %d, flags 0x%x), new %p (%a, pri %d, flags 0x%x)\n",
            cpu, oldthread, oldthread->name, oldthread->priority,
            oldthread->flags, newthread, newthread->name,
            newthread->priority, newthread->flags));
#endif

#if THREAD_STACK_BOUNDS_CHECK
    /* check that the old thread has not blown its stack just before pushing its context */
    if (oldthread->flags & THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK) {
        ASSERT((THREAD_STACK_PADDING_SIZE % sizeof(UINT32)) == 0);
        UINT32 *s = (UINT32 *)oldthread->stack;
        for (UINTN i = 0; i < THREAD_STACK_PADDING_SIZE / sizeof(UINT32); i++) {
            if (unlikely(s[i] != STACK_DEBUG_WORD)) {
                /* NOTE: will probably blow the stack harder here, but hopefully enough
                 * state exists to at least get some sort of debugging done.
                 */
                DEBUG((EFI_D_ERROR, "stack overrun at %p: thread %p (%s), stack %p\n", &s[i],
                      oldthread, oldthread->name, oldthread->stack));
                ASSERT(FALSE);
                for(;;);
            }
        }
    }
#endif

    /* do the low level context switch */
    SwapContext(&oldthread->context, &newthread->context);
}

/**
 * @brief Yield the cpu to another thread
 *
 * This function places the current thread at the end of the run queue
 * and yields the cpu to another waiting thread (if any.)
 *
 * This function will return at some later time. Possibly immediately if
 * no other threads are waiting to execute.
 */
VOID thread_yield(VOID)
{
    thread_t *current_thread = get_current_thread();

    ASSERT(current_thread->magic == THREAD_MAGIC);
    ASSERT(current_thread->state == THREAD_RUNNING);

    THREAD_LOCK(state);

    /* we are yielding the cpu, so stick ourselves into the tail of the run queue and reschedule */
    current_thread->state = THREAD_READY;
    current_thread->remaining_quantum = 0;
    if (likely(!thread_is_idle(current_thread))) { /* idle thread doesn't go in the run queue */
        insert_in_run_queue_tail(current_thread);
    }
    thread_resched();

    THREAD_UNLOCK(state);
}

/**
 * @brief  Briefly yield cpu to another thread
 *
 * This function is similar to thread_yield(), except that it will
 * restart more quickly.
 *
 * This function places the current thread at the head of the run
 * queue and then yields the cpu to another thread.
 *
 * Exception:  If the time slice for this thread has expired, then
 * the thread goes to the end of the run queue.
 *
 * This function will return at some later time. Possibly immediately if
 * no other threads are waiting to execute.
 */
VOID thread_preempt(VOID)
{
    thread_t *current_thread = get_current_thread();

    ASSERT(current_thread->magic == THREAD_MAGIC);
    ASSERT(current_thread->state == THREAD_RUNNING);

    THREAD_LOCK(state);

    /* we are being preempted, so we get to go back into the front of the run queue if we have quantum left */
    current_thread->state = THREAD_READY;
    if (likely(!thread_is_idle(current_thread))) { /* idle thread doesn't go in the run queue */
        if (current_thread->remaining_quantum > 0)
            insert_in_run_queue_head(current_thread);
        else
            insert_in_run_queue_tail(current_thread); /* if we're out of quantum, go to the tail of the queue */
    }
    thread_resched();

    THREAD_UNLOCK(state);
}

/**
 * @brief  Suspend thread until woken.
 *
 * This function schedules another thread to execute.  This function does not
 * return until the thread is made runable again by some other module.
 *
 * You probably don't want to call this function directly; it's meant to be called
 * from other modules, such as mutex, which will presumably set the thread's
 * state to blocked and add it to some queue or another.
 */
VOID thread_block(VOID)
{
    __UNUSED thread_t *current_thread = get_current_thread();

    ASSERT(current_thread->magic == THREAD_MAGIC);
    ASSERT(current_thread->state == THREAD_BLOCKED);
    ASSERT(spin_lock_held(&thread_lock));
    ASSERT(!thread_is_idle(current_thread));

    /* we are blocking on something. the blocking code should have already stuck us on a queue */
    thread_resched();
}

VOID thread_unblock(thread_t *t, BOOLEAN resched)
{
    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state == THREAD_BLOCKED);
    ASSERT(spin_lock_held(&thread_lock));
    ASSERT(!thread_is_idle(t));

    t->state = THREAD_READY;
    insert_in_run_queue_head(t);
    if (resched)
        thread_resched();
}

enum handler_return thread_timer_tick(VOID)
{
    thread_t *current_thread = get_current_thread();

    if (thread_is_real_time_or_idle(current_thread))
        return INT_NO_RESCHEDULE;

    current_thread->remaining_quantum--;
    if (current_thread->remaining_quantum <= 0) {
        return INT_RESCHEDULE;
    } else {
        return INT_NO_RESCHEDULE;
    }
}

/* timer callback to wake up a sleeping thread */
STATIC VOID EFIAPI thread_sleep_handler (
    IN  EFI_EVENT   Event,
    IN  VOID        *Context
)
{
    TIMER_CONTEXT *TimerContext = Context;
    thread_t *t = TimerContext->thread;

    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(t->state == THREAD_SLEEPING);

    THREAD_LOCK(state);

    t->state = THREAD_READY;
    insert_in_run_queue_head(t);

    THREAD_UNLOCK(state);

    gBS->CloseEvent(TimerContext->Event);
    FreePool(TimerContext);
}

/**
 * @brief  Put thread to sleep; delay specified in ms
 *
 * This function puts the current thread to sleep until the specified
 * delay in ms has expired.
 *
 * Note that this function could sleep for longer than the specified delay if
 * other threads are running.  When the timer expires, this thread will
 * be placed at the head of the run queue.
 */
VOID thread_sleep(THREAD_TIME_MS delay)
{
    TIMER_CONTEXT *TimerContext = NULL;
    EFI_STATUS Status;

    thread_t *current_thread = get_current_thread();

    ASSERT(current_thread->magic == THREAD_MAGIC);
    ASSERT(current_thread->state == THREAD_RUNNING);
    ASSERT(!thread_is_idle(current_thread));

    TimerContext = AllocatePool(sizeof(TIMER_CONTEXT));
    ASSERT(TimerContext);
    TimerContext->thread = current_thread;
    Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK, thread_sleep_handler, TimerContext, &TimerContext->Event);
    ASSERT_EFI_ERROR (Status);

    THREAD_LOCK(state);
    Status = gBS->SetTimer (TimerContext->Event, TimerRelative, MS2100N(delay));
    ASSERT_EFI_ERROR (Status);
    current_thread->state = THREAD_SLEEPING;
    thread_resched();
    THREAD_UNLOCK(state);
}

/**
 * @brief  Initialize threading system
 *
 * This function is called once, from kmain()
 */
VOID thread_init_early(VOID)
{
    INTN i;

    ASSERT(arch_curr_cpu_num() == 0);

    /* make sure the bitmap is large enough to cover our number of priorities */
    ASSERT(NUM_PRIORITIES <= sizeof(run_queue_bitmap) * 8);

    /* initialize the run queues */
    for (i=0; i < NUM_PRIORITIES; i++)
        list_initialize(&run_queue[i]);

    /* initialize the thread list */
    list_initialize(&thread_list);

    /* create a thread to cover the current running state */
    thread_t *t = AllocatePool(sizeof(thread_t));
    init_thread_struct(t, "bootstrap");

    /* half construct this thread, since we're already running */
    t->priority = HIGHEST_PRIORITY;
    t->state = THREAD_RUNNING;
    t->flags = THREAD_FLAG_DETACHED;
    thread_set_curr_cpu(t, 0);
    thread_set_pinned_cpu(t, 0);
    wait_queue_init(&t->retcode_wait_queue);
    list_add_head(&thread_list, &t->thread_list_node);
    set_current_thread(t);

    list_initialize(&t->tls_list);
}

/**
 * @brief Complete thread initialization
 *
 * This function is called once at boot time
 */
VOID thread_init(VOID)
{
    dpc_init();
}

/**
 * @brief Change name of current thread
 */
VOID thread_set_name(CONST CHAR8 *name)
{
    thread_t *current_thread = get_current_thread();
    AsciiStrCpyS(current_thread->name, sizeof(current_thread->name), name);
}

/**
 * @brief Change priority of current thread
 *
 * See thread_create() for a discussion of priority values.
 */
VOID thread_set_priority(INTN priority)
{
    thread_t *current_thread = get_current_thread();

    THREAD_LOCK(state);

    if (priority <= IDLE_PRIORITY)
        priority = IDLE_PRIORITY + 1;
    if (priority > HIGHEST_PRIORITY)
        priority = HIGHEST_PRIORITY;
    current_thread->priority = priority;

    current_thread->state = THREAD_READY;
    insert_in_run_queue_head(current_thread);
    thread_resched();

    THREAD_UNLOCK(state);
}

STATIC INTN idle_thread_entry(VOID *arg)
{
    idle_thread_routine();
    return 0;
}

VOID thread_create_idle(VOID) {
    /* create a thread to cover the current running state */
    thread_t *t = idle_thread(0);
    thread_create_etc(t, "idle", idle_thread_entry, NULL, IDLE_PRIORITY, NULL, DEFAULT_STACK_SIZE);
    thread_set_pinned_cpu(t, arch_curr_cpu_num());
    thread_detach(t);
}

STATIC CONST CHAR8 *thread_state_to_str(enum thread_state state)
{
    switch (state) {
        case THREAD_SUSPENDED:
            return "susp";
        case THREAD_READY:
            return "rdy";
        case THREAD_RUNNING:
            return "run";
        case THREAD_BLOCKED:
            return "blok";
        case THREAD_SLEEPING:
            return "slep";
        case THREAD_DEATH:
            return "deth";
        default:
            return "unkn";
    }
}

UINTN thread_stack_used(thread_t *t) {
#ifdef THREAD_STACK_HIGHWATER
    UINT8 *stack_base;
    UINTN stack_size;
    UINTN i;

    stack_base = t->stack;
    stack_size = t->stack_size;

    for (i = 0; i < stack_size; i++) {
        if (stack_base[i] != STACK_DEBUG_BYTE)
            break;
    }
    return stack_size - i;
#else
    return 0;
#endif
}
/**
 * @brief  Dump debugging info about the specified thread.
 */
VOID dump_thread(thread_t *t)
{
    (VOID)(thread_state_to_str);
    DEBUG((EFI_D_INFO, "dump_thread: t %p (%a)\n", t, t->name));
#if WITH_SMP
    DEBUG((EFI_D_INFO, "\tstate %a, curr_cpu %d, pinned_cpu %d, priority %d, remaining quantum %d\n",
            thread_state_to_str(t->state), t->curr_cpu, t->pinned_cpu, t->priority, t->remaining_quantum));
#else
    DEBUG((EFI_D_INFO, "\tstate %a, priority %d, remaining quantum %d\n",
            thread_state_to_str(t->state), t->priority, t->remaining_quantum));
#endif
#ifdef THREAD_STACK_HIGHWATER
    DEBUG((EFI_D_INFO, "\tstack %p, stack_size %d, stack_used %d\n",
            t->stack, t->stack_size, thread_stack_used(t)));
#else
    DEBUG((EFI_D_INFO, "\tstack %p, stack_size %d\n", t->stack, t->stack_size));
#endif
    DEBUG((EFI_D_INFO, "\tentry %p, arg %p, flags 0x%x\n", t->entry, t->arg, t->flags));
    DEBUG((EFI_D_INFO, "\twait queue %p, wait queue ret %d\n", t->blocking_wait_queue, t->wait_queue_block_ret));
    DEBUG((EFI_D_INFO, "\ttls:"));
    tls_entry_t *tls_entry;
    list_for_every_entry(&t->tls_list, tls_entry, tls_entry_t, node) {
        DEBUG((EFI_D_INFO, "%u: 0x%x", tls_entry->key, tls_entry->data));
    }
    DEBUG((EFI_D_INFO, "\n"));
}

/**
 * @brief  Dump debugging info about all threads
 */
VOID dump_all_threads(VOID)
{
    thread_t *t;

    THREAD_LOCK(state);
    list_for_every_entry(&thread_list, t, thread_t, thread_list_node) {
        if (t->magic != THREAD_MAGIC) {
            DEBUG((EFI_D_INFO, "bad magic on thread struct %p, aborting.\n", t));
            break;
        }
        dump_thread(t);
    }
    THREAD_UNLOCK(state);
}

/** @} */


/**
 * @defgroup  wait  Wait Queue
 * @{
 */
VOID wait_queue_init(wait_queue_t *wait)
{
    *wait = (wait_queue_t)WAIT_QUEUE_INITIAL_VALUE(*wait);
}

STATIC VOID EFIAPI wait_queue_timeout_handler (
    IN  EFI_EVENT   Event,
    IN  VOID        *Context
)
{
    TIMER_CONTEXT *TimerContext = Context;
    thread_t *thread = TimerContext->thread;

    ASSERT(thread->magic == THREAD_MAGIC);

    spin_lock(&thread_lock);
    thread_unblock_from_wait_queue(thread, EFI_TIMEOUT);
    spin_unlock(&thread_lock);
}

/**
 * @brief  Block until a wait queue is notified.
 *
 * This function puts the current thread at the end of a wait
 * queue and then blocks until some other thread wakes the queue
 * up again.
 *
 * @param  wait     The wait queue to enter
 * @param  timeout  The maximum time, in ms, to wait
 *
 * If the timeout is zero, this function returns immediately with
 * EFI_TIMEOUT.  If the timeout is INFINITE_TIME, this function
 * waits indefinitely.  Otherwise, this function returns with
 * EFI_TIMEOUT at the end of the timeout period.
 *
 * @return EFI_TIMEOUT on timeout, else returns the return
 * value specified when the queue was woken by wait_queue_wake_one().
 */
EFI_STATUS wait_queue_block(wait_queue_t *wait, THREAD_TIME_MS timeout)
{
    TIMER_CONTEXT *TimerContext = NULL;
    EFI_STATUS Status;

    thread_t *current_thread = get_current_thread();

    ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    ASSERT(current_thread->state == THREAD_RUNNING);
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    if (timeout == 0)
        return EFI_TIMEOUT;

    list_add_tail(&wait->list, &current_thread->queue_node);
    wait->count++;
    current_thread->state = THREAD_BLOCKED;
    current_thread->blocking_wait_queue = wait;
    current_thread->wait_queue_block_ret = EFI_SUCCESS;

    /* if the timeout is nonzero or noninfinite, set a callback to yank us out of the queue */
    if (timeout != INFINITE_TIME) {
        TimerContext = AllocatePool(sizeof(TIMER_CONTEXT));
        ASSERT(TimerContext);
        TimerContext->thread = current_thread;

        Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_NOTIFY, wait_queue_timeout_handler, TimerContext, &TimerContext->Event);
        ASSERT_EFI_ERROR (Status);

        Status = gBS->SetTimer (TimerContext->Event, TimerRelative, MS2100N(timeout));
        ASSERT_EFI_ERROR (Status);
    }

    thread_resched();

    /* we don't really know if the timer fired or not, so it's better safe to try to cancel it */
    if (timeout != INFINITE_TIME) {
        Status = gBS->SetTimer (TimerContext->Event, TimerCancel, 0);
        ASSERT_EFI_ERROR (Status);

        Status = gBS->CloseEvent(TimerContext->Event);
        ASSERT_EFI_ERROR (Status);

        FreePool(TimerContext);
    }

    return current_thread->wait_queue_block_ret;
}

/**
 * @brief  Wake up one thread sleeping on a wait queue
 *
 * This function removes one thread (if any) from the head of the wait queue and
 * makes it executable.  The new thread will be placed at the head of the
 * run queue.
 *
 * @param wait  The wait queue to wake
 * @param reschedule  If true, the newly-woken thread will run immediately.
 * @param wait_queue_error  The return value which the new thread will receive
 * from wait_queue_block().
 *
 * @return  The number of threads woken (zero or one)
 */
INTN wait_queue_wake_one(wait_queue_t *wait, BOOLEAN reschedule, EFI_STATUS wait_queue_error)
{
    thread_t *t;
    INTN ret = 0;

    thread_t *current_thread = get_current_thread();

    ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    t = list_remove_head_type(&wait->list, thread_t, queue_node);
    if (t) {
        wait->count--;
        ASSERT(t->state == THREAD_BLOCKED);
        t->state = THREAD_READY;
        t->wait_queue_block_ret = wait_queue_error;
        t->blocking_wait_queue = NULL;

        /* if we're instructed to reschedule, stick the current thread on the head
         * of the run queue first, so that the newly awakened thread gets a chance to run
         * before the current one, but the current one doesn't get unnecessarilly punished.
         */
        if (reschedule) {
            current_thread->state = THREAD_READY;
            insert_in_run_queue_head(current_thread);
        }
        insert_in_run_queue_head(t);
        if (reschedule) {
            thread_resched();
        }
        ret = 1;

    }

    return ret;
}


/**
 * @brief  Wake all threads sleeping on a wait queue
 *
 * This function removes all threads (if any) from the wait queue and
 * makes them executable.  The new threads will be placed at the head of the
 * run queue.
 *
 * @param wait  The wait queue to wake
 * @param reschedule  If true, the newly-woken threads will run immediately.
 * @param wait_queue_error  The return value which the new thread will receive
 * from wait_queue_block().
 *
 * @return  The number of threads woken (zero or one)
 */
INTN wait_queue_wake_all(wait_queue_t *wait, BOOLEAN reschedule, EFI_STATUS wait_queue_error)
{
    thread_t *t;
    INTN ret = 0;

    thread_t *current_thread = get_current_thread();

    ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    if (reschedule && wait->count > 0) {
        /* if we're instructed to reschedule, stick the current thread on the head
         * of the run queue first, so that the newly awakened threads get a chance to run
         * before the current one, but the current one doesn't get unnecessarilly punished.
         */
        current_thread->state = THREAD_READY;
        insert_in_run_queue_head(current_thread);
    }

    /* pop all the threads off the wait queue into the run queue */
    while ((t = list_remove_head_type(&wait->list, thread_t, queue_node))) {
        wait->count--;
        ASSERT(t->state == THREAD_BLOCKED);
        t->state = THREAD_READY;
        t->wait_queue_block_ret = wait_queue_error;
        t->blocking_wait_queue = NULL;

        insert_in_run_queue_head(t);
        ret++;
    }

    ASSERT(wait->count == 0);

    if (ret > 0) {
        if (reschedule) {
            thread_resched();
        }
    }

    return ret;
}

/**
 * @brief  Free all resources allocated in wait_queue_init()
 *
 * If any threads were waiting on this queue, they are all woken.
 */
VOID wait_queue_destroy(wait_queue_t *wait, BOOLEAN reschedule)
{
    ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    wait_queue_wake_all(wait, reschedule, EFI_ABORTED);
    wait->magic = 0;
}

/**
 * @brief  Wake a specific thread in a wait queue
 *
 * This function extracts a specific thread from a wait queue, wakes it, and
 * puts it at the head of the run queue.
 *
 * @param t  The thread to wake
 * @param wait_queue_error  The return value which the new thread will receive
 *   from wait_queue_block().
 *
 * @return EFI_UNSUPPORTED if thread was not in any wait queue.
 */
EFI_STATUS thread_unblock_from_wait_queue(thread_t *t, EFI_STATUS wait_queue_error)
{
    ASSERT(t->magic == THREAD_MAGIC);
    ASSERT(arch_ints_disabled());
    ASSERT(spin_lock_held(&thread_lock));

    if (t->state != THREAD_BLOCKED)
        return EFI_UNSUPPORTED;

    ASSERT(t->blocking_wait_queue != NULL);
    ASSERT(t->blocking_wait_queue->magic == WAIT_QUEUE_MAGIC);
    ASSERT(list_in_list(&t->queue_node));

    list_delete(&t->queue_node);
    t->blocking_wait_queue->count--;
    t->blocking_wait_queue = NULL;
    t->state = THREAD_READY;
    t->wait_queue_block_ret = wait_queue_error;
    insert_in_run_queue_head(t);

    return EFI_SUCCESS;
}

SPINLOCK_THREAD_DATA* get_current_spinlock_thread_data(VOID) {
    return &(get_current_thread()->SpinlockThreadData);
}
