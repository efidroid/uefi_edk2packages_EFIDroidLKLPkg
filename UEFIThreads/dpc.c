/*
 * Copyright (c) 2008 Travis Geiselbrecht
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
#include <Library/MemoryAllocationLib.h>
#include "private/list.h"
#include "private/kernel/dpc.h"
#include "private/kernel/thread.h"
#include "private/kernel/event.h"
#include "private/kernel/spinlock.h"

struct dpc {
    struct list_node node;

    dpc_callback cb;
    VOID *arg;
};

STATIC struct list_node dpc_list = LIST_INITIAL_VALUE(dpc_list);
SPINLOCK dpc_lock = SPIN_LOCK_INITIAL_VALUE;
STATIC event_t dpc_event;

STATIC INTN dpc_thread_routine(VOID *arg);

EFI_STATUS dpc_queue(dpc_callback cb, VOID *arg, UINTN flags)
{
    struct dpc *dpc;

    dpc = AllocatePool(sizeof(struct dpc));

    if (dpc == NULL)
        return EFI_OUT_OF_RESOURCES;

    dpc->cb = cb;
    dpc->arg = arg;

    SPINLOCK_STATE state;
    spin_lock_irqsave(&dpc_lock, state);

    list_add_tail(&dpc_list, &dpc->node);

    spin_unlock_irqrestore(&dpc_lock, state);

    event_signal(&dpc_event, (flags & DPC_FLAG_NORESCHED) ? FALSE : TRUE);

    return EFI_SUCCESS;
}

STATIC INTN dpc_thread_routine(VOID *arg)
{
    for (;;) {
        event_wait(&dpc_event);

        SPINLOCK_STATE state;
        spin_lock_irqsave(&dpc_lock, state);

        struct dpc *dpc = list_remove_head_type(&dpc_list, struct dpc, node);
        if (!dpc)
            event_unsignal(&dpc_event);

        spin_unlock_irqrestore(&dpc_lock, state);

        if (dpc) {
            dpc->cb(dpc->arg);

            FreePool(dpc);
        }
    }

    return 0;
}

VOID dpc_init(VOID)
{
    event_init(&dpc_event, FALSE, 0);

    thread_detach_and_resume(thread_create("dpc", &dpc_thread_routine, NULL, DPC_PRIORITY, DEFAULT_STACK_SIZE));
}


