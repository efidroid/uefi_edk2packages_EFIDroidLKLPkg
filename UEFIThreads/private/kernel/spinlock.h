/*
 * Copyright (c) 2014 Travis Geiselbrecht
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
#ifndef __KERNEL_SPINLOCK_H
#define __KERNEL_SPINLOCK_H

#include <Library/DebugLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/UefiThread.h>

#define SPIN_LOCK_INITIAL_VALUE 0
#define SPIN_LOCK_FLAG_INTERRUPTS 0x00000001

typedef UINTN SPINLOCK;
typedef UINTN SPINLOCK_STATE;
typedef UINTN SPINLOCK_FLAGS;
typedef struct {
  EFI_TPL OldTpl;
  BOOLEAN OldTplIsValid;
} SPINLOCK_THREAD_DATA;

SPINLOCK_THREAD_DATA* get_current_spinlock_thread_data(VOID);

/* interrupts should already be disabled */
STATIC inline VOID spin_lock(SPINLOCK *lock)
{
    ASSERT(*lock == 0);
    *lock = 1;
}

/* Returns 0 on success, non-0 on failure */
STATIC inline INTN spin_trylock(SPINLOCK *lock)
{
    return 0;
}

/* interrupts should already be disabled */
STATIC inline VOID spin_unlock(SPINLOCK *lock)
{
    ASSERT(*lock == 1);
    *lock = 0;
}

STATIC inline VOID spin_lock_init(SPINLOCK *lock)
{
    *lock = SPIN_LOCK_INITIAL_VALUE;
}

STATIC inline BOOLEAN spin_lock_held(SPINLOCK *lock)
{
    return *lock != 0;
}

STATIC inline VOID arch_enable_ints(VOID)
{
    SPINLOCK_THREAD_DATA *t = get_current_spinlock_thread_data();

    ASSERT(t->OldTplIsValid==TRUE);
    t->OldTplIsValid = FALSE;
    gBS->RestoreTPL(t->OldTpl);
}

STATIC inline VOID arch_disable_ints(VOID)
{
    SPINLOCK_THREAD_DATA *t = get_current_spinlock_thread_data();

    ASSERT(t->OldTplIsValid==FALSE);
    t->OldTpl = gBS->RaiseTPL(TPL_NOTIFY);
    t->OldTplIsValid = TRUE;
}

STATIC inline BOOLEAN arch_ints_disabled(VOID)
{
    return EfiGetCurrentTpl()>=TPL_NOTIFY;
}

STATIC inline UINTN arch_curr_cpu_num(VOID) {
    return 0;
}

/* spin lock irq save flags: */

/* Possible future flags:
 * SPIN_LOCK_FLAG_PMR_MASK         = 0x000000ff
 * SPIN_LOCK_FLAG_PREEMPTION       = 0x00000100
 * SPIN_LOCK_FLAG_SET_PMR          = 0x00000200
 */

enum {
    /* private */
    SPIN_LOCK_STATE_RESTORE_IRQ = 1,
};

/* same as spin lock, but save disable and save interrupt state first */
STATIC inline VOID spin_lock_save(
    SPINLOCK *lock,
    SPINLOCK_STATE *statep,
    SPINLOCK_FLAGS flags)
{
    SPINLOCK_STATE state = 0;
    if ((flags & SPIN_LOCK_FLAG_INTERRUPTS) && !arch_ints_disabled()) {
        state |= SPIN_LOCK_STATE_RESTORE_IRQ;
        arch_disable_ints();
    }
    *statep = state;

    spin_lock(lock);
}

/* restore interrupt state before unlocking */
STATIC inline VOID spin_unlock_restore(
    SPINLOCK *lock,
    SPINLOCK_STATE old_state,
    SPINLOCK_FLAGS flags)
{
    spin_unlock(lock);

    if ((flags & SPIN_LOCK_FLAG_INTERRUPTS) && (old_state & SPIN_LOCK_STATE_RESTORE_IRQ))
        arch_enable_ints();
}

/* hand(ier) routines */
#define spin_lock_irqsave(lock, state) spin_lock_save(lock, &(state), SPIN_LOCK_FLAG_INTERRUPTS)
#define spin_unlock_irqrestore(lock, state) spin_unlock_restore(lock, state, SPIN_LOCK_FLAG_INTERRUPTS)

#endif
