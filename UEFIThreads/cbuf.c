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
#include <Base.h>
#include <PiDxe.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include "private/lib/cbuf.h"
#include "private/kernel/event.h"
#include "private/kernel/spinlock.h"
#include "private/pow2.h"

#define INC_POINTER(cbuf, ptr, inc) \
    modpow2(((ptr) + (inc)), (cbuf)->len_pow2)

VOID cbuf_initialize(cbuf_t *cbuf, UINTN len)
{
    cbuf_initialize_etc(cbuf, len, AllocatePool(len));
}

VOID cbuf_initialize_etc(cbuf_t *cbuf, UINTN len, VOID *buf)
{
    ASSERT(cbuf);
    ASSERT(len > 0);
    ASSERT(ispow2(len));

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->len_pow2 = log2_uint(len);
    cbuf->buf = buf;
    event_init(&cbuf->event, FALSE, 0);
    spin_lock_init(&cbuf->lock);

    //LTRACEF("len %zd, len_pow2 %u\n", len, cbuf->len_pow2);
}

UINTN cbuf_space_avail(cbuf_t *cbuf)
{
    UINTN consumed = modpow2((UINTN)(cbuf->head - cbuf->tail), cbuf->len_pow2);
    return valpow2(cbuf->len_pow2) - consumed - 1;
}

UINTN cbuf_space_used(cbuf_t *cbuf)
{
    return modpow2((UINTN)(cbuf->head - cbuf->tail), cbuf->len_pow2);
}

UINTN cbuf_write(cbuf_t *cbuf, CONST VOID *_buf, UINTN len, BOOLEAN canreschedule)
{
    CONST CHAR8 *buf = (CONST CHAR8 *)_buf;

    //LTRACEF("len %zd\n", len);

    ASSERT(cbuf);
    ASSERT(len < valpow2(cbuf->len_pow2));

    SPINLOCK_STATE state;
    spin_lock_irqsave(&cbuf->lock, state);

    UINTN write_len;
    UINTN pos = 0;

    while (pos < len && cbuf_space_avail(cbuf) > 0) {
        if (cbuf->head >= cbuf->tail) {
            if (cbuf->tail == 0) {
                // Special case - if tail is at position 0, we can't write all
                // the way to the end of the buffer. Otherwise, head ends up at
                // 0, head == tail, and buffer is considered "empty" again.
                write_len =
                    MIN(valpow2(cbuf->len_pow2) - cbuf->head - 1, len - pos);
            } else {
                // Write to the end of the buffer.
                write_len =
                    MIN(valpow2(cbuf->len_pow2) - cbuf->head, len - pos);
            }
        } else {
            // Write from head to tail-1.
            write_len = MIN(cbuf->tail - cbuf->head - 1, len - pos);
        }

        // if it's full, abort and return how much we've written
        if (write_len == 0) {
            break;
        }

        if (NULL == buf) {
            SetMem(cbuf->buf + cbuf->head, write_len, 0);
        } else {
            CopyMem(cbuf->buf + cbuf->head, buf + pos, write_len);
        }

        cbuf->head = INC_POINTER(cbuf, cbuf->head, write_len);
        pos += write_len;
    }

    if (cbuf->head != cbuf->tail)
        event_signal(&cbuf->event, FALSE);

    spin_unlock_irqrestore(&cbuf->lock, state);

    // XXX convert to only rescheduling if
    if (canreschedule)
        thread_preempt();

    return pos;
}

UINTN cbuf_read(cbuf_t *cbuf, VOID *_buf, UINTN buflen, BOOLEAN block)
{
    CHAR8 *buf = (CHAR8 *)_buf;

    ASSERT(cbuf);

retry:
    // block on the cbuf outside of the lock, which may
    // unblock us early and we'll have to double check below
    if (block)
        event_wait(&cbuf->event);

    SPINLOCK_STATE state;
    spin_lock_irqsave(&cbuf->lock, state);

    // see if there's data available
    UINTN ret = 0;
    if (cbuf->tail != cbuf->head) {
        UINTN pos = 0;

        // loop until we've read everything we need
        // at most this will make two passes to deal with wraparound
        while (pos < buflen && cbuf->tail != cbuf->head) {
            UINTN read_len;
            if (cbuf->head > cbuf->tail) {
                // simple case where there is no wraparound
                read_len = MIN(cbuf->head - cbuf->tail, buflen - pos);
            } else {
                // read to the end of buffer in this pass
                read_len = MIN(valpow2(cbuf->len_pow2) - cbuf->tail, buflen - pos);
            }

            // Only perform the copy if a buf was supplied
            if (NULL != buf) {
                CopyMem(buf + pos, cbuf->buf + cbuf->tail, read_len);
            }

            cbuf->tail = INC_POINTER(cbuf, cbuf->tail, read_len);
            pos += read_len;
        }

        if (cbuf->tail == cbuf->head) {
            ASSERT(pos > 0);
            // we've emptied the buffer, unsignal the event
            event_unsignal(&cbuf->event);
        }

        ret = pos;
    }

    spin_unlock_irqrestore(&cbuf->lock, state);

    // we apparently blocked but raced with another thread and found no data, retry
    if (block && ret == 0)
        goto retry;

    return ret;
}

UINTN cbuf_peek(cbuf_t *cbuf, CBUF_IOVEC *regions, BOOLEAN block, THREAD_TIME_MS timeout)
{
    EFI_STATUS status;

    ASSERT(cbuf && regions);

    if (block) {
        status = event_wait_timeout(&cbuf->event, timeout);
        if (status==EFI_TIMEOUT) {
            DEBUG((EFI_D_ERROR, "%a: timeout of %lu ms epired\n", __func__, (UINT64)timeout));
            return 0;
        }
    }

    SPINLOCK_STATE state;
    spin_lock_irqsave(&cbuf->lock, state);

    UINTN ret = cbuf_space_used(cbuf);
    UINTN sz  = cbuf_size(cbuf);

    ASSERT(cbuf->tail < sz);
    ASSERT(ret <= sz);

    regions[0].Base = ret ? (cbuf->buf + cbuf->tail) : NULL;
    if (ret + cbuf->tail > sz) {
        regions[0].Length  = sz - cbuf->tail;
        regions[1].Base = cbuf->buf;
        regions[1].Length  = ret - regions[0].Length;
    } else {
        regions[0].Length  = ret;
        regions[1].Base = NULL;
        regions[1].Length  = 0;
    }

    spin_unlock_irqrestore(&cbuf->lock, state);
    return ret;
}

UINTN cbuf_write_char(cbuf_t *cbuf, CHAR8 c, BOOLEAN canreschedule)
{
    ASSERT(cbuf);

    SPINLOCK_STATE state;
    spin_lock_irqsave(&cbuf->lock, state);

    UINTN ret = 0;
    if (cbuf_space_avail(cbuf) > 0) {
        cbuf->buf[cbuf->head] = c;

        cbuf->head = INC_POINTER(cbuf, cbuf->head, 1);
        ret = 1;

        if (cbuf->head != cbuf->tail)
            event_signal(&cbuf->event, canreschedule);
    }

    spin_unlock_irqrestore(&cbuf->lock, state);

    return ret;
}

UINTN cbuf_read_char(cbuf_t *cbuf, CHAR8 *c, BOOLEAN block)
{
    ASSERT(cbuf);
    ASSERT(c);

retry:
    if (block)
        event_wait(&cbuf->event);

    SPINLOCK_STATE state;
    spin_lock_irqsave(&cbuf->lock, state);

    // see if there's data available
    UINTN ret = 0;
    if (cbuf->tail != cbuf->head) {

        *c = cbuf->buf[cbuf->tail];
        cbuf->tail = INC_POINTER(cbuf, cbuf->tail, 1);

        if (cbuf->tail == cbuf->head) {
            // we've emptied the buffer, unsignal the event
            event_unsignal(&cbuf->event);
        }

        ret = 1;
    }

    spin_unlock_irqrestore(&cbuf->lock, state);

    if (block && ret == 0)
        goto retry;

    return ret;
}
