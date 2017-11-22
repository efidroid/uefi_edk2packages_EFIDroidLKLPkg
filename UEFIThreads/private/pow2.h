/*
 * Copyright (c) 2008-2014 Travis Geiselbrecht
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
#ifndef __POW2_H
#define __POW2_H

#include "private/compat.h"

/* routines for dealing with power of 2 values for efficiency */
STATIC inline __ALWAYS_INLINE BOOLEAN ispow2(UINTN val)
{
    return ((val - 1) & val) == 0;
}

STATIC inline __ALWAYS_INLINE UINTN log2_uint(UINTN val)
{
    if (val == 0)
        return 0; // undefined

    return (sizeof(val) * 8) - 1 - __builtin_clz(val);
}

STATIC inline __ALWAYS_INLINE UINTN valpow2(UINTN valp2)
{
    return 1U << valp2;
}

STATIC inline __ALWAYS_INLINE UINTN divpow2(UINTN val, UINTN divp2)
{
    return val >> divp2;
}

STATIC inline __ALWAYS_INLINE UINTN modpow2(UINTN val, UINTN modp2)
{
    return val & ((1UL << modp2) - 1);
}

// Cribbed from:
// http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
STATIC inline __ALWAYS_INLINE UINT32 round_up_pow2_u32(UINT32 v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

#endif

