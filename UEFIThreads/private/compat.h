#ifndef _COMPAT_H
#define _COMPAT_H

enum handler_return {
    INT_NO_RESCHEDULE = 0,
    INT_RESCHEDULE,
};

#define __ALWAYS_INLINE __attribute__((always_inline))
#define __UNUSED __attribute__((__unused__))
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#define MS2100N(x) ((x)*(1000000/100))

#endif
