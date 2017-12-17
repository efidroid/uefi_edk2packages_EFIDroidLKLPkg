#ifndef _LKLTS_FCNTL_H
#define _LKLTS_FCNTL_H

#include <lkl/linux/fcntl.h>

#define O_RDONLY LKL_O_RDONLY
#define O_RDWR LKL_O_RDWR
#define O_NONBLOCK LKL_O_NONBLOCK

int open(const char *pathname, int flags);

#endif
