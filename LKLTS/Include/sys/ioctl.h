#ifndef _SYS_IOCTL_H
#define _SYS_IOCTL_H

#include <lkl/linux/ioctl.h>

int ioctl(int fd, int request, void* arg);

#endif
