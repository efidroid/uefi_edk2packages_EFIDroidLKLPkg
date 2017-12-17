#include "LKLTS.h"

#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

STATIC inline long get_syscall_ret(long rc) {
    if(rc<0) {
        errno = -rc;
        return -1;
    }

    return rc;
}

ssize_t read(int fd, void *buf, size_t count) {
    return (ssize_t)get_syscall_ret(lkl_sys_read(fd, (void*)buf, count));
}

int open(const char *pathname, int flags/*, lkl_mode_t mode*/) {
    return (int)get_syscall_ret(lkl_sys_open(pathname, flags, 0/*, mode*/));
}

int close(int fd) {
    return (int)get_syscall_ret(lkl_sys_close(fd));
}

int ioctl(int fd, int request, void* arg) {
    return get_syscall_ret(lkl_sys_ioctl(fd, request, (unsigned long)arg));
}
