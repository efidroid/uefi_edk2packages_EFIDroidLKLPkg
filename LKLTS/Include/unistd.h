#ifndef _LKLTS_UNISTD_H
#define _LKLTS_UNISTD_H

#include_next <unistd.h>

#include <stdint.h>

ssize_t read(int fd, void *buf, size_t count);
int close(int fd);

#endif
