#ifndef _CONFIG_H
#define _CONFIG_H

#define PACKAGE_VERSION "1.14"

#define TSLIB_STATIC_INPUT_MODULE 1
#define HAVE_UNISTD_H 1

#define stderr EFI_D_ERROR
#define stdout EFI_D_ERROR

#define fprintf(level, fmt, ...) printf(fmt, ##__VA_ARGS__)
#define vfprintf(level, fmt, ap) vprintf(fmt, ap)

#endif
