#ifndef _LKL_PRIVATE_H
#define _LKL_PRIVATE_H

#include <PiDxe.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Protocol/UefiThread.h>
#include <Protocol/LKL.h>

#include <lkl.h>
#include <lkl_host.h>
#include <iomem.h>

extern UEFI_THREAD_PROTOCOL            *gThreads;

#endif
