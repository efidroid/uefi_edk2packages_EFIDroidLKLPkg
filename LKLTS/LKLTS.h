#ifndef _LKLTS_H_
#define _LKLTS_H_

#include <Uefi.h>

#include <Protocol/AbsolutePointer.h>
#include <Protocol/UefiThread.h>

#include <Library/DebugLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseLib.h>
#include <Library/DevicePathLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>

#include <lkl.h>
#include <lkl_host.h>
#include <lkl/asm/syscalls.h>
#include <lkl/linux/input.h>
#include <stdint.h>

#include <tslib.h>
#include <tslib-private.h>

//
// Driver Private Data
//
#define TSLIB_ABSOLUTE_POINTER_DEV_SIGNATURE SIGNATURE_32 ('t', 's', 'l', 't')

typedef struct {
  UINTN                               Signature;

  EFI_ABSOLUTE_POINTER_PROTOCOL       AbsolutePointerProtocol;
  EFI_ABSOLUTE_POINTER_STATE          State;
  EFI_ABSOLUTE_POINTER_MODE           Mode;
  BOOLEAN                             StateChanged;

  EFI_DEVICE_PATH_PROTOCOL            *DevicePath;
  struct tsdev                        *ts;
} TSLIB_ABSOLUTE_POINTER_DEV;

#define  TSLIB_ABSOLUTE_POINTER_DEV_FROM_THIS(a)  CR (a, TSLIB_ABSOLUTE_POINTER_DEV, AbsolutePointerProtocol, TSLIB_ABSOLUTE_POINTER_DEV_SIGNATURE)

#endif
