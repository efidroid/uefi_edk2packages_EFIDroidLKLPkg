#ifndef __UCONTEXT_H__
#define __UCONTEXT_H__

#if defined (MDE_CPU_ARM)
#include "Arm/UContext.h"
#endif

typedef VOID (UCONTEXT_START_ROUTINE) (VOID);

INTN
GetContext (
  OUT     UCONTEXT *UContext
  );

INTN
SetContext (
  IN      CONST UCONTEXT *UContext
  );

VOID
MakeContext (
  OUT     UCONTEXT               *UContext,
  IN      UCONTEXT_START_ROUTINE StartRoutine,
  IN      VOID                   *Stack,
  IN      UINTN                  StackSize
  );

INTN
SwapContext (
  OUT     UCONTEXT          *OldUContext,
  IN      CONST UCONTEXT    *UContext
  );

#endif
