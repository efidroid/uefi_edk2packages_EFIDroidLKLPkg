/* Copyright (C) 2012-2017 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <http://www.gnu.org/licenses/>.  */

#include <AsmMacroIoLib.h>
#include "UContextOffsets.h"

//INTN
//SwapContext (
//  OUT     UCONTEXT          *OldUContext,
//  IN      CONST UCONTEXT    *UContext
//  );
ASM_FUNC(SwapContext)
  // save our arguments
  push {r0-r2, lr}

  // store current context in oucp
  bl GetContext
  mov r3, r0

  // restore our arguments
  pop {r0-r2, lr}

  // check return value of GetContext
  cmp r3, #0
  itt ne
  movne r0, r3
  bxne lr

  // directly return to our caller, not the caller of GetContext(us)
  str sp, [r0, #UCONTEXT_SP]
  str lr, [r0, #UCONTEXT_LR]
  str lr, [r0, #UCONTEXT_PC]

  // set new context to ucp
  mov r0, r1
  b SetContext
