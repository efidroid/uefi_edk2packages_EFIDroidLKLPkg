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
//SetContext (
//  IN      CONST UCONTEXT *UContext
//  );
ASM_FUNC(SetContext)
  mov r4, r0
  add r0, r0, #UCONTEXT_D8

  // restore floating point registers
  // vldmia r0!, {d8-d15}
  ldc p11, cr8, [r0], #64

  // restore fpscr
  ldr r1, [r0], #4
  // fmxr fpscr, r1
  mcr p10, 7, r1, cr1, cr0, 0

  // restore r0-r12
  add lr, r4, #UCONTEXT_R0
  ldmia lr, {r0-r12}

  // restore sp
  ldr sp, [lr, #UCONTEXT_SP]

  // restore lr and pc
  add lr, lr, #UCONTEXT_LR
  ldmia lr, {lr, pc}
