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
//GetContext (
//  OUT     UCONTEXT *UContext
//  );
ASM_FUNC(GetContext)
  // save non callee trashed supervisor registers
  add r1, r0, #UCONTEXT_R4
  stmia r1, {r4-r12}

  // save sp
  str sp, [r0, #UCONTEXT_SP]
  // save lr
  str lr, [r0, #UCONTEXT_LR]
  // return to pc
  str lr, [r0, #UCONTEXT_PC]
  // set return-value to 0
  mov r2, #0
  str r2, [r0, #UCONTEXT_R0]

  add r1, r0, #UCONTEXT_D8

  // save floating point registers
  // vstmia r1!, {d8-d15}
  stc p11, cr8, [r1], #64

  // save fpscr
  // vmrs r2, fpscr
  mrc p10, 7, r2, cr1, cr0, 0
  str r2, [r1], #4

  // return 0
  mov r0, #0
  bx lr
