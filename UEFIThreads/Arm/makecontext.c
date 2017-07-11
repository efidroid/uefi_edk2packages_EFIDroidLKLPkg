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

#include <Library/DebugLib.h>
#include "private/UContext.h"

#define ROUNDDOWN(a, b) ((a) & ~((b)-1))

VOID
MakeContext (
  OUT     UCONTEXT               *UContext,
  IN      UCONTEXT_START_ROUTINE StartRoutine,
  IN      VOID                   *Stack,
  IN      UINTN                  StackSize
  )
{
  UINTN StackTop;

  // create a default stack frame on the stack
  StackTop = (UINTN) (Stack + StackSize);

  // make sure the top of the stack is 8 byte aligned for EABI compliance
  StackTop = ROUNDDOWN(StackTop, 8);

  // set sp
  UContext->SP = (UINT32) StackTop;

  // set entry point
  UContext->PC = (UINT32) StartRoutine;
}
