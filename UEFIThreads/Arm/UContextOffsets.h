#ifndef UCONTEXT_OFFSETS_H
#define UCONTEXT_OFFSETS_H

#define UCONTEXT_R0 (0 * 4)
#define UCONTEXT_R4 (4 * 4)
#define UCONTEXT_SP (13 * 4)
#define UCONTEXT_LR (14 * 4)
#define UCONTEXT_PC (15 * 4)
#define UCONTEXT_D8 (UCONTEXT_PC + 4)

#endif
