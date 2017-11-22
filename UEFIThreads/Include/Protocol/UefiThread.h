#ifndef __PROTOCOL_UEFITHREAD_H__
#define __PROTOCOL_UEFITHREAD_H__

#define UEFI_THREAD_PROTOCOL_GUID \
  { 0xba4d789a, 0x9b1a, 0x473d, { 0x9a, 0x88, 0x74, 0xee, 0x6f, 0xd9, 0x02, 0xbf } }

#define THREAD_EVENT_FLAG_AUTOUNSIGNAL 1

#define NUM_PRIORITIES 32
#define LOWEST_PRIORITY 0
#define HIGHEST_PRIORITY (NUM_PRIORITIES - 1)
#define DPC_PRIORITY (NUM_PRIORITIES - 2)
#define IDLE_PRIORITY LOWEST_PRIORITY
#define LOW_PRIORITY (NUM_PRIORITIES / 4)
#define DEFAULT_PRIORITY (NUM_PRIORITIES / 2)
#define HIGH_PRIORITY ((NUM_PRIORITIES / 4) * 3)

#define DEFAULT_STACK_SIZE 4096

typedef UINTN THREAD_TIME_MS;
#define INFINITE_TIME MAX_UINTN

typedef struct _UEFI_THREAD_PROTOCOL   UEFI_THREAD_PROTOCOL;

typedef struct {
    VOID *Base;
    UINTN Length;
} CBUF_IOVEC;

typedef VOID *THREAD;
typedef VOID *THREAD_EVENT;
typedef VOID *SEMAPHORE;
typedef VOID *MUTEX;
typedef VOID *CBUF;
typedef INTN (*THREAD_START_ROUTINE)(VOID *arg);

typedef VOID       (EFIAPI *THREAD_SET_NAME)(CONST CHAR8 *name);
typedef VOID       (EFIAPI *THREAD_SET_PRIORITY)(INTN priority);
typedef EFI_STATUS (EFIAPI *THREAD_CREATE)(THREAD *pthread, CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, UINTN stack_size);
typedef EFI_STATUS (EFIAPI *THREAD_RESUME)(THREAD t);
typedef VOID       (EFIAPI *THREAD_EXIT)(INTN retcode) NORETURN;
typedef VOID       (EFIAPI *THREAD_SLEEP)(THREAD_TIME_MS delay);
typedef EFI_STATUS (EFIAPI *THREAD_DETACH)(THREAD t);
typedef EFI_STATUS (EFIAPI *THREAD_JOIN)(THREAD t, INTN *retcode, THREAD_TIME_MS timeout);
typedef EFI_STATUS (EFIAPI *THREAD_DETACH_AND_RESUME)(THREAD t);
typedef EFI_STATUS (EFIAPI *THREAD_SET_REAL_TIME)(THREAD t);
typedef THREAD     (EFIAPI *THREAD_SELF)(VOID);

typedef VOID       (EFIAPI *THREAD_YIELD)(VOID);
typedef VOID       (EFIAPI *THREAD_BLOCK)(VOID);
typedef VOID       (EFIAPI *THREAD_UNBLOCK)(THREAD t, BOOLEAN resched);

typedef EFI_STATUS (EFIAPI *EVENT_CREATE)(THREAD_EVENT *pevent, BOOLEAN initial, UINTN flags);
typedef VOID       (EFIAPI *EVENT_DESTROY)(THREAD_EVENT event);
typedef EFI_STATUS (EFIAPI *EVENT_WAIT)(THREAD_EVENT event);
typedef EFI_STATUS (EFIAPI *EVENT_WAIT_TIMEOUT)(THREAD_EVENT event, THREAD_TIME_MS timeout);
typedef EFI_STATUS (EFIAPI *EVENT_SIGNAL)(THREAD_EVENT event, BOOLEAN reschedule);
typedef EFI_STATUS (EFIAPI *EVENT_UNSIGNAL)(THREAD_EVENT event);

typedef EFI_STATUS (EFIAPI *SEM_CREATE)(SEMAPHORE *psem, UINTN value);
typedef VOID       (EFIAPI *SEM_DESTROY)(SEMAPHORE sem);
typedef INTN       (EFIAPI *SEM_POST)(SEMAPHORE sem, BOOLEAN resched);
typedef EFI_STATUS (EFIAPI *SEM_WAIT)(SEMAPHORE sem);
typedef EFI_STATUS (EFIAPI *SEM_TRYWAIT)(SEMAPHORE sem);
typedef EFI_STATUS (EFIAPI *SEM_TIMED_WAIT)(SEMAPHORE sem, THREAD_TIME_MS duration);

typedef EFI_STATUS (EFIAPI *MUTEX_CREATE)(MUTEX *pmutex);
typedef VOID       (EFIAPI *MUTEX_DESTROY)(MUTEX mutex);
typedef EFI_STATUS (EFIAPI *MUTEX_ACQUIRE)(MUTEX mutex);
typedef EFI_STATUS (EFIAPI *MUTEX_ACQUIRE_TIMEOUT)(MUTEX mutex, THREAD_TIME_MS timeout);
typedef EFI_STATUS (EFIAPI *MUTEX_RELEASE)(MUTEX mutex);
typedef BOOLEAN    (EFIAPI *MUTEX_HELD)(MUTEX mutex);

typedef UINT64     (EFIAPI *CURRENT_TIME_NS)(VOID);

typedef VOID (*TLS_DESTRUCTOR)(VOID *);
typedef EFI_STATUS (EFIAPI *TLS_CREATE)(UINTN *pkey, TLS_DESTRUCTOR destructor);
typedef EFI_STATUS (EFIAPI *TLS_DELETE)(UINTN key);
typedef EFI_STATUS (EFIAPI *TLS_SET)(UINTN key, VOID *data);
typedef VOID*      (EFIAPI *TLS_GET)(UINTN key);

typedef EFI_STATUS (EFIAPI *CBUF_CREATE)(CBUF *pcbuf, UINTN len, VOID *buf);
typedef VOID       (EFIAPI *CBUF_DESTROY)(CBUF mutex);
typedef UINTN      (EFIAPI *CBUF_READ)(CBUF cbuf, VOID *buf, UINTN buflen, BOOLEAN block);
typedef UINTN      (EFIAPI *CBUF_PEEK)(CBUF cbuf, CBUF_IOVEC *regions, BOOLEAN block, THREAD_TIME_MS timeout);
typedef UINTN      (EFIAPI *CBUF_WRITE)(CBUF cbuf, CONST VOID *buf, UINTN len, BOOLEAN canreschedule);
typedef UINTN      (EFIAPI *CBUF_SPACE_AVAILABLE)(CBUF cbuf);
typedef UINTN      (EFIAPI *CBUF_SPACE_USED)(CBUF cbuf);
typedef UINTN      (EFIAPI *CBUF_SIZE)(CBUF cbuf);
typedef VOID       (EFIAPI *CBUF_RESET)(CBUF cbuf);

struct _UEFI_THREAD_PROTOCOL {
  THREAD_SET_NAME           ThreadSetName;
  THREAD_SET_PRIORITY       ThreadSetPriority;
  THREAD_CREATE             ThreadCreate;
  THREAD_RESUME             ThreadResume;
  THREAD_EXIT               ThreadExit;
  THREAD_SLEEP              ThreadSleep;
  THREAD_DETACH             ThreadDetach;
  THREAD_JOIN               ThreadJoin;
  THREAD_DETACH_AND_RESUME  ThreadDetachAndResume;
  THREAD_SET_REAL_TIME      ThreadSetRealTime;
  THREAD_SELF               ThreadSelf;

  THREAD_YIELD              ThreadYield;
  THREAD_BLOCK              ThreadBlock;
  THREAD_UNBLOCK            ThreadUnBlock;

  EVENT_CREATE              EventCreate;
  EVENT_DESTROY             EventDestroy;
  EVENT_WAIT                EventWait;
  EVENT_WAIT_TIMEOUT        EventWaitTimeout;
  EVENT_SIGNAL              EventSignal;
  EVENT_UNSIGNAL            EventUnSignal;

  SEM_CREATE                SemCreate;
  SEM_DESTROY               SemDestroy;
  SEM_POST                  SemPost;
  SEM_WAIT                  SemWait;
  SEM_TRYWAIT               SemTryWait;
  SEM_TIMED_WAIT            SemTimedWait;

  MUTEX_CREATE              MutexCreate;
  MUTEX_DESTROY             MutexDestroy;
  MUTEX_ACQUIRE             MutexAcquire;
  MUTEX_ACQUIRE_TIMEOUT     MutexAcquireTimeout; 
  MUTEX_RELEASE             MutexRelease;
  MUTEX_HELD                MutexHeld;

  CURRENT_TIME_NS           CurrentTimeNs;

  TLS_CREATE                TlsCreate;
  TLS_DELETE                TlsDelete;
  TLS_SET                   TlsSet;
  TLS_GET                   TlsGet;

  CBUF_CREATE               CBufCreate;
  CBUF_DESTROY              CBufDestroy;
  CBUF_READ                 CBufRead;
  CBUF_PEEK                 CBufPeek;
  CBUF_WRITE                CBufWrite;
  CBUF_SPACE_AVAILABLE      CBufSpaceAvailable;
  CBUF_SPACE_USED           CBufSpaceUsed;
  CBUF_SIZE                 CBufSize;
  CBUF_RESET                CBufReset;
};

extern EFI_GUID gUefiThreadProtocolGuid;

#endif
