#include <PiDxe.h>
#include <Protocol/UefiThread.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>

#include "private/kernel/thread.h"
#include "private/kernel/event.h"
#include "private/kernel/mutex.h"
#include "private/kernel/semaphore.h"

#define HANDLE_TO_THREAD(h) ((thread_t*)(h))
#define HANDLE_TO_EVENT(h) ((event_t*)(h))
#define HANDLE_TO_MUTEX(h) ((mutex_t*)(h))
#define HANDLE_TO_SEMAPHORE(h) ((semaphore_t*)(h))

STATIC EFI_EVENT mTimerEvent;
STATIC EFI_EVENT mTimerEventNotify;
STATIC volatile THREAD_TIME_MS mTicks = 0;

STATIC VOID EFIAPI ThreadSetName(CONST CHAR8 *name) {
  thread_set_name(name);
}

STATIC VOID EFIAPI ThreadSetPriority(INTN priority) {
  thread_set_priority(priority);
}

STATIC EFI_STATUS EFIAPI ThreadCreate(THREAD *pthread, CONST CHAR8 *name, THREAD_START_ROUTINE entry, VOID *arg, INTN priority, UINTN stack_size) {
  thread_t *thread = thread_create(name, entry, arg, priority, stack_size);
  if (thread) {
    *pthread = thread;
    return EFI_SUCCESS;
  }

  return EFI_OUT_OF_RESOURCES;
}

STATIC EFI_STATUS EFIAPI ThreadResume(THREAD handle) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  return thread_resume(thread);
}

STATIC VOID EFIAPI NORETURN ThreadExit(INTN retcode) {
  thread_exit(retcode);
}

STATIC VOID EFIAPI ThreadSleep(THREAD_TIME_MS delay) {
  thread_sleep(delay);
}

STATIC EFI_STATUS EFIAPI ThreadDetach(THREAD handle) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  return thread_detach(thread);
}

STATIC EFI_STATUS EFIAPI ThreadJoin(THREAD handle, INTN *retcode, THREAD_TIME_MS timeout) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  return thread_join(thread, retcode, timeout);
}

STATIC EFI_STATUS EFIAPI ThreadDetachAndResume(THREAD handle) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  return thread_detach_and_resume(thread);
}

STATIC EFI_STATUS EFIAPI ThreadSetRealTime(THREAD handle) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  return thread_set_real_time(thread);
}

STATIC THREAD EFIAPI ThreadSelf(VOID) {
  return get_current_thread();
}


STATIC VOID EFIAPI ThreadYield(VOID) {
  thread_yield();
}

STATIC VOID EFIAPI ThreadBlock(VOID) {
  thread_block();
}

STATIC VOID EFIAPI ThreadUnBlock(THREAD handle, BOOLEAN resched) {
  thread_t *thread = HANDLE_TO_THREAD (handle);
  thread_unblock(thread, resched);
}


STATIC EFI_STATUS EFIAPI EventCreate(THREAD_EVENT *pevent, BOOLEAN initial, UINTN flags) {
  event_t *event = NULL;

  event = AllocatePool(sizeof(*event));
  if (event==NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  event_init(event, initial, flags);
  *pevent = event;

  return EFI_SUCCESS;
}

STATIC VOID EFIAPI EventDestroy(THREAD_EVENT handle) {
  event_t *event = HANDLE_TO_EVENT (handle);
  event_destroy(event);
  FreePool(event);
}

STATIC EFI_STATUS EFIAPI EventWait(THREAD_EVENT handle) {
  event_t *event = HANDLE_TO_EVENT (handle);
  return event_wait(event);
}

STATIC EFI_STATUS EFIAPI EventWaitTimeout(THREAD_EVENT handle, THREAD_TIME_MS timeout) {
  event_t *event = HANDLE_TO_EVENT (handle);
  return event_wait_timeout(event, timeout);
}

STATIC EFI_STATUS EFIAPI EventSignal(THREAD_EVENT handle, BOOLEAN reschedule) {
  event_t *event = HANDLE_TO_EVENT (handle);
  return event_signal(event, reschedule);
}

STATIC EFI_STATUS EFIAPI EventUnSignal(THREAD_EVENT handle) {
  event_t *event = HANDLE_TO_EVENT (handle);
  return event_unsignal(event);
}


STATIC EFI_STATUS EFIAPI SemCreate(SEMAPHORE *psem, UINTN value) {
  semaphore_t *sem = NULL;

  sem = AllocatePool(sizeof(*sem));
  if (sem==NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  sem_init(sem, value);
  *psem = sem;

  return EFI_SUCCESS;
}

STATIC VOID EFIAPI SemDestroy(SEMAPHORE handle) {
  semaphore_t *sem = HANDLE_TO_SEMAPHORE (handle);
  sem_destroy(sem);
  FreePool(sem);
}

STATIC INTN EFIAPI SemPost(SEMAPHORE handle, BOOLEAN resched) {
  semaphore_t *sem = HANDLE_TO_SEMAPHORE (handle);
  return sem_post(sem, resched);
}

STATIC EFI_STATUS EFIAPI SemWait(SEMAPHORE handle) {
  semaphore_t *sem = HANDLE_TO_SEMAPHORE (handle);
  return sem_wait(sem);
}

STATIC EFI_STATUS EFIAPI SemTryWait(SEMAPHORE handle) {
  semaphore_t *sem = HANDLE_TO_SEMAPHORE (handle);
  return sem_trywait(sem);
}

STATIC EFI_STATUS EFIAPI SemTimedWait(SEMAPHORE handle, THREAD_TIME_MS duration) {
  semaphore_t *sem = HANDLE_TO_SEMAPHORE (handle);
  return sem_timedwait(sem, duration);
}


STATIC EFI_STATUS EFIAPI MutexCreate(MUTEX *pmutex) {
  mutex_t *mutex = NULL;

  mutex = AllocatePool(sizeof(*mutex));
  if (mutex==NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  mutex_init(mutex);
  *pmutex = mutex;

  return EFI_SUCCESS;
}

STATIC VOID EFIAPI MutexDestroy(MUTEX handle) {
  mutex_t *mutex = HANDLE_TO_MUTEX (handle);
  mutex_destroy(mutex);
  FreePool(mutex);
}

STATIC EFI_STATUS EFIAPI MutexAcquire(MUTEX handle) {
  mutex_t *mutex = HANDLE_TO_MUTEX (handle);
  return mutex_acquire(mutex);
}

STATIC EFI_STATUS EFIAPI MutexAcquireTimeout(MUTEX handle, THREAD_TIME_MS timeout) {
  mutex_t *mutex = HANDLE_TO_MUTEX (handle);
  return mutex_acquire_timeout(mutex, timeout);
}

STATIC EFI_STATUS EFIAPI MutexRelease(MUTEX handle) {
  mutex_t *mutex = HANDLE_TO_MUTEX (handle);
  return mutex_release(mutex);
}

STATIC BOOLEAN EFIAPI MutexHeld(MUTEX handle) {
  mutex_t *mutex = HANDLE_TO_MUTEX (handle);
  return is_mutex_held(mutex);
}

STATIC UINT64 EFIAPI CurrentTimeNs(VOID) {
  ASSERT(EfiGetCurrentTpl()<TPL_NOTIFY);
  return mTicks*1000000ULL;
}

STATIC EFI_STATUS TlsCreate(UINTN *pkey, TLS_DESTRUCTOR destructor) {
  return tls_create(pkey, destructor);
}

STATIC EFI_STATUS TlsDelete(UINTN key) {
  return tls_delete(key);
}

STATIC EFI_STATUS TlsSet(UINTN key, VOID *data) {
  return tls_set(key, data);
}

STATIC VOID* TlsGet(UINTN key) {
  return tls_get(key);
}

STATIC UEFI_THREAD_PROTOCOL mThreads = {
  .ThreadSetName = ThreadSetName,
  .ThreadSetPriority = ThreadSetPriority,
  .ThreadCreate = ThreadCreate,
  .ThreadResume = ThreadResume,
  .ThreadExit = ThreadExit,
  .ThreadSleep = ThreadSleep,
  .ThreadDetach = ThreadDetach,
  .ThreadJoin = ThreadJoin,
  .ThreadDetachAndResume = ThreadDetachAndResume,
  .ThreadSetRealTime = ThreadSetRealTime,
  .ThreadSelf = ThreadSelf,

  .ThreadYield = ThreadYield,
  .ThreadBlock = ThreadBlock,
  .ThreadUnBlock = ThreadUnBlock,

  .EventCreate = EventCreate,
  .EventDestroy = EventDestroy,
  .EventWait = EventWait,
  .EventWaitTimeout = EventWaitTimeout,
  .EventSignal = EventSignal,
  .EventUnSignal = EventUnSignal,

  .SemCreate = SemCreate,
  .SemDestroy = SemDestroy,
  .SemPost = SemPost,
  .SemWait = SemWait,
  .SemTryWait = SemTryWait,
  .SemTimedWait = SemTimedWait,

  .MutexCreate = MutexCreate,
  .MutexDestroy = MutexDestroy,
  .MutexAcquire = MutexAcquire,
  .MutexAcquireTimeout = MutexAcquireTimeout,
  .MutexRelease = MutexRelease,
  .MutexHeld = MutexHeld,

  .CurrentTimeNs = CurrentTimeNs,

  .TlsCreate = TlsCreate,
  .TlsDelete = TlsDelete,
  .TlsSet = TlsSet,
  .TlsGet = TlsGet,
};

STATIC
VOID
EFIAPI
TimerCallback (
    IN  EFI_EVENT   Event,
    IN  VOID        *Context
)
{
  if (thread_timer_tick()==INT_RESCHEDULE) {
    thread_preempt();
  }
}

STATIC
VOID
EFIAPI
TimerCallbackNotify (
    IN  EFI_EVENT   Event,
    IN  VOID        *Context
)
{
  mTicks += 10;
}

EFI_STATUS
EFIAPI
UEFIThreadsEntry (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_HANDLE Handle = NULL;
  EFI_STATUS Status;

  thread_init_early();
  thread_init();
  thread_create_idle();
  thread_set_priority(DEFAULT_PRIORITY);

  //
  // create TPL_CALLBACK timer
  //
  Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK, TimerCallback, NULL, &mTimerEvent);
  ASSERT_EFI_ERROR (Status);

  Status = gBS->SetTimer (mTimerEvent, TimerPeriodic, MS2100N(10));
  ASSERT_EFI_ERROR (Status);

  //
  // create TPL_NOTIFY timer
  //
  Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_NOTIFY, TimerCallbackNotify, NULL, &mTimerEventNotify);
  ASSERT_EFI_ERROR (Status);

  Status = gBS->SetTimer (mTimerEventNotify, TimerPeriodic, MS2100N(10));
  ASSERT_EFI_ERROR (Status);

  Status = gBS->InstallMultipleProtocolInterfaces(
                  &Handle,
                  &gUefiThreadProtocolGuid,      &mThreads,
                  NULL
                  );
  ASSERT_EFI_ERROR(Status);

  return Status;
}
