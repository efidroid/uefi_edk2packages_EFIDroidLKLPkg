#include "LKLPrivate.h"
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>

struct lkl_mutex {
	int recursive;
	MUTEX mutex;
	SEMAPHORE sem;
};

struct lkl_sem {
	SEMAPHORE sem;
};

struct lkl_tls_key {
	UINTN key;
};

typedef struct {
	EFI_EVENT event;
	void (*fn)(void *);
	void *arg;
	THREAD_EVENT cond;
	THREAD thread;

	int request_stop;
} ltimer_t;

typedef BASE_LIBRARY_JUMP_BUFFER jmp_buf[1];

static void print(const char *str, int len)
{
	int i;
	for (i=0; i<len; i++) {
		if (str[i]=='\n')
			DEBUG((EFI_D_INFO, "\r"));

		DEBUG((EFI_D_INFO, "%c", str[i]));
	}
}

static struct lkl_sem *sem_alloc(int count)
{
	struct lkl_sem *sem;

	sem = AllocatePool(sizeof(*sem));
	if (!sem)
		return NULL;

	EFI_STATUS Status = gThreads->SemCreate(&sem->sem, count);
	if (EFI_ERROR(Status))
		return NULL;

	return sem;
}

static void sem_free(struct lkl_sem *sem)
{
	gThreads->SemDestroy(sem->sem);
	FreePool(sem);
}

static void sem_up(struct lkl_sem *sem)
{
	gThreads->SemPost(sem->sem, 1);
}

static void sem_down(struct lkl_sem *sem)
{
	int err;
	do {
		err = gThreads->SemWait(sem->sem);
	} while (err < 0);
}

static struct lkl_mutex *mutex_alloc(int recursive)
{
	EFI_STATUS Status;
	struct lkl_mutex *mutex = AllocatePool(sizeof(struct lkl_mutex));

	if (!mutex)
		return NULL;

	if (recursive)
		Status = gThreads->MutexCreate(&mutex->mutex);
	else
		Status = gThreads->SemCreate(&mutex->sem, 1);
	if (EFI_ERROR(Status)) {
		FreePool(mutex);
		return NULL;
	}

	mutex->recursive = recursive;

	return mutex;
}

static void mutex_lock(struct lkl_mutex *mutex)
{
	int err;

	if (mutex->recursive)
		gThreads->MutexAcquire(mutex->mutex);
	else {
		do {
			err = gThreads->SemWait(mutex->sem);
		} while (err < 0);
	}
}

static void mutex_unlock(struct lkl_mutex *mutex)
{
	if (mutex->recursive)
		gThreads->MutexRelease(mutex->mutex);
	else {
		gThreads->SemPost(mutex->sem, 1);
	}
}

static void mutex_free(struct lkl_mutex *mutex)
{
	if (mutex->recursive)
		gThreads->MutexDestroy(mutex->mutex);
	else
		gThreads->SemDestroy(mutex->sem);
	FreePool(mutex);
}

static lkl_thread_t lkl_thread_create(void (*fn)(void *), void *arg)
{
	EFI_STATUS Status;
	THREAD thread;

	Status = gThreads->ThreadCreate(&thread, "lkl", (int (*)(void *))fn, arg, DEFAULT_PRIORITY, 2*1024*1024);
	if (EFI_ERROR(Status))
		return 0;
	else {
		gThreads->ThreadResume(thread);
		return (lkl_thread_t) thread;
	}
}

static void lkl_thread_detach(void)
{
	gThreads->ThreadDetach(gThreads->ThreadSelf());
}

static void lkl_thread_exit(void)
{
	gThreads->ThreadExit(0);
}

static int lkl_thread_join(lkl_thread_t tid)
{
	if (gThreads->ThreadJoin((THREAD *)tid, NULL, INFINITE_TIME))
		return -1;
	else
		return 0;
}

static lkl_thread_t thread_self(void)
{
	return (lkl_thread_t)gThreads->ThreadSelf();
}

static int thread_equal(lkl_thread_t a, lkl_thread_t b)
{
	return a==b;
}

static struct lkl_tls_key *tls_alloc(void (*destructor)(void *))
{
	struct lkl_tls_key *ret = AllocatePool(sizeof(struct lkl_tls_key));

	if (EFI_ERROR(gThreads->TlsCreate(&ret->key, destructor))) {
		FreePool(ret);
		return NULL;
	}
	return ret;
}

static void tls_free(struct lkl_tls_key *key)
{
	gThreads->TlsDelete(key->key);
	FreePool(key);
}

static int tls_set(struct lkl_tls_key *key, void *data)
{
	if (EFI_ERROR(gThreads->TlsSet(key->key, data)))
		return -1;
	return 0;
}

static void *tls_get(struct lkl_tls_key *key)
{
	return gThreads->TlsGet(key->key);
}

VOID
EFIAPI
LTimerCallback (
	IN  EFI_EVENT   Event,
	IN  VOID		*Context
)
{
	ltimer_t *timer = Context;
	gThreads->EventSignal(timer->cond, 1);
}

static int timer_thread(void *pdata)
{
	ltimer_t *timer = pdata;

	while (!timer->request_stop) {
		gThreads->EventWait(timer->cond);
		if (timer->request_stop)
			break;

		timer->fn(timer->arg);
	}

	gThreads->EventDestroy(timer->cond);
	FreePool(timer);

	return 0;
}

static void *timer_alloc(void (*fn)(void *), void *arg)
{
	EFI_STATUS Status;

	ltimer_t *timer = AllocatePool(sizeof(ltimer_t));
	ASSERT(timer);
	timer->fn = fn;
	timer->arg = arg;
	timer->request_stop = 0;
	gThreads->EventCreate(&timer->cond, 0, THREAD_EVENT_FLAG_AUTOUNSIGNAL);

	Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK, LTimerCallback, timer, &timer->event);
	ASSERT_EFI_ERROR (Status);

	Status = gThreads->ThreadCreate(&timer->thread, "lkl_timer", timer_thread, timer, DEFAULT_PRIORITY, 1*1024*1024);
	gThreads->ThreadDetachAndResume(timer->thread);

	return timer;
}

static int timer_set_oneshot(void *_timer, unsigned long ns)
{
	EFI_STATUS Status;
	ltimer_t *timer = _timer;

	Status = gBS->SetTimer (timer->event, TimerCancel, 0);
	ASSERT_EFI_ERROR (Status);

	Status = gBS->SetTimer (timer->event, TimerRelative, ns/100);
	ASSERT_EFI_ERROR (Status);
	return 0;
}

static void timer_free(void *_timer)
{
	EFI_STATUS Status;
	ltimer_t *timer = _timer;

	Status = gBS->CloseEvent (timer->event);
	ASSERT_EFI_ERROR (Status);

	timer->request_stop = 1;
	gThreads->EventSignal(timer->cond, 1);
}

static void lkl_panic(void)
{
	ASSERT(0);
}

static long _gettid(void)
{
	return (long)gThreads->ThreadSelf();
}

static unsigned long long lkl_time(void)
{
	return (unsigned long long)gThreads->CurrentTimeNs();
}

static void *lkl_mem_alloc(unsigned long size)
{
	return AllocatePool(size);
}

static void lkl_mem_free(void *ptr)
{
	if (ptr) FreePool(ptr);
}

static void jmp_buf_set(struct lkl_jmp_buf *jmpb, void (*f)(void))
{
	if (!SetJump(*((jmp_buf *)jmpb->buf)))
		f();
}

static void jmp_buf_longjmp(struct lkl_jmp_buf *jmpb, int val)
{
	LongJump(*((jmp_buf *)jmpb->buf), (val == 0) ? 1 : val);
}

static void lkl_idle_loop_callback(void) {
	EfiGetCurrentTpl();
	gThreads->ThreadYield();
}

struct lkl_host_operations lkl_host_ops = {
	.panic = lkl_panic,
	.thread_create = lkl_thread_create,
	.thread_detach = lkl_thread_detach,
	.thread_exit = lkl_thread_exit,
	.thread_join = lkl_thread_join,
	.thread_self = thread_self,
	.thread_equal = thread_equal,
	.sem_alloc = sem_alloc,
	.sem_free = sem_free,
	.sem_up = sem_up,
	.sem_down = sem_down,
	.mutex_alloc = mutex_alloc,
	.mutex_free = mutex_free,
	.mutex_lock = mutex_lock,
	.mutex_unlock = mutex_unlock,
	.tls_alloc = tls_alloc,
	.tls_free = tls_free,
	.tls_set = tls_set,
	.tls_get = tls_get,
	.time = lkl_time,
	.timer_alloc = timer_alloc,
	.timer_set_oneshot = timer_set_oneshot,
	.timer_free = timer_free,
	.print = print,
	.mem_alloc = lkl_mem_alloc,
	.mem_free = lkl_mem_free,
	.ioremap = lkl_ioremap,
	.iomem_access = lkl_iomem_access,
	.virtio_devices = lkl_virtio_devs,
	.gettid = _gettid,
	.jmp_buf_set = jmp_buf_set,
	.jmp_buf_longjmp = jmp_buf_longjmp,
	.idle_loop_callback = lkl_idle_loop_callback,
};

static int uefi_blk_get_capacity(struct lkl_disk disk, unsigned long long *res)
{
	return -1;
}

static int uefi_blk_request(struct lkl_disk disk, struct lkl_blk_req *req)
{
	return LKL_DEV_BLK_STATUS_UNSUP;
}

struct lkl_dev_blk_ops lkl_dev_blk_ops = {
	.get_capacity = uefi_blk_get_capacity,
	.request = uefi_blk_request,
};
