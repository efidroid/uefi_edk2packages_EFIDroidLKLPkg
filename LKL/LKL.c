#include "LKLPrivate.h"

int encode_dev_from_sysfs(const char *sysfs_path, UINT32 *pdevid);

UEFI_THREAD_PROTOCOL   *gThreads = NULL;

STATIC UEFI_LKL_PROTOCOL mLKL = {
  .HostOps = &lkl_host_ops,
  .Syscall = lkl_syscall,
  .TriggerIrq = lkl_trigger_irq,
  .GetFreeIrq = lkl_get_free_irq,
  .PutIrq = lkl_put_irq,
  .IsRunning = lkl_is_running,

  .virtio_req_complete = virtio_req_complete,
  .virtio_process_queue = virtio_process_queue,
  .virtio_set_queue_max_merge_len = virtio_set_queue_max_merge_len,
  .virtio_dev_setup = virtio_dev_setup,
  .virtio_dev_cleanup = virtio_dev_cleanup,
  .virtio_get_num_bootdevs = virtio_get_num_bootdevs,

  .lkl_disk_add = lkl_disk_add,
  .lkl_disk_remove = lkl_disk_remove,

  .lkl_mount_fs = lkl_mount_fs,
  .lkl_encode_dev_from_sysfs = lkl_encode_dev_from_sysfs,
  .lkl_get_virtio_blkdev = lkl_get_virtio_blkdev,
  .lkl_mount_dev = lkl_mount_dev,
  .lkl_umount_timeout = lkl_umount_timeout,
  .lkl_umount_dev = lkl_umount_dev,

  .lkl_opendir = lkl_opendir,
  .lkl_fdopendir = lkl_fdopendir,
  .lkl_rewinddir = lkl_rewinddir,
  .lkl_closedir = lkl_closedir,
  .lkl_readdir = lkl_readdir,
  .lkl_errdir = lkl_errdir,
  .lkl_dirfd= lkl_dirfd,
  .lkl_set_fd_limit = lkl_set_fd_limit,

  .lkl_strerror = lkl_strerror,
};

static EFI_STATUS CONST LKLErrorStatus[] = {
  EFI_SUCCESS,
  EFI_ACCESS_DENIED, // LKL_EPERM
  EFI_NOT_FOUND, // LKL_ENOENT
  EFI_NOT_FOUND, // LKL_ESRCH
  EFI_ABORTED, // LKL_EINTR
  EFI_DEVICE_ERROR, // LKL_EIO
  EFI_NOT_FOUND, // LKL_ENXIO
  EFI_INVALID_PARAMETER, // LKL_E2BIG
  EFI_INVALID_PARAMETER, // LKL_ENOEXEC
  EFI_INVALID_PARAMETER, // LKL_EBADF

  EFI_NOT_FOUND, // LKL_ECHILD
  EFI_NOT_READY, // LKL_EAGAIN
  EFI_OUT_OF_RESOURCES, // LKL_ENOMEM
  EFI_ACCESS_DENIED, // LKL_EACCES
  EFI_INVALID_PARAMETER, // LKL_EFAULT
  EFI_INVALID_PARAMETER, // LKL_ENOTBLK
  EFI_DEVICE_ERROR, // LKL_EBUSY
  EFI_DEVICE_ERROR, // LKL_EEXIST
  EFI_DEVICE_ERROR, // LKL_EXDEV
  EFI_NOT_FOUND, // LKL_ENODEV

  EFI_INVALID_PARAMETER, // LKL_ENOTDIR
  EFI_INVALID_PARAMETER, // LKL_EISDIR
  EFI_INVALID_PARAMETER, // LKL_EINVAL
  EFI_OUT_OF_RESOURCES, // LKL_ENFILE
  EFI_OUT_OF_RESOURCES, // LKL_EMFILE
  EFI_INVALID_PARAMETER, // LKL_ENOTTY
  EFI_NOT_READY, // LKL_ETXTBSY
  EFI_OUT_OF_RESOURCES, // LKL_EFBIG
  EFI_VOLUME_FULL, // LKL_ENOSPC
  EFI_INVALID_PARAMETER, // LKL_ESPIPE

  EFI_WRITE_PROTECTED, // LKL_EROFS
  EFI_OUT_OF_RESOURCES, // LKL_EMLINK
  EFI_DEVICE_ERROR, // LKL_EPIPE
  EFI_INVALID_PARAMETER, // LKL_EDOM
  EFI_UNSUPPORTED, // LKL_ERANGE
  EFI_UNSUPPORTED, // LKL_EDEADLK
  EFI_INVALID_PARAMETER, // LKL_ENAMETOOLONG
  EFI_OUT_OF_RESOURCES, // LKL_ENOLCK
  EFI_INVALID_PARAMETER, // LKL_ENOSYS
  EFI_UNSUPPORTED, // LKL_ENOTEMPTY

  EFI_DEVICE_ERROR, // LKL_ELOOP
  EFI_UNSUPPORTED,  // LKL_EWOULDBLOCK
  EFI_UNSUPPORTED, // LKL_ENOMSG
  EFI_INVALID_PARAMETER, // LKL_EIDRM
  EFI_INVALID_PARAMETER, // LKL_ECHRNG
  EFI_DEVICE_ERROR, // LKL_EL2NSYNC
  EFI_DEVICE_ERROR, // LKL_EL3HLT
  EFI_DEVICE_ERROR, // LKL_EL3RST
  EFI_INVALID_PARAMETER, // LKL_ELNRNG
  EFI_DEVICE_ERROR, // LKL_EUNATCH

  EFI_NOT_READY, // LKL_ENOCSI
  EFI_DEVICE_ERROR, // LKL_EL2HLT
  EFI_DEVICE_ERROR, // LKL_EBADE
  EFI_INVALID_PARAMETER, // LKL_EBADR
  EFI_OUT_OF_RESOURCES, // LKL_EXFULL
  EFI_NOT_FOUND, // LKL_ENOANO
  EFI_INVALID_PARAMETER, // LKL_EBADRQC
  EFI_INVALID_PARAMETER, // LKL_EBADSLT
  EFI_UNSUPPORTED, // LKL_EDEADLOCK
  EFI_UNSUPPORTED, // LKL_EBFONT

  EFI_INVALID_PARAMETER, // LKL_ENOSTR
  EFI_NOT_READY, // LKL_ENODATA
  EFI_TIMEOUT, // LKL_ETIME
  EFI_OUT_OF_RESOURCES, // LKL_ENOSR
  EFI_NOT_FOUND, // LKL_ENONET
  EFI_NOT_FOUND, // LKL_ENOPKG
  EFI_NOT_FOUND, // LKL_EREMOTE
  EFI_DEVICE_ERROR, // LKL_ENOLINK
  EFI_DEVICE_ERROR, // LKL_EADV
  EFI_DEVICE_ERROR, // LKL_ESRMNT

  EFI_DEVICE_ERROR, // LKL_ECOMM
  EFI_PROTOCOL_ERROR, // LKL_EPROTO
  EFI_UNSUPPORTED, // LKL_EMULTIHOP
  EFI_DEVICE_ERROR, // LKL_EDOTDOT
  EFI_DEVICE_ERROR, // LKL_EBADMSG
  EFI_OUT_OF_RESOURCES, // LKL_EOVERFLOW
  EFI_DEVICE_ERROR, // LKL_ENOTUNIQ
  EFI_DEVICE_ERROR, // LKL_EBADFD
  EFI_DEVICE_ERROR, // LKL_EREMCHG
  EFI_NOT_FOUND, // LKL_ELIBACC

  EFI_LOAD_ERROR, // LKL_ELIBBAD
  EFI_LOAD_ERROR, // LKL_ELIBSCN
  EFI_LOAD_ERROR, // LKL_ELIBMAX
  EFI_LOAD_ERROR, // LKL_ELIBEXEC
  EFI_LOAD_ERROR, // LKL_EILSEQ
  EFI_DEVICE_ERROR, // LKL_ERESTART
  EFI_DEVICE_ERROR, // LKL_ESTRPIPE
  EFI_OUT_OF_RESOURCES, // LKL_EUSERS
  EFI_INVALID_PARAMETER, // LKL_ENOTSOCK
  EFI_INVALID_PARAMETER, // LKL_EDESTADDRREQ

  EFI_OUT_OF_RESOURCES, // LKL_EMSGSIZE
  EFI_INVALID_PARAMETER, // LKL_EPROTOTYPE
  EFI_NOT_FOUND, // LKL_ENOPROTOOPT
  EFI_UNSUPPORTED, // LKL_EPROTONOSUPPORT
  EFI_UNSUPPORTED, // LKL_ESOCKTNOSUPPORT
  EFI_UNSUPPORTED, // LKL_EOPNOTSUPP
  EFI_UNSUPPORTED, // LKL_EPFNOSUPPORT
  EFI_UNSUPPORTED, // LKL_EAFNOSUPPORT
  EFI_NOT_READY, // LKL_EADDRINUSE
  EFI_DEVICE_ERROR, // LKL_EADDRNOTAVAIL

  EFI_DEVICE_ERROR, // LKL_ENETDOWN
  EFI_DEVICE_ERROR, // LKL_ENETUNREACH
  EFI_DEVICE_ERROR, // LKL_ENETRESET
  EFI_ABORTED, // LKL_ECONNABORTED
  EFI_DEVICE_ERROR, // LKL_ECONNRESET
  EFI_OUT_OF_RESOURCES, // LKL_ENOBUFS
  EFI_DEVICE_ERROR, // LKL_EISCONN
  EFI_DEVICE_ERROR, // LKL_ENOTCONN
  EFI_DEVICE_ERROR, // LKL_ESHUTDOWN
  EFI_DEVICE_ERROR, // LKL_ETOOMANYREFS

  EFI_TIMEOUT, // LKL_ETIMEDOUT
  EFI_ACCESS_DENIED, // LKL_ECONNREFUSED
  EFI_DEVICE_ERROR, // LKL_EHOSTDOWN
  EFI_DEVICE_ERROR, // LKL_EHOSTUNREACH
  EFI_NOT_READY, // LKL_EALREADY
  EFI_NOT_READY, // LKL_EINPROGRESS
  EFI_INVALID_PARAMETER, // LKL_ESTALE
  EFI_DEVICE_ERROR, // LKL_EUCLEAN
  EFI_UNSUPPORTED, // LKL_ENOTNAM
  EFI_OUT_OF_RESOURCES, // LKL_ENAVAIL

  EFI_INVALID_PARAMETER, // LKL_EISNAM
  EFI_DEVICE_ERROR, // LKL_EREMOTEIO
  EFI_VOLUME_FULL, // LKL_EDQUOT
  EFI_NO_MEDIA, // LKL_ENOMEDIUM
  EFI_INVALID_PARAMETER, // LKL_EMEDIUMTYPE
  EFI_ABORTED, // LKL_ECANCELED
  EFI_NOT_FOUND, // LKL_ENOKEY
  EFI_DEVICE_ERROR, // LKL_EKEYEXPIRED
  EFI_DEVICE_ERROR, // LKL_EKEYREVOKED
  EFI_DEVICE_ERROR, // LKL_EKEYREJECTED

  EFI_INVALID_PARAMETER, // LKL_EOWNERDEAD
  EFI_DEVICE_ERROR, // LKL_ENOTRECOVERABLE
  EFI_INVALID_PARAMETER, // LKL_ERFKILL
  EFI_DEVICE_ERROR, // LKL_EHWPOISON
};

EFI_STATUS
LKLError2EfiError (
  INTN Error
)
{
  if (Error < 0)
    Error = -Error;

  if ((UINTN)Error >= sizeof(LKLErrorStatus) / sizeof(EFI_STATUS))
    return MAX_UINTN;

  return LKLErrorStatus[Error];
}

EFI_STATUS
EFIAPI
LKLEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS                Status;
  EFI_HANDLE Handle = NULL;
  long ret;

  Status = gBS->LocateProtocol (&gUefiThreadProtocolGuid, NULL, (VOID **)&gThreads);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // start linux kernel
  ret = lkl_start_kernel(&lkl_host_ops, "mem=64M");
  if (ret) {
    DEBUG((EFI_D_ERROR, "can't start kernel: %s\n", lkl_strerror(ret)));
    return LKLError2EfiError(ret);
  }

  Status = gBS->InstallMultipleProtocolInterfaces(
                  &Handle,
                  &gUefiLKLProtocolGuid,      &mLKL,
                  NULL
                  );
  if (EFI_ERROR(Status)) {
    return Status;
  }

  return Status;
}

EFI_STATUS
EFIAPI
LKLUnload (
  IN EFI_HANDLE  ImageHandle
  )
{
  EFI_STATUS  Status;

  Status = EFI_SUCCESS;

  lkl_sys_halt();

  // this kills the kernel's main thread
  gThreads->ThreadYield();

  return Status;
}
