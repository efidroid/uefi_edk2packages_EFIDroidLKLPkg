#ifndef __PROTOCOL_LKL_H__
#define __PROTOCOL_LKL_H__

#include <lkl.h>
#include <lkl_host.h>
#include <virtio.h>

#define UEFI_LKL_PROTOCOL_GUID \
  { 0x9241b418, 0x9001, 0x474f, { 0x97, 0xbd, 0x51, 0xa7, 0xf1, 0xf1, 0xe7, 0xf4 } }

typedef struct _UEFI_LKL_PROTOCOL   UEFI_LKL_PROTOCOL;

typedef long       (EFIAPI *LKL_SYSCALL)(long no, long *params);
typedef int        (EFIAPI *LKL_TRIGGER_IRQ)(int irq);
typedef int        (EFIAPI *LKL_GET_FREE_IRQ)(const char *user);
typedef void       (EFIAPI *LKL_PUT_IRQ)(int irq, const char *name);
typedef int        (EFIAPI *LKL_IS_RUNNING)(void);

typedef void     (EFIAPI *virtio_req_complete_t)(struct virtio_req *req, UINT32 len);
typedef void     (EFIAPI *virtio_process_queue_t)(struct virtio_dev *dev, UINT32 qidx);
typedef void     (EFIAPI *virtio_set_queue_max_merge_len_t)(struct virtio_dev *dev, int q, int len);
typedef int      (EFIAPI *virtio_dev_setup_t)(struct virtio_dev *dev, int queues, int num_max);
typedef int      (EFIAPI *virtio_dev_cleanup_t)(struct virtio_dev *dev);
typedef UINT32   (EFIAPI *virtio_get_num_bootdevs_t)(void);

typedef int      (EFIAPI *lkl_disk_add_t)(struct lkl_disk *disk);
typedef int      (EFIAPI *lkl_disk_remove_t)(struct lkl_disk disk);

typedef int  (EFIAPI *lkl_mount_fs_t)(char *fstype);
typedef int  (EFIAPI *lkl_encode_dev_from_sysfs_t)(const char *sysfs_path, UINT32 *pdevid);
typedef int  (EFIAPI *lkl_get_virtio_blkdev_t)(int disk_id, unsigned int part, UINT32 *pdevid);
typedef long (EFIAPI *lkl_mount_dev_t)(unsigned int disk_id, unsigned int part,
		   const char *fs_type, int flags,
		   const char *data, char *mnt_str, unsigned int mnt_str_len);
typedef long (EFIAPI *lkl_umount_timeout_t)(char *path, int flags, long timeout_ms);
typedef long (EFIAPI *lkl_umount_dev_t)(unsigned int disk_id, unsigned int part, int flags, long timeout_ms);

typedef struct lkl_dir *(EFIAPI *lkl_opendir_t)(const char *path, int *err);
typedef struct lkl_dir *(EFIAPI *lkl_fdopendir_t)(int fd, int *err);
typedef void            (EFIAPI *lkl_rewinddir_t)(struct lkl_dir *dir);
typedef int             (EFIAPI *lkl_closedir_t)(struct lkl_dir *dir);
typedef struct lkl_linux_dirent64 *(EFIAPI *lkl_readdir_t)(struct lkl_dir *dir);
typedef int             (EFIAPI *lkl_errdir_t)(struct lkl_dir *dir);
typedef int             (EFIAPI *lkl_dirfd_t)(struct lkl_dir *dir);
typedef int             (EFIAPI *lkl_set_fd_limit_t)(unsigned int fd_limit);

typedef const char *    (EFIAPI *lkl_strerror_t)(int err);

struct _UEFI_LKL_PROTOCOL {
  struct lkl_host_operations *HostOps;

  LKL_SYSCALL               Syscall;
  LKL_TRIGGER_IRQ           TriggerIrq;
  LKL_GET_FREE_IRQ          GetFreeIrq;
  LKL_PUT_IRQ               PutIrq;
  LKL_IS_RUNNING            IsRunning;

  virtio_req_complete_t             virtio_req_complete;
  virtio_process_queue_t            virtio_process_queue;
  virtio_set_queue_max_merge_len_t  virtio_set_queue_max_merge_len;
  virtio_dev_setup_t                virtio_dev_setup;
  virtio_dev_cleanup_t              virtio_dev_cleanup;
  virtio_get_num_bootdevs_t         virtio_get_num_bootdevs;

  lkl_disk_add_t      lkl_disk_add;
  lkl_disk_remove_t   lkl_disk_remove;

  lkl_mount_fs_t           lkl_mount_fs;
  lkl_encode_dev_from_sysfs_t  lkl_encode_dev_from_sysfs;
  lkl_get_virtio_blkdev_t  lkl_get_virtio_blkdev;
  lkl_mount_dev_t          lkl_mount_dev;
  lkl_umount_timeout_t     lkl_umount_timeout;
  lkl_umount_dev_t         lkl_umount_dev;

  lkl_opendir_t       lkl_opendir;
  lkl_fdopendir_t     lkl_fdopendir;
  lkl_rewinddir_t     lkl_rewinddir;
  lkl_closedir_t      lkl_closedir;
  lkl_readdir_t       lkl_readdir;
  lkl_errdir_t        lkl_errdir;
  lkl_dirfd_t         lkl_dirfd;
  lkl_set_fd_limit_t  lkl_set_fd_limit;

  lkl_strerror_t      lkl_strerror;
};

extern EFI_GUID gUefiLKLProtocolGuid;

#endif
