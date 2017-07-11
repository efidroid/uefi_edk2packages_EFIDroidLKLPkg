#include <PiDxe.h>
#include <Protocol/LKL.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>
#include <lkl.h>
#include <lkl_host.h>

STATIC UEFI_LKL_PROTOCOL *mLKL = NULL;

long lkl_syscall(long no, long *params) {
    return mLKL->Syscall(no, params);
}

int lkl_trigger_irq(int irq) {
    return mLKL->TriggerIrq(irq);
}

int lkl_get_free_irq(const char *user) {
    return mLKL->GetFreeIrq(user);
}

void lkl_put_irq(int irq, const char *name) {
    return mLKL->PutIrq(irq, name);
}

int lkl_is_running(void) {
    return mLKL->IsRunning();
}

void virtio_req_complete(struct virtio_req *req, UINT32 len) {
    mLKL->virtio_req_complete(req, len);
}

void virtio_process_queue(struct virtio_dev *dev, UINT32 qidx) {
    mLKL->virtio_process_queue(dev, qidx);
}

void virtio_set_queue_max_merge_len(struct virtio_dev *dev, int q, int len) {
    mLKL->virtio_set_queue_max_merge_len(dev, q, len);
}

int virtio_dev_setup(struct virtio_dev *dev, int queues, int num_max) {
    return mLKL->virtio_dev_setup(dev, queues, num_max);
}

int virtio_dev_cleanup(struct virtio_dev *dev) {
    return mLKL->virtio_dev_cleanup(dev);
}

UINT32 virtio_get_num_bootdevs(void) {
    return mLKL->virtio_get_num_bootdevs();
}

int lkl_disk_add(struct lkl_disk *disk) {
    return mLKL->lkl_disk_add(disk);
}

int lkl_disk_remove(struct lkl_disk disk) {
    return mLKL->lkl_disk_remove(disk);
}

int lkl_mount_fs(char *fstype) {
    return mLKL->lkl_mount_fs(fstype);
}

int lkl_encode_dev_from_sysfs(const char *sysfs_path, UINT32 *pdevid) {
    return mLKL->lkl_encode_dev_from_sysfs(sysfs_path, pdevid);
}

int lkl_get_virtio_blkdev(int disk_id, unsigned int part, UINT32 *pdevid) {
    return mLKL->lkl_get_virtio_blkdev(disk_id, part, pdevid);
}

long lkl_mount_dev(unsigned int disk_id, unsigned int part,
		   const char *fs_type, int flags,
		   const char *data, char *mnt_str, unsigned int mnt_str_len) {
    return mLKL->lkl_mount_dev(disk_id, part, fs_type, flags, data, mnt_str, mnt_str_len);
}

long lkl_umount_timeout(char *path, int flags, long timeout_ms) {
    return mLKL->lkl_umount_timeout(path, flags, timeout_ms);
}

long lkl_umount_dev(unsigned int disk_id, unsigned int part, int flags, long timeout_ms) {
    return mLKL->lkl_umount_dev(disk_id, part, flags, timeout_ms);
}

struct lkl_dir *lkl_opendir(const char *path, int *err) {
    return mLKL->lkl_opendir(path, err);
}

struct lkl_dir *lkl_fdopendir(int fd, int *err) {
    return mLKL->lkl_fdopendir(fd, err);
}

void lkl_rewinddir(struct lkl_dir *dir) {
    mLKL->lkl_rewinddir(dir);
}

int lkl_closedir(struct lkl_dir *dir) {
    return mLKL->lkl_closedir(dir);
}

struct lkl_linux_dirent64 *lkl_readdir(struct lkl_dir *dir) {
    return mLKL->lkl_readdir(dir);
}

int lkl_errdir(struct lkl_dir *dir) {
    return mLKL->lkl_errdir(dir);
}

int lkl_dirfd(struct lkl_dir *dir) {
    return mLKL->lkl_dirfd(dir);
}

int lkl_set_fd_limit(unsigned int fd_limit) {
    return mLKL->lkl_set_fd_limit(fd_limit);
}

const char *lkl_strerror(int err) {
    return mLKL->lkl_strerror(err);
}

RETURN_STATUS
EFIAPI
LKLBaseLibConstructor (
  VOID
  )
{
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gUefiLKLProtocolGuid, NULL, (VOID **)&mLKL);
  ASSERT_EFI_ERROR (Status);

  return Status;
}
