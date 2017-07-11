#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/PrintLib.h>

#include <lkl.h>
#include <lkl/linux/ioctl.h>
#include <lkl/linux/dm-ioctl.h>
#include <lkl/linux/fs.h>
#include <lkl/asm/syscalls.h>

#define DM_CRYPT_BUF_SIZE 4096
#define MAX_CRYPTO_TYPE_NAME_LEN 64
#define TABLE_LOAD_RETRIES 10
#define DEVMAPPER_CONTROL_FILE "/dev/device-mapper"

struct crypt_mnt_ftr {
    UINT32 flags;         /* See above */
    UINT32 keysize;       /* in bytes */
    UINT64 fs_size; /* Size of the encrypted fs, in 512 byte sectors */
    unsigned char crypto_type_name[MAX_CRYPTO_TYPE_NAME_LEN]; /* The type of encryption
                                                               needed to decrypt this
                                                               partition, null terminated */
};

static unsigned int get_blkdev_size(int fd, int *perror)
{
    unsigned long nr_sec;
    int rc;

    rc = lkl_sys_ioctl(fd, LKL_BLKGETSIZE, (UINTN)&nr_sec);
    if (rc < 0) {
        nr_sec = 0;
    }

    if (perror)
        *perror = -rc;

    return nr_sec;
}


static void ioctl_init(struct lkl_dm_ioctl *io, UINTN dataSize, const char *name, unsigned flags)
{
    SetMem(io, dataSize, 0);
    io->data_size = dataSize;
    io->data_start = sizeof(struct lkl_dm_ioctl);
    io->version[0] = 4;
    io->version[1] = 0;
    io->version[2] = 0;
    io->flags = flags;
    if (name) {
        AsciiStrCpyS(io->name, sizeof(io->name), name);
    }
}

/* Convert a binary key of specified length into an ascii hex string equivalent,
 * without the leading 0x and with null termination
 */
static void convert_key_to_hex_ascii(const unsigned char *master_key,
                                     unsigned int keysize, char *master_key_ascii)
{
    unsigned int i, a;
    unsigned char nibble;

    for (i=0, a=0; i<keysize; i++, a+=2) {
        /* For each byte, write out two ascii hex digits */
        nibble = (master_key[i] >> 4) & 0xf;
        master_key_ascii[a] = nibble + (nibble > 9 ? 0x37 : 0x30);

        nibble = master_key[i] & 0xf;
        master_key_ascii[a+1] = nibble + (nibble > 9 ? 0x37 : 0x30);
    }

    /* Add the null termination */
    master_key_ascii[a] = '\0';

}

static int load_crypto_mapping_table(struct crypt_mnt_ftr *crypt_ftr, const unsigned char *master_key,
                                     const char *real_blk_name, const char *name, int fd,
                                     char *extra_params)
{
    char buffer[DM_CRYPT_BUF_SIZE];
    struct lkl_dm_ioctl *io;
    struct lkl_dm_target_spec *tgt;
    char *crypt_params;
    char master_key_ascii[129]; /* Large enough to hold 512 bit key and null */
    int i;
    int rc;

    io = (struct lkl_dm_ioctl *) buffer;

    /* Load the mapping table for this device */
    tgt = (struct lkl_dm_target_spec *) &buffer[sizeof(struct lkl_dm_ioctl)];

    ioctl_init(io, DM_CRYPT_BUF_SIZE, name, 0);
    io->target_count = 1;
    tgt->status = 0;
    tgt->sector_start = 0;
    tgt->length = crypt_ftr->fs_size;
    crypt_params = buffer + sizeof(struct lkl_dm_ioctl) + sizeof(struct lkl_dm_target_spec);

    convert_key_to_hex_ascii(master_key, crypt_ftr->keysize, master_key_ascii);
    AsciiStrCpyS(tgt->target_type, LKL_DM_MAX_TYPE_NAME, "crypt");

    AsciiSPrint(crypt_params,
            DM_CRYPT_BUF_SIZE - sizeof(struct lkl_dm_ioctl) - sizeof(struct lkl_dm_target_spec),
            "%a %a 0 %a 0 %a", crypt_ftr->crypto_type_name,
            master_key_ascii, real_blk_name, extra_params);

    crypt_params += AsciiStrLen(crypt_params) + 1;
    crypt_params = (char *) (((unsigned long)crypt_params + 7) & ~8); /* Align to an 8 byte boundary */
    tgt->next = crypt_params - buffer;

    for (i = 0; i < TABLE_LOAD_RETRIES; i++) {
        rc = lkl_sys_ioctl(fd, LKL_DM_TABLE_LOAD, (UINTN)io);
        if (!rc) {
            break;
        }
        DEBUG ((EFI_D_ERROR, "%i\n", -rc));
        gBS->Stall(500000);
    }

    if (i == TABLE_LOAD_RETRIES) {
        /* We failed to load the table, return an error */
        return -1;
    } else {
        return i + 1;
    }
}

static int create_crypto_blk_dev(struct crypt_mnt_ftr *crypt_ftr, const unsigned char *master_key,
                                 const char *real_blk_name, char *crypto_blk_name, const char *name)
{
    char buffer[DM_CRYPT_BUF_SIZE];
    struct lkl_dm_ioctl *io;
    unsigned int minor;
    int fd=0;
    int retval = -1;
    char *extra_params;
    int load_count;

    if ((fd = lkl_sys_open(DEVMAPPER_CONTROL_FILE, LKL_O_RDWR, 0)) < 0 ) {
        DEBUG ((EFI_D_ERROR, "Cannot open device-mapper\n"));
        goto errout;
    }

    io = (struct lkl_dm_ioctl *) buffer;

    ioctl_init(io, DM_CRYPT_BUF_SIZE, name, 0);
    retval = lkl_sys_ioctl(fd, LKL_DM_DEV_CREATE, (UINTN)io);
    if (retval) {
        DEBUG ((EFI_D_ERROR, "Cannot create dm-crypt device %i\n", -retval));
        goto errout;
    }

    /* Get the device status, in particular, the name of it's device file */
    ioctl_init(io, DM_CRYPT_BUF_SIZE, name, 0);
    if (lkl_sys_ioctl(fd, LKL_DM_DEV_STATUS, (UINTN)io)) {
        DEBUG ((EFI_D_ERROR, "Cannot retrieve dm-crypt device status\n"));
        goto errout;
    }

    minor = (io->dev & 0xff) | ((io->dev >> 12) & 0xfff00);
    AsciiSPrint(crypto_blk_name, LKL_PATH_MAX, "/dev/block/dm-%u", minor);
    retval = lkl_sys_mknod(crypto_blk_name, LKL_S_IFBLK | 0600, io->dev);
    if (retval && retval != -LKL_EEXIST) {
        DEBUG ((EFI_D_ERROR, "Cannot create dm-crypt block device node %d\n", retval));
        goto errout;
    }

    extra_params = "";

    load_count = load_crypto_mapping_table(crypt_ftr, master_key, real_blk_name, name,
                                           fd, extra_params);
    if (load_count < 0) {
        DEBUG ((EFI_D_ERROR, "Cannot load dm-crypt mapping table.\n"));
        goto errout;
    } else if (load_count > 1) {
        DEBUG ((EFI_D_ERROR, "Took %d tries to load dmcrypt table.\n", load_count));
    }

    /* Resume this device to activate it */
    ioctl_init(io, DM_CRYPT_BUF_SIZE, name, 0);

    if (lkl_sys_ioctl(fd, LKL_DM_DEV_SUSPEND, (UINTN)io)) {
        DEBUG ((EFI_D_ERROR, "Cannot resume the dm-crypt device\n"));
        goto errout;
    }

    /* We made it here with no errors.  Woot! */
    retval = 0;

errout:
    lkl_sys_close(fd);   /* If fd is <0 from a failed open call, it's safe to just ignore the close error */

    return retval;
}

/*
 * Called by vold when it's asked to mount an encrypted external
 * storage volume. The incoming partition has no crypto header/footer,
 * as any metadata is been stored in a separate, small partition.
 *
 * out_crypto_blkdev must be LKL_PATH_MAX.
 */
int cryptfs_setup_ext_volume(const char *label, const char *real_blkdev,
                             const unsigned char *key, int keysize, char *out_crypto_blkdev)
{
    int rc;
    int fd = lkl_sys_open(real_blkdev, LKL_O_RDONLY|LKL_O_CLOEXEC, 0);
    if (fd < 0) {
        DEBUG ((EFI_D_ERROR, "Failed to open %a: %a", real_blkdev, lkl_strerror(-fd)));
        return -1;
    }

    unsigned long nr_sec = 0;
    nr_sec = get_blkdev_size(fd, &rc);
    lkl_sys_close(fd);

    if (nr_sec == 0) {
        DEBUG ((EFI_D_ERROR, "Failed to get size of %a: %a", real_blkdev, lkl_strerror(rc)));
        return -1;
    }

    struct crypt_mnt_ftr ext_crypt_ftr;
    SetMem(&ext_crypt_ftr, sizeof(ext_crypt_ftr), 0);
    ext_crypt_ftr.fs_size = nr_sec;
    ext_crypt_ftr.keysize = keysize;
    AsciiStrCpyS((char *) ext_crypt_ftr.crypto_type_name, MAX_CRYPTO_TYPE_NAME_LEN, "aes-cbc-essiv:sha256");

    return create_crypto_blk_dev(&ext_crypt_ftr, key, real_blkdev,
                                 out_crypto_blkdev, label);
}

int delete_crypto_blk_dev(char *name)
{
    int fd;
    char buffer[DM_CRYPT_BUF_SIZE];
    struct lkl_dm_ioctl *io;
    int retval = -1;

    if ((fd = lkl_sys_open(DEVMAPPER_CONTROL_FILE, LKL_O_RDWR, 0)) < 0 ) {
        DEBUG ((EFI_D_ERROR, "Cannot open device-mapper\n"));
        goto errout;
    }

    io = (struct lkl_dm_ioctl *) buffer;

    ioctl_init(io, DM_CRYPT_BUF_SIZE, name, 0);
    if (lkl_sys_ioctl(fd, LKL_DM_DEV_REMOVE, (UINTN)io)) {
        DEBUG ((EFI_D_ERROR, "Cannot remove dm-crypt device\n"));
        goto errout;
    }

    /* We made it here with no errors.  Woot! */
    retval = 0;

errout:
    lkl_sys_close(fd);    /* If fd is <0 from a failed open call, it's safe to just ignore the close error */

    return retval;

}

/*
 * Called by vold when it's asked to unmount an encrypted external
 * storage volume.
 */
int cryptfs_revert_ext_volume(const char *label)
{
    return delete_crypto_blk_dev((char *) label);
}
