#include "LKLTS.h"

#include <fcntl.h>
#include <errno.h>

#define DEV_INPUT_EVENT "/dev/input"
#define EVENT_DEV_NAME "event"

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define BITS_PER_BYTE           8
#define BITS_PER_LONG           (sizeof(long) * BITS_PER_BYTE)
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))

STATIC UEFI_THREAD_PROTOCOL *mThreads = NULL;
STATIC THREAD mPollThread = NULL;

static int is_event_device(const struct lkl_linux_dirent64 *dir) {
  return AsciiStrnCmp(EVENT_DEV_NAME, dir->d_name, 5) == 0;
}

static char* scan_devices(void)
{
  int rc;
  struct lkl_dir *dir = NULL;
  struct lkl_linux_dirent64 *dirent;
  int i, ndev;
  char *filename = NULL;
  int have_touchscreen = 0;
  long propbit[BITS_TO_LONGS(LKL_INPUT_PROP_MAX)] = {0};

  dir = lkl_opendir(DEV_INPUT_EVENT, &rc);
  if (!dir) {
    return NULL;
  }

  while ((dirent = lkl_readdir(dir))) {
    char fname[64];
    int fd = -1;

    if (!is_event_device(dirent))
      continue;

    AsciiSPrint(fname, sizeof(fname), "%a/%a", DEV_INPUT_EVENT, dirent->d_name);
    fd = open(fname, O_RDONLY);
    if (fd < 0) {
      DEBUG((DEBUG_ERROR, "can't open %a: %d\n", fname, fd));
      continue;
    }

    if ((ioctl(fd, LKL_EVIOCGPROP(sizeof(propbit)), propbit) < 0) ||
      !(propbit[BIT_WORD(LKL_INPUT_PROP_DIRECT)] &
          BIT_MASK(LKL_INPUT_PROP_DIRECT)) ) {
      DEBUG((DEBUG_INFO, "%a is not a touchscreen device\n", fname));
    } else {
      have_touchscreen = 1;
    }

    close(fd);

    if (have_touchscreen) {
      UINTN filename_sz = AsciiStrLen(DEV_INPUT_EVENT) + AsciiStrLen(EVENT_DEV_NAME) + 3;

      filename = AllocatePool(filename_sz);
      if (!filename)
        return NULL;

      AsciiSPrint(filename, filename_sz, "%a/%a%d", DEV_INPUT_EVENT, EVENT_DEV_NAME, i);
      break;
    }
  }

  lkl_closedir(dir);

  return filename;
}

static struct tsdev *lkl_ts_setup(int nonblock)
{
  struct tsdev *ts = NULL;
  char *fname = NULL;
  int rc;

  fname = scan_devices();
  if (!fname) {
    DEBUG((DEBUG_ERROR, "Can't find any touchscreen devices\n"));
    return NULL;
  }

  ts = ts_open(fname, nonblock);
  FreePool(fname);
  if (!ts) {
    DEBUG((DEBUG_ERROR, "ts_config: %a\n", lkl_strerror(errno)));
    ts_close(ts);
    return NULL;
  }

  /* if detected try to configure it */
  rc = ts_load_module_raw(ts, "input", "");
  if (rc != 0) {
    DEBUG((DEBUG_ERROR, "Can't init input module: %d\n", rc));
    ts_close(ts);
    return NULL;
  }

  return ts;
}

STATIC INTN poll_thread_routine(VOID *arg) {
  TSLIB_ABSOLUTE_POINTER_DEV  *MouseAbsolutePointerDev = arg;
  EFI_TPL OldTpl;
  struct tsdev *ts = MouseAbsolutePointerDev->ts;
  int read_samples = 1;
  struct ts_sample_mt **samp_mt = NULL;
  short raw = 0;
  int ret, i, j;
  INT32 max_slots = 1;
  struct lkl_input_absinfo slot;
  int m_x = 0, m_y = 0, m_pressure = 0;

  if (lkl_sys_ioctl(ts_fd(ts), LKL_EVIOCGABS(LKL_ABS_MT_SLOT), (unsigned long)&slot) < 0) {
    ts_close(ts);
    return -1;
  }

  max_slots = slot.maximum + 1 - slot.minimum;

  samp_mt = AllocatePool(read_samples * sizeof(struct ts_sample_mt *));
  if (!samp_mt) {
    ts_close(ts);
    return -1;
  }

  for (i = 0; i < read_samples; i++) {
    samp_mt[i] = AllocateZeroPool(max_slots * sizeof(struct ts_sample_mt));
    if (!samp_mt[i]) {
      FreePool(samp_mt);
      ts_close(ts);
      return -1;
    }
  }

  for (;;) {
    if (raw)
      ret = ts_read_raw_mt(ts, samp_mt, max_slots, read_samples);
    else
      ret = ts_read_mt(ts, samp_mt, max_slots, read_samples);
    if (ret < 0) {
      if (ret==-LKL_EAGAIN)
        continue;
      ts_close(ts);
      return -1;
    }

    for (j = 0; j < ret; j++) {
      for (i = 0; i < max_slots; i++) {
        if (samp_mt[j][i].valid != 1)
          continue;

        int pressure = samp_mt[j][i].pressure;
        int x = samp_mt[j][i].x;
        int y = samp_mt[j][i].y;

        // work around missing coordinates on mouse release
        if (pressure == 0 && x == 0 && y == 0) {
            x = m_x;
            y = m_y;
        }

        if (!raw) {
            //filtering: ignore movements of 2 pixels or less
            int dx = x - m_x;
            int dy = y - m_y;
            if (dx*dx <= 4 && dy*dy <= 4 && (pressure>0) == (m_pressure>0))
                continue;
        }

        OldTpl = gBS->RaiseTPL (TPL_NOTIFY);
        MouseAbsolutePointerDev->State.CurrentX = x;
        MouseAbsolutePointerDev->State.CurrentY = y;
        MouseAbsolutePointerDev->State.CurrentZ = pressure;
        MouseAbsolutePointerDev->StateChanged = TRUE;
        gBS->RestoreTPL (OldTpl);

        m_x = x;
        m_y = y;
        m_pressure = pressure;
      }
    }
  }

  ts_close(ts);

  return 0;
}

STATIC
EFI_STATUS
EFIAPI
MouseAbsolutePointerReset (
  IN EFI_ABSOLUTE_POINTER_PROTOCOL    *This,
  IN BOOLEAN                          ExtendedVerification
  )
{
  TSLIB_ABSOLUTE_POINTER_DEV       *MouseAbsolutePointerDev;

  MouseAbsolutePointerDev = TSLIB_ABSOLUTE_POINTER_DEV_FROM_THIS (This);

  ZeroMem (&MouseAbsolutePointerDev->State, sizeof (EFI_ABSOLUTE_POINTER_STATE));
  MouseAbsolutePointerDev->StateChanged = FALSE;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
MouseAbsolutePointerGetState (
  IN EFI_ABSOLUTE_POINTER_PROTOCOL     *This,
  IN OUT EFI_ABSOLUTE_POINTER_STATE    *State
  )
{
  TSLIB_ABSOLUTE_POINTER_DEV *MouseAbsolutePointerDev;
  EFI_TPL       OldTpl;

  MouseAbsolutePointerDev = TSLIB_ABSOLUTE_POINTER_DEV_FROM_THIS (This);

  if (State == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (!MouseAbsolutePointerDev->StateChanged) {
    return EFI_NOT_READY;
  }

  OldTpl = gBS->RaiseTPL (TPL_NOTIFY);
  CopyMem (State, &(MouseAbsolutePointerDev->State), sizeof (EFI_ABSOLUTE_POINTER_STATE));

  //
  // clear mouse state
  //
  MouseAbsolutePointerDev->State.CurrentX = 0;
  MouseAbsolutePointerDev->State.CurrentY = 0;
  MouseAbsolutePointerDev->State.CurrentZ = 0;
  MouseAbsolutePointerDev->State.ActiveButtons = 0x0;
  MouseAbsolutePointerDev->StateChanged        = FALSE;

  gBS->RestoreTPL (OldTpl);

  return EFI_SUCCESS;
}

STATIC
VOID
EFIAPI
MouseAbsolutePointerWaitForInput (
  IN  EFI_EVENT               Event,
  IN  VOID                    *Context
  )
{
  TSLIB_ABSOLUTE_POINTER_DEV *MouseAbsolutePointerDev;

  MouseAbsolutePointerDev = (TSLIB_ABSOLUTE_POINTER_DEV *) Context;

  //
  // Someone is waiting on the mouse event, if there's
  // input from mouse, signal the event
  //
  if (MouseAbsolutePointerDev->StateChanged) {
    gBS->SignalEvent (Event);
  }

}

STATIC
EFI_STATUS
CreateDevInput(VOID)
{
  struct lkl_dir *dir = NULL;
  struct lkl_linux_dirent64 *dirent;
  UINT32 dev;
  int rc;
  char pathbuf[1024];

  lkl_sys_mkdir("/dev", 0700);
  lkl_sys_mkdir("/dev/input", 0700);

  lkl_mount_fs("sysfs");
  dir = lkl_opendir("/sysfs/class/input", &rc);
  if (!dir) {
    DEBUG((EFI_D_ERROR, "Can't open sysfs input dir: %d\n", rc));
    return EFI_DEVICE_ERROR;
  }

  while ((dirent = lkl_readdir(dir))) {
    rc = AsciiSPrint(pathbuf, ARRAY_SIZE(pathbuf), "/sysfs/class/input/%a/dev", dirent->d_name);
    if (rc < 0 || rc>=ARRAY_SIZE(pathbuf))
      continue;

    // get dev node id's
    rc = lkl_encode_dev_from_sysfs(pathbuf, &dev);
    if (rc) {
      continue;
    }

    rc = AsciiSPrint(pathbuf, ARRAY_SIZE(pathbuf), "/dev/input/%a", dirent->d_name);
    if (rc < 0 || rc>=ARRAY_SIZE(pathbuf))
      continue;

    // create dev node
    rc = lkl_sys_mknod(pathbuf, LKL_S_IFCHR | 0600, dev);
    if (rc) {
      DEBUG((EFI_D_ERROR, "Can't create input node at %a: %d\n", pathbuf, rc));
      continue;
    }
  }

  lkl_closedir(dir);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
LKLTSEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS Status;
  INTN rc;
  struct tsdev *ts;
  TSLIB_ABSOLUTE_POINTER_DEV  *MouseAbsolutePointerDev;
  EFI_HANDLE    Handle = NULL;
  int abs_x[6] = {0};
  int abs_y[6] = {0};

  ts = NULL;
  MouseAbsolutePointerDev = NULL;

  Status = gBS->LocateProtocol (&gUefiThreadProtocolGuid, NULL, (VOID **)&mThreads);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  CreateDevInput();

  // initialize tslib
  ts = lkl_ts_setup(0);
  if (!ts) {
    DEBUG((EFI_D_ERROR, "Can't setup tslib\n"));
    return EFI_DEVICE_ERROR;
  }

  rc = lkl_sys_ioctl(ts_fd(ts), LKL_EVIOCGABS(LKL_ABS_MT_POSITION_X), (unsigned long)&abs_x);
  if (rc < 0) {
    DEBUG((EFI_D_ERROR, "Can't get LKL_ABS_MT_POSITION_X: %d\n", rc));
    Status = EFI_DEVICE_ERROR;
    goto ErrorExit;
  }

  rc = lkl_sys_ioctl(ts_fd(ts), LKL_EVIOCGABS(LKL_ABS_MT_POSITION_Y), (unsigned long)&abs_y);
  if (rc < 0) {
    DEBUG((EFI_D_ERROR, "Can't get LKL_ABS_MT_POSITION_Y: %d\n", rc));
    Status = EFI_DEVICE_ERROR;
    goto ErrorExit;
  }

  //
  // Allocate private data
  //
  MouseAbsolutePointerDev = AllocateZeroPool (sizeof (TSLIB_ABSOLUTE_POINTER_DEV));
  if (MouseAbsolutePointerDev == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  //
  // Setup the device instance
  //

  MouseAbsolutePointerDev->Signature       = TSLIB_ABSOLUTE_POINTER_DEV_SIGNATURE;
  MouseAbsolutePointerDev->ts              = ts;

  MouseAbsolutePointerDev->Mode.AbsoluteMaxX               = abs_x[2];
  MouseAbsolutePointerDev->Mode.AbsoluteMinX               = abs_x[1];
  MouseAbsolutePointerDev->Mode.AbsoluteMaxY               = abs_y[2];
  MouseAbsolutePointerDev->Mode.AbsoluteMinY               = abs_y[1];
  MouseAbsolutePointerDev->Mode.AbsoluteMaxZ               = 255ULL;
  MouseAbsolutePointerDev->Mode.AbsoluteMinZ               = 0ULL;
  MouseAbsolutePointerDev->Mode.Attributes                 = EFI_ABSP_SupportsPressureAsZ;

  MouseAbsolutePointerDev->AbsolutePointerProtocol.Reset     = MouseAbsolutePointerReset;
  MouseAbsolutePointerDev->AbsolutePointerProtocol.GetState  = MouseAbsolutePointerGetState;
  MouseAbsolutePointerDev->AbsolutePointerProtocol.Mode      = &(MouseAbsolutePointerDev->Mode);

  //
  // Setup the WaitForKey event
  //
  Status = gBS->CreateEvent (
                  EVT_NOTIFY_WAIT,
                  TPL_NOTIFY,
                  MouseAbsolutePointerWaitForInput,
                  MouseAbsolutePointerDev,
                  &((MouseAbsolutePointerDev->AbsolutePointerProtocol).WaitForInput)
                  );
  if (EFI_ERROR (Status)) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  //
  // Install protocol interfaces for the touchscreen device.
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiAbsolutePointerProtocolGuid,
                  &MouseAbsolutePointerDev->AbsolutePointerProtocol,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  // start tslib thread
  mThreads->ThreadCreate(&mPollThread, "tslib", poll_thread_routine, MouseAbsolutePointerDev, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
  mThreads->ThreadDetach(mPollThread);
  mThreads->ThreadResume(mPollThread);

  return EFI_SUCCESS;

ErrorExit:
  if ((MouseAbsolutePointerDev != NULL) && (MouseAbsolutePointerDev->AbsolutePointerProtocol.WaitForInput != NULL)) {
    gBS->CloseEvent (MouseAbsolutePointerDev->AbsolutePointerProtocol.WaitForInput);
  }
  if (MouseAbsolutePointerDev)
    FreePool (MouseAbsolutePointerDev);
  if (ts)
    ts_close (ts);

  return Status;
}

EFI_STATUS
EFIAPI
LKLTSUnload (
  IN EFI_HANDLE  ImageHandle
  )
{
  return EFI_SUCCESS;
}
