#ifndef _LINUX_INPUT_H
#define _LINUX_INPUT_H

#include <lkl/linux/input.h>

struct input_event {
	struct lkl_timeval time;
	uint16_t type;
	uint16_t code;
	INT32 value;
};

struct input_id {
	uint16_t bustype;
	uint16_t vendor;
	uint16_t product;
	uint16_t version;
};

#define EVIOCGID LKL_EVIOCGID
#define EVIOCGVERSION LKL_EVIOCGVERSION
#define EV_VERSION LKL_EV_VERSION
#define EVIOCGBIT LKL_EVIOCGBIT
#define EVIOCGRAB LKL_EVIOCGRAB

#define EV_MAX LKL_EV_MAX
#define ABS_MAX LKL_ABS_MAX
#define KEY_MAX LKL_KEY_MAX
#define EV_ABS LKL_EV_ABS
#define EV_KEY LKL_EV_KEY
#define ABS_X LKL_ABS_X
#define ABS_Y LKL_ABS_Y
#define ABS_PRESSURE LKL_ABS_PRESSURE
#define BTN_TOUCH LKL_BTN_TOUCH
#define BTN_LEFT LKL_BTN_LEFT
#define SYN_REPORT LKL_SYN_REPORT

#endif
