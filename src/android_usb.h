#ifndef __ANDROID_USB_H
#define __ANDROID_USB_H
#include <hamlib/rig.h>

int usb_init(hamlib_port_t *p);
int usb_open(hamlib_port_t *p);
int usb_close(hamlib_port_t *p);
int usb_setup(hamlib_port_t *p);
int usb_set_rts(hamlib_port_t *p, int state);
int usb_get_rts(hamlib_port_t *p, int *state);
int usb_set_dtr(hamlib_port_t *p, int state);
int usb_get_dtr(hamlib_port_t *p, int *state);
#endif
