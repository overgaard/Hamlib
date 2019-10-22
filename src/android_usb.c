#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <hamlib/rig.h>
#include "pl2303.h"

#define BIT(nr) (1UL << (nr))

#define PL2303_QUIRK_UART_STATE_IDX0		BIT(0)
#define PL2303_QUIRK_LEGACY			BIT(1)
#define PL2303_QUIRK_ENDPOINT_HACK		BIT(2)

#define USB_DEVICE(vend, prod) \
	.idVendor = (vend), \
	.idProduct = (prod)

static libusb_device_handle *handle;
static libusb_device *dev;
static bool kernel_driver_detached = false;
static bool found = false;
static int controlLineValue = 0;

struct usb_device_id {
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t driver_info;
} usb_device_id;

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_RSAQ2) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_DCU11) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_RSAQ3) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_CHILITAG) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_PHAROS) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_ALDIGA) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_MMX) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_GPRS) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_HCR331) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_MOTOROLA) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_ZTEK) },
	{ USB_DEVICE(PL2303_VENDOR_ID, PL2303_PRODUCT_ID_TB) },
	{ USB_DEVICE(IODATA_VENDOR_ID, IODATA_PRODUCT_ID) },
	{ USB_DEVICE(IODATA_VENDOR_ID, IODATA_PRODUCT_ID_RSAQ5) },
	{ USB_DEVICE(ATEN_VENDOR_ID, ATEN_PRODUCT_ID), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(ATEN_VENDOR_ID, ATEN_PRODUCT_UC485), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(ATEN_VENDOR_ID, ATEN_PRODUCT_UC232B), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(ATEN_VENDOR_ID, ATEN_PRODUCT_ID2) },
	{ USB_DEVICE(ATEN_VENDOR_ID2, ATEN_PRODUCT_ID) },
	{ USB_DEVICE(ELCOM_VENDOR_ID, ELCOM_PRODUCT_ID) },
	{ USB_DEVICE(ELCOM_VENDOR_ID, ELCOM_PRODUCT_ID_UCSGT) },
	{ USB_DEVICE(ITEGNO_VENDOR_ID, ITEGNO_PRODUCT_ID) },
	{ USB_DEVICE(ITEGNO_VENDOR_ID, ITEGNO_PRODUCT_ID_2080) },
	{ USB_DEVICE(MA620_VENDOR_ID, MA620_PRODUCT_ID) },
	{ USB_DEVICE(RATOC_VENDOR_ID, RATOC_PRODUCT_ID) },
	{ USB_DEVICE(TRIPP_VENDOR_ID, TRIPP_PRODUCT_ID) },
	{ USB_DEVICE(RADIOSHACK_VENDOR_ID, RADIOSHACK_PRODUCT_ID) },
	{ USB_DEVICE(DCU10_VENDOR_ID, DCU10_PRODUCT_ID) },
	{ USB_DEVICE(SITECOM_VENDOR_ID, SITECOM_PRODUCT_ID) },
	{ USB_DEVICE(ALCATEL_VENDOR_ID, ALCATEL_PRODUCT_ID) },
	{ USB_DEVICE(SIEMENS_VENDOR_ID, SIEMENS_PRODUCT_ID_SX1), .driver_info = PL2303_QUIRK_UART_STATE_IDX0 },
	{ USB_DEVICE(SIEMENS_VENDOR_ID, SIEMENS_PRODUCT_ID_X65), .driver_info = PL2303_QUIRK_UART_STATE_IDX0 },
	{ USB_DEVICE(SIEMENS_VENDOR_ID, SIEMENS_PRODUCT_ID_X75), .driver_info = PL2303_QUIRK_UART_STATE_IDX0 },
	{ USB_DEVICE(SIEMENS_VENDOR_ID, SIEMENS_PRODUCT_ID_EF81), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(BENQ_VENDOR_ID, BENQ_PRODUCT_ID_S81) }, /* Benq/Siemens S81 */
	{ USB_DEVICE(SYNTECH_VENDOR_ID, SYNTECH_PRODUCT_ID) },
	{ USB_DEVICE(NOKIA_CA42_VENDOR_ID, NOKIA_CA42_PRODUCT_ID) },
	{ USB_DEVICE(CA_42_CA42_VENDOR_ID, CA_42_CA42_PRODUCT_ID) },
	{ USB_DEVICE(SAGEM_VENDOR_ID, SAGEM_PRODUCT_ID) },
	{ USB_DEVICE(LEADTEK_VENDOR_ID, LEADTEK_9531_PRODUCT_ID) },
	{ USB_DEVICE(SPEEDDRAGON_VENDOR_ID, SPEEDDRAGON_PRODUCT_ID) },
	{ USB_DEVICE(DATAPILOT_U2_VENDOR_ID, DATAPILOT_U2_PRODUCT_ID) },
	{ USB_DEVICE(BELKIN_VENDOR_ID, BELKIN_PRODUCT_ID) },
	{ USB_DEVICE(ALCOR_VENDOR_ID, ALCOR_PRODUCT_ID), .driver_info = PL2303_QUIRK_ENDPOINT_HACK },
	{ USB_DEVICE(WS002IN_VENDOR_ID, WS002IN_PRODUCT_ID) },
	{ USB_DEVICE(COREGA_VENDOR_ID, COREGA_PRODUCT_ID) },
	{ USB_DEVICE(YCCABLE_VENDOR_ID, YCCABLE_PRODUCT_ID) },
	{ USB_DEVICE(SUPERIAL_VENDOR_ID, SUPERIAL_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LD220_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LD220TA_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LD960_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LD960TA_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LCM220_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LCM960_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LM920_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_LM940_PRODUCT_ID) },
	{ USB_DEVICE(HP_VENDOR_ID, HP_TD620_PRODUCT_ID) },
	{ USB_DEVICE(CRESSI_VENDOR_ID, CRESSI_EDY_PRODUCT_ID) },
	{ USB_DEVICE(ZEAGLE_VENDOR_ID, ZEAGLE_N2ITION3_PRODUCT_ID) },
	{ USB_DEVICE(SONY_VENDOR_ID, SONY_QN3USB_PRODUCT_ID) },
	{ USB_DEVICE(SANWA_VENDOR_ID, SANWA_PRODUCT_ID) },
	{ USB_DEVICE(ADLINK_VENDOR_ID, ADLINK_ND6530_PRODUCT_ID) },
	{ USB_DEVICE(SMART_VENDOR_ID, SMART_PRODUCT_ID) },
	{ USB_DEVICE(AT_VENDOR_ID, AT_VTKIT3_PRODUCT_ID) },
	{ 0, 0, 0}					/* Terminating entry */
};


#define PL2303_VENDOR_WRITE_REQUEST			0x01
#define PL2303_VENDOR_WRITE_REQUEST_TYPE	0x40
#define PL2303_VENDOR_READ_REQUEST_TYPE		0xc0
#define PL2303_VENDOR_READ_REQUEST			0x01
#define PL2303_GET_LINE_REQUEST_TYPE		0xa1
#define PL2303_GET_LINE_REQUEST				0x21
#define PL2303_SET_LINE_REQUEST_TYPE		0x21
#define PL2303_SET_LINE_REQUEST				0x22
#define PL2303_SET_CONROL_REQUEST_TYPE		0x21
#define PL2303_SET_CONTROL_REQUEST			0x22

#define PL2303_CONTROL_DTR					0x01
#define PL2303_CONTROL_RTS					0x02

static int is_interresting(libusb_device *device)
{
		struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        const struct usb_device_id *device_id_table = &id_table[0];
        
        if (r != 0) {
                rig_debug(RIG_DEBUG_VERBOSE, "ANDROID(in %s) failed to get device descriptor\n", __func__);
                return -RIG_EIO;
        }
        
        rig_debug(RIG_DEBUG_VERBOSE, "ANDROID(in %s) Searching for %04x:%04x ", __func__, desc.idVendor, desc.idProduct);
        while (device_id_table->idVendor != 0) {
				if ((desc.idVendor == device_id_table->idVendor) & (desc.idProduct == device_id_table->idProduct)) {
						rig_debug(RIG_DEBUG_VERBOSE, "FOUND!\n");
						return RIG_OK;
				}
				device_id_table++;
		}
		rig_debug(RIG_DEBUG_VERBOSE, "NOT FOUND!\n");
		return -RIG_EIO; // Do not support the device
}

static int getMantissaExponentFromBaudRate(uint32_t baudrate, uint32_t *mantissa, uint32_t *exponent)
{
	int32_t baseline = 12000000 * 32;

	*mantissa = baseline / baudrate;
	if (*mantissa == 0)
		*mantissa = 1;	/* Avoid dividing by zero if baud > 32*12M. */
	*exponent = 0;
	while (*mantissa >= 512) {
		if (*exponent < 7) {
			(*mantissa) >>= 2;	/* divide by 4 */
			(*exponent)++;
		} else {
			/* Exponent is maxed. Trim mantissa and leave. */
			*mantissa = 511;
			break;
		}
	}

	return (baseline / *mantissa) >> ((*exponent) << 1);
}

static int vendorRead(libusb_device_handle *device, uint16_t value, unsigned char buf[1])
{
	int res;
	res = libusb_control_transfer(device, PL2303_VENDOR_READ_REQUEST_TYPE, PL2303_VENDOR_READ_REQUEST,
		value, 0, buf, 1, 100);
	rig_debug(RIG_DEBUG_ERR, "vendorRead - result: %d, value: %d\n", res, value);
	rig_debug(RIG_DEBUG_ERR, "vendorRead - result: %d, value: %d\n", res, value);
	return res;
}

static int vendorWrite(libusb_device_handle *device, uint16_t value, uint16_t index)
{
	int res;

	res = libusb_control_transfer(device, PL2303_VENDOR_WRITE_REQUEST_TYPE, PL2303_VENDOR_WRITE_REQUEST,
		value, index, NULL, 0, 100);
	rig_debug(RIG_DEBUG_ERR, "vendorWrite - result: %d, value: %d, index: %d\n", res, value, index);
	return res;		 
}

static int getLineRequest(libusb_device_handle *device, unsigned char *data)
{
	int res;

	res = libusb_control_transfer(device, PL2303_GET_LINE_REQUEST_TYPE, PL2303_GET_LINE_REQUEST,
		0, 0, data, 7, 100);
	rig_debug(RIG_DEBUG_ERR, "getLineRequest - result: %d\n", res);
	
	if (res == 7)
		return RIG_OK;
	else
		return RIG_EIO;
}

static int setLineRequest(libusb_device_handle *device, unsigned char *data)
{
	int res;

	res = libusb_control_transfer(device, PL2303_SET_LINE_REQUEST_TYPE, PL2303_SET_LINE_REQUEST,
		0, 0, data, 7, 100);
	rig_debug(RIG_DEBUG_ERR, "setLineRequest - result: %d\n", res);
	
	if (res == 7)
		return RIG_OK;
	else
		return RIG_EIO;
}

static int setControlLines(libusb_device_handle *device, int value)
{	
	int res;
	
	res = libusb_control_transfer(device, PL2303_SET_CONROL_REQUEST_TYPE, PL2303_SET_CONTROL_REQUEST,
		value, 0, NULL, 0, 100);
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) value: %d res: %d\n", __func__, value, res);
	return res;
}

static void resetDataPipes(libusb_device_handle *device)
{
	vendorWrite(device, 8, 0);
	vendorWrite(device, 9, 0);
}

int usb_init(hamlib_port_t *p)
{
	int res;
	int cnt;
	libusb_device **devs;
	int i = 0;
	
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) USB init starting!\n", __func__);
	// Initialize the library
	res = libusb_init(NULL);
	if (res < 0) {
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Can't initialize libusb!\n", __func__);
		return RIG_EIO;
	}
	
	// Scan for supported devices
	found = false;
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Scanning for supported devices\n", __func__);
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0) {
		libusb_exit(NULL);
		return -RIG_EIO;
	}
	
	while ((dev = devs[i++]) != NULL) {
		if (is_interresting(dev) == RIG_OK) {
			found = true;
			break;
		}
	}
	libusb_free_device_list(devs, 1);
	
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) USB init done!\n", __func__);
	return RIG_OK;
}

int usb_setup(hamlib_port_t *p)
{
	int res;
	unsigned char tmp[8];
	uint32_t mantissa, exponent;
	uint32_t baudrate;
	
	res = getLineRequest(handle, tmp);
	if (res != RIG_OK) {
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Couldn't getLineRequest\n", __func__);
		return res;
	}
	
	baudrate = getMantissaExponentFromBaudRate(p->parm.serial.rate,
			&mantissa, &exponent);
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Baudrate requested %d, and got %d\n", 
		__func__, p->parm.serial.rate, baudrate);
	
	/* Databits */
	tmp[6] = p->parm.serial.data_bits;
	
	/* Parity */
	tmp[5] = p->parm.serial.parity;
	
	/* Stop bits */
	tmp[4] = p->parm.serial.parity;
	
	/* Baudrate */
	tmp[3] = 0x80;
	tmp[2] = 0x00;
	tmp[1] = exponent << 1 | mantissa >> 8;
	tmp[0] = mantissa & 0xff;
	
	res = setLineRequest(handle, tmp);
	if (res != RIG_OK) {
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Couldn't setLineRequest\n", __func__);
		return res;
	}
		
	return RIG_OK;
}

int usb_open(hamlib_port_t *p)
{
	int res;
	unsigned char tmp[8];
	
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) USB open begin!\n", __func__);
	/* Open the USB device using the new libusb_open2 function */
	res = libusb_open2(dev, &handle, p->fd);
	if (res != 0) {
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Can't open USB device!\n", __func__);
		return -RIG_EIO;
	}
	
	/* Check if kernel driver is attached. If so detach it */
	if (libusb_kernel_driver_active(handle, 0)) {
		rig_debug(RIG_DEBUG_ERR, "Kernel driver is attaced! Detaching...");
		res = libusb_detach_kernel_driver(handle, 0);
		if (res != 0) {
			rig_debug(RIG_DEBUG_ERR, " not detached!\n");
			return -RIG_EIO;
		}
		kernel_driver_detached = true;
		rig_debug(RIG_DEBUG_ERR, " detached!\n");
	}
	
	/* Now claim the interface */
	res = libusb_claim_interface(handle, 0);
	if (res != 0) {
		rig_debug(RIG_DEBUG_ERR, "Couldn't claim interface!\n");
		/* Re-attach kernel driver if it was detached */	
		if (kernel_driver_detached) {
			libusb_attach_kernel_driver(handle, 0);
			rig_debug(RIG_DEBUG_ERR, "Kernel driver attached!\n");
		}
		return -RIG_EIO;
	}
	
	/* Do some black magic */
	vendorRead(handle, 0x8484, tmp);
	vendorWrite(handle, 0x0404, 0);
	vendorRead(handle, 0x8484, tmp);
	vendorRead(handle, 0x8383, tmp);
	vendorRead(handle, 0x8484, tmp);
	vendorWrite(handle, 0x0404, 1);
	vendorRead(handle, 0x8484, tmp);
	vendorRead(handle, 0x8383, tmp);
	vendorWrite(handle, 0, 1);
	vendorWrite(handle, 1, 0);

	vendorWrite(handle, 2, 0x24);
	
	resetDataPipes(handle);
	
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) USB open done!\n", __func__);
	
	return RIG_OK;
}

int usb_close(hamlib_port_t *p)
{
	int res;
	
	rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s)\n", __func__);
	/* Release the claimed interface */
	res = libusb_release_interface(handle, 0);
	if (res != 0) {
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Couldn't release interface!\n", __func__);
	}

	/* Re-attach kernel driver if it was detached */	
	if (kernel_driver_detached) {
		libusb_attach_kernel_driver(handle, 0);
		rig_debug(RIG_DEBUG_ERR, "ANDROID(in %s) Kernel driver attaced!\n", __func__);
	}

	libusb_close(handle);
	libusb_exit(NULL);

	return RIG_OK;
}

int usb_set_rts(hamlib_port_t *p, int state)
{

		return RIG_OK;
}

int usb_get_rts(hamlib_port_t *p, int *state)
{
	
}

int usb_set_dtr(hamlib_port_t *p, int state)
{
	
}

int usb_get_dtr(hamlib_port_t *p, int *state)
{
	
}

