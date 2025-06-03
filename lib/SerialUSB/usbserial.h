#ifndef __USBSERIAL_H__
#define __USBSERIAL_H__

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/usbd.h>
#include <stdlib.h>

#ifndef MAX_BUFFER_USB_RX
#define MAX_BUFFER_USB_RX 256UL ///< @brief Tamaño del buffer de recepción del Puerto Serie Virtual.
#endif // MAX_BUFFER_USB_RX
#ifndef MAX_BUFFER_USB_TX
#define MAX_BUFFER_USB_TX 256UL ///< @brief Tamaño del buffer de transmisión del Puerto Serie Virtual.
#endif // MAX_BUFFER_USB_TX

#if MAX_BUFFER_USB_TX < 2
#error MAX_BUFFER_USB_TX is too small.  It must be larger than 1.
#elif ((MAX_BUFFER_USB_TX & (MAX_BUFFER_USB_TX - 1)) != 0)
#error MAX_BUFFER_USB_TX must be a power of 2.
#endif
#if MAX_BUFFER_USB_RX < 2
#error MAX_BUFFER_USB_RX is too small.  It must be larger than 1.
#elif ((MAX_BUFFER_USB_RX & (MAX_BUFFER_USB_RX - 1)) != 0)
#error MAX_BUFFER_USB_RX must be a power of 2.
#endif

/* USB Device information for the host, must be valid CDC VID/PID  */
#define ID_VENDOR   0x16C0  // Van Ooijen Technische Informatica
#define ID_PRODUCT  0x05E1  // Free shared USB VID/PID pair for CDC devices
#define BCD_DEVICE  0x0200  // v2.0

/* Developer information of the device */
#define USB_MANUFAC "Prof. Tute Avalos"
#define USB_PRODUCT "CDC-ACM Virtual Serial Port"
#define USB_SERIALN "000001"

BEGIN_DECLS

void usb_serial_begin(void);

void usb_serial_poll(void);

uint16_t usb_serial_available(void);

uint8_t usb_serial_read(void);

uint16_t usb_serial_sendable(void);

bool usb_serial_write(const uint8_t data);

uint16_t usb_serial_puts(const char *str);

uint16_t usb_serial_send_data(const void *data, uint32_t size);

END_DECLS

#endif // __USBSERIAL_H__
