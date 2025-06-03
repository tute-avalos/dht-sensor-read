#include "usbserial.h"
#include <stdarg.h>
#include <sys/errno.h>
#include <sys/unistd.h>

typedef struct __packet {
  uint16_t in;
  uint16_t out;
  volatile uint8_t *data;
  uint16_t mask;
} circular_buffer_t;

// Buffers -------------------------------------------------------)
volatile uint8_t
    buffer_tx[MAX_BUFFER_USB_TX]; ///< @brief Buffer de Transmision.
volatile uint8_t buffer_rx[MAX_BUFFER_USB_RX]; ///< @brief Buffer de Recepcion.
/**
 * @brief Estructura que contiene los índices y puntero al buffer de transmisión
 * de la USART1.
 */
static circular_buffer_t cbuff_tx = {
    .in = 0, .out = 0, .data = buffer_tx, .mask = MAX_BUFFER_USB_TX - 1};
/**
 * @brief Estructura que contiene los índices y puntero al buffer de recepción
 * de la USART1.
 */
static circular_buffer_t cbuff_rx = {
    .in = 0, .out = 0, .data = buffer_rx, .mask = MAX_BUFFER_USB_RX - 1};

static bool push(circular_buffer_t *buff, const uint8_t data) {
  bool result = false;
  if ((uint16_t)(buff->in - buff->out) < buff->mask) {
    result = true;
    buff->data[buff->in & buff->mask] = data;
    buff->in++;
  }
  return result;
}

static int16_t pop(circular_buffer_t *buff) {
  int16_t ndata = -1;
  if (buff->in != buff->out) {
    ndata = (uint16_t)(buff->data[buff->out & buff->mask]);
    buff->out++;
  }
  return ndata;
}

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = ID_VENDOR,
    .idProduct = ID_PRODUCT,
    .bcdDevice = BCD_DEVICE,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    }};

static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header =
        {
            .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
            .bcdCDC = 0x0110,
        },
    .call_mgmt =
        {
            .bFunctionLength =
                sizeof(struct usb_cdc_call_management_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
            .bmCapabilities = 0,
            .bDataInterface = 1,
        },
    .acm =
        {
            .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_ACM,
            .bmCapabilities = 0,
        },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }};

static const struct usb_interface_descriptor comm_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
     .bNumEndpoints = 1,
     .bInterfaceClass = USB_CLASS_CDC,
     .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
     .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
     .iInterface = 0,
     .endpoint = comm_endp,
     .extra = &cdcacm_functional_descriptors,
     .extralen = sizeof(cdcacm_functional_descriptors)}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
                                                  .num_altsetting = 1,
                                                  .altsetting = comm_iface,
                                              },
                                              {
                                                  .num_altsetting = 1,
                                                  .altsetting = data_iface,
                                              }};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    USB_MANUFAC, // Manufacturer
    USB_PRODUCT, // Product
    USB_SERIALN, // SerialNumber
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[256];

usbd_device *usbd_dev;

static enum usbd_request_return_codes
cdcacm_control_request(usbd_device *device __unused, struct usb_setup_data *req,
                       uint8_t **buf __unused, uint16_t *len,
                       void (**complete)(usbd_device *device,
                                         struct usb_setup_data *req) __unused) {

  switch (req->bRequest) {
  case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
    /*
     * This Linux cdc_acm driver requires this to be implemented
     * even though it's optional in the CDC spec, and we don't
     * advertise it in the ACM functional descriptor.
     */
    char local_buf[10];
    struct usb_cdc_notification *notif = (void *)local_buf;

    /* We echo signals back to host as notification. */
    notif->bmRequestType = 0xA1;
    notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
    notif->wValue = 0;
    notif->wIndex = 0;
    notif->wLength = 2;
    local_buf[8] = req->wValue & 3;
    local_buf[9] = 0;
    // usbd_ep_write_packet(0x83, buf, 10);
    return USBD_REQ_HANDLED;
  }
  case USB_CDC_REQ_SET_LINE_CODING:
    if (*len < sizeof(struct usb_cdc_line_coding)) {
      return USBD_REQ_NOTSUPP;
    }
    return USBD_REQ_HANDLED;
  }
  return 0;
}

static void cdcacm_data_rx_cb(usbd_device *device, uint8_t ep __unused) {
  char buf[64];
  int len = usbd_ep_read_packet(device, 0x01, buf, 64);
  if (len != 0)
    for (int i = 0; i < len; i++)
      push(&cbuff_rx, buf[i]);
}

static void cdcacm_set_config(usbd_device *device, uint16_t wValue) {
  (void)wValue;

  usbd_ep_setup(device, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
  usbd_ep_setup(device, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(device, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(
      device, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);
}

void usb_serial_begin(void) {

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_AFIO);

  AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO12);
  gpio_clear(GPIOA, GPIO12);
  for (unsigned i = 0; i < 7200000; i++)
    __asm__("nop");

  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3,
                       usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
}

void usb_serial_poll(void) {
  usbd_poll(usbd_dev);
  if ((cbuff_tx.in - cbuff_tx.out) != 0) {
    char buff[64];
    int len = 0;
    int16_t c = 0;
    while (len < 64 && ((c = pop(&cbuff_tx)) != -1))
      buff[len++] = (char)c;

    if (len != 0) {
      usbd_ep_write_packet(usbd_dev, 0x82, buff, len);
    }
  }
}

uint16_t usb_serial_available(void) { return (cbuff_rx.in - cbuff_rx.out); }

uint8_t usb_serial_read(void) { return (uint8_t)pop(&cbuff_rx); }

uint16_t usb_serial_sendable(void) {
  return (cbuff_tx.mask - (cbuff_tx.in - cbuff_tx.out)) + 1U;
}

bool usb_serial_write(const uint8_t data) { return push(&cbuff_tx, data); }

uint16_t usb_serial_puts(const char *str) {
  uint32_t nsend = 0;
  while (*str && usb_serial_write(*str++))
    nsend++;

  return nsend;
}

uint16_t usb_serial_send_data(const void *data, uint32_t size) {
  uint32_t nsend = 0;
  uint8_t *ddata = (uint8_t *)data;

  do {
    if (!usb_serial_write(*ddata++))
      break;
    nsend++;
  } while (--size);

  return nsend;
}
