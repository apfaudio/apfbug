/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "dirtyJtagConfig.h"
#include "tusb.h"
#include "version.h"
#include "get_serial.h"

// USB 2.1 to signal BOS descriptor support (needed for WebUSB)
#define USB_BCD   0x0210

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = 0x00, // Each interface specifies its own
    .bDeviceSubClass    = 0x00, // Each interface specifies its own
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor  = 0x1209,
    .idProduct = 0xC0CA,
    .bcdDevice = 0x0111,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_PROBE = 0,
#if ( USB_CDC_UART_BRIDGE )
  ITF_NUM_CDC_1 = 1,
  ITF_NUM_CDC_1_DATA,
#endif 
  ITF_NUM_TOTAL
};

#define PROBE_OUT_EP_NUM 0x01
#define PROBE_IN_EP_NUM  0x82
#if ( USB_CDC_UART_BRIDGE )
#define CDC_NOTIF_EP1_NUM 0x83
#define CDC_OUT_EP1_NUM   0x03
#define CDC_IN_EP1_NUM    0x84
#define CDC_NOTIF_EP2_NUM 0x85
#define CDC_OUT_EP2_NUM   0x05
#define CDC_IN_EP2_NUM    0x86
#endif 

#if ( USB_CDC_UART_BRIDGE )
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN * CFG_TUD_CDC)
#else
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)
#endif

uint8_t const desc_configuration[CONFIG_TOTAL_LEN] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // Interface 2 : Interface number, string index, EP Out & IN address, EP size
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_PROBE, 0, PROBE_OUT_EP_NUM, PROBE_IN_EP_NUM, 64),
#if ( USB_CDC_UART_BRIDGE )
  // Interface 3 : Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, CDC_NOTIF_EP1_NUM, 8, CDC_OUT_EP1_NUM, CDC_IN_EP1_NUM, 64),
#endif
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// BOS Descriptor (WebUSB)
//--------------------------------------------------------------------+

#define WEBUSB_VENDOR_CODE 0xD0

// WebUSB Platform Capability UUID: 3408b638-09a9-47a0-8bfd-a0768815b665
#define TUD_BOS_WEBUSB_DESC_LEN 24
#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN)

uint8_t const desc_bos[] = {
  // BOS header
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

  // WebUSB Platform Capability
  TUD_BOS_WEBUSB_DESC_LEN,           // bLength
  TUSB_DESC_DEVICE_CAPABILITY,        // bDescriptorType
  0x05, // bDevCapabilityType (Platform)
  0x00,                               // bReserved
  // PlatformCapabilityUUID: WebUSB
  0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
  0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
  0x00, 0x01,                         // bcdVersion 1.0
  WEBUSB_VENDOR_CODE,                 // bVendorCode
  0x01,                               // iLandingPage
};

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}

// WebUSB URL descriptor for "https://tiliqua.io"
#define WEBUSB_URL_STR "tiliqua.io"

const uint8_t webusb_url_descriptor[] = {
  3 + sizeof(WEBUSB_URL_STR) - 1,    // bLength
  0x03,                               // bDescriptorType (URL)
  0x01,                               // bScheme (https://)
  't', 'i', 'l', 'i', 'q', 'u', 'a', '.', 'i', 'o',
};

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const *string_desc_arr[] =
{
    (const char[]){0x09, 0x04},   // 0: is supported language is English (0x0409)
    "apf.audio",                  // 1: Manufacturer
#if (TILIQUA_HW_MAJOR == 2)
    "Tiliqua R2 " GIT_VERSION,
#elif (TILIQUA_HW_MAJOR == 3)
    "Tiliqua R3 " GIT_VERSION,
#elif (TILIQUA_HW_MAJOR == 4)
    "Tiliqua R4 " GIT_VERSION,
#elif (TILIQUA_HW_MAJOR == 5)
    "Tiliqua R5 " GIT_VERSION,
#else
#error "Unknown TILIQUA_HW_MAJOR"
#endif
    usb_serial,                   // 3: Serial, uses flash unique ID
#if ( USB_CDC_UART_BRIDGE )
    "Tiliqua CDC 0", // 4: CDC Interface 0
#endif
};

static uint16_t _desc_str[32];

// Microsoft OS 1.0 String Descriptor (raw 18-byte format per spec)
static const uint8_t ms_os_10_string_descriptor[18] = {
  0x12,                                     // bLength = 18
  0x03,                                     // bDescriptorType = string
  'M', 0, 'S', 0, 'F', 0, 'T', 0,         // qwSignature = "MSFT100" UTF-16LE
  '1', 0, '0', 0, '0', 0,
  0xEE,                                     // bMS_VendorCode
  0x00,                                     // bPad
};

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0) {
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
  } else if (index == 0xee) {
      // Microsoft OS 1.0 String Descriptor — return raw descriptor directly
      return (uint16_t const*)ms_os_10_string_descriptor;
  } else {
      // Convert ASCII string into UTF-16
      const char* str;

      if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) {
          return NULL;
      }

      str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      if ( chr_count > 31 ) chr_count = 31;

      for(uint8_t i=0; i<chr_count; i++)
      {
          _desc_str[1+i] = str[i];
      }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}
