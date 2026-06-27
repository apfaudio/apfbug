#ifndef DirtyJtagConfig_h
#define DirtyJtagConfig_h

// Set to 0 to disable USB-CDC-UART bridge
#define USB_CDC_UART_BRIDGE  1

// General mapping
// TDI  SPIO RX
// TDO  SPIO TX
// TCK  SPIO SCK
// TMS  SPIO CS

#define PIN_VBUS 22

#define PIN_TDI 16
#define PIN_TDO 17
#define PIN_TCK 18
#define PIN_TMS 19

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 1
#define PIN_UART0 uart0
#define PIN_UART0_TX    12
#define PIN_UART0_RX    13
#define PIN_UART1 uart1
#define PIN_UART1_TX    4
#define PIN_UART1_RX    5
#endif // USB_CDC_UART_BRIDGE

#endif // DirtyJtagConfig_h
