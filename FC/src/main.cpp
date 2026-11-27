/**
 * @file main.cpp
 * @brief Standalone STM32F4 Black Pill USB CDC ("Hello" every 2s) + LED (1s/1s)
 *        Zero C++ STL / libstdc++ header dependencies (<stddef.h>, <stdint.h> only).
 */

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

namespace {

// ============================================================================
// Конфигурация платы и таймингов
// ============================================================================
constexpr uint32_t kSysClkHz           = 96000000U;
constexpr uint32_t kCdcSendIntervalMs  = 2000U;
constexpr uint32_t kLedBlinkIntervalMs = 1000U;
constexpr bool     kLedActiveLow       = true;
constexpr char     kHelloPayload[]     = "Hello\r\n";

// USB CDC ACM константы
constexpr uint8_t  kCdcDataInEp        = 0x81U;  // EP1 IN (Device -> Host)
constexpr uint8_t  kCdcDataOutEp       = 0x01U;  // EP1 OUT (Host -> Device)
constexpr uint8_t  kCdcCmdEp           = 0x82U;  // EP2 IN (Notification)
constexpr uint16_t kEp0MaxPacketSize   = 64U;
constexpr uint16_t kBulkMaxPacketSize  = 64U;
constexpr uint16_t kCmdMaxPacketSize   = 8U;

// USB стандартные запросы и дескрипторы
constexpr uint8_t  kReqGetStatus       = 0x00U;
constexpr uint8_t  kReqClearFeature    = 0x01U;
constexpr uint8_t  kReqSetFeature      = 0x03U;
constexpr uint8_t  kReqSetAddress      = 0x05U;
constexpr uint8_t  kReqGetDescriptor   = 0x06U;
constexpr uint8_t  kReqSetConfiguration = 0x09U;

constexpr uint8_t  kDescDevice         = 0x01U;
constexpr uint8_t  kDescConfiguration  = 0x02U;
constexpr uint8_t  kDescString         = 0x03U;

// CDC запросы
constexpr uint8_t  kCdcSetLineCoding   = 0x20U;
constexpr uint8_t  kCdcGetLineCoding   = 0x21U;
constexpr uint8_t  kCdcSetControlLine  = 0x22U;

struct __attribute__((packed)) UsbSetupPacket {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
};

struct __attribute__((packed)) CdcLineCoding {
  uint32_t dwDTERate;
  uint8_t  bCharFormat;
  uint8_t  bParityType;
  uint8_t  bDataBits;
};

// ============================================================================
// Локальные утилиты памяти и строк (без <cstring> и libstdc++)
// ============================================================================
void copyBytes(void *dst, const void *src, size_t count) {
  uint8_t *d = static_cast<uint8_t *>(dst);
  const uint8_t *s = static_cast<const uint8_t *>(src);
  for (size_t i = 0U; i < count; ++i) {
    d[i] = s[i];
  }
}

void zeroBytes(void *dst, size_t count) {
  uint8_t *d = static_cast<uint8_t *>(dst);
  for (size_t i = 0U; i < count; ++i) {
    d[i] = 0U;
  }
}

size_t cstrLen(const char *str) {
  if (str == NULL) {
    return 0U;
  }
  size_t len = 0U;
  while (str[len] != '\0') {
    ++len;
  }
  return len;
}

// ============================================================================
// USB Дескрипторы
// ============================================================================
constexpr uint8_t kDeviceDescriptor[] = {
    18U,                  // bLength
    kDescDevice,          // bDescriptorType (Device)
    0x00U, 0x02U,         // bcdUSB = 2.00
    0x02U,                // bDeviceClass = CDC (Communications)
    0x02U,                // bDeviceSubClass
    0x00U,                // bDeviceProtocol
    kEp0MaxPacketSize,    // bMaxPacketSize0
    0x83U, 0x04U,         // idVendor = 0x0483 (STMicroelectronics)
    0x40U, 0x57U,         // idProduct = 0x5740 (STM32 Virtual COM Port)
    0x00U, 0x02U,         // bcdDevice = 2.00
    1U,                   // iManufacturer
    2U,                   // iProduct
    3U,                   // iSerialNumber
    1U                    // bNumConfigurations
};

constexpr uint8_t kConfigDescriptor[] = {
    // Configuration Descriptor (9 bytes)
    9U,                   // bLength
    kDescConfiguration,   // bDescriptorType
    67U, 0x00U,           // wTotalLength = 67
    2U,                   // bNumInterfaces = 2
    1U,                   // bConfigurationValue = 1
    0U,                   // iConfiguration
    0xC0U,                // bmAttributes = Self-powered
    50U,                  // bMaxPower = 100 mA

    // Interface 0: CDC Communication Class Interface (9 bytes)
    9U,                   // bLength
    0x04U,                // bDescriptorType (Interface)
    0U,                   // bInterfaceNumber = 0
    0U,                   // bAlternateSetting = 0
    1U,                   // bNumEndpoints = 1
    0x02U,                // bInterfaceClass = CDC Communication
    0x02U,                // bInterfaceSubClass = Abstract Control Model
    0x01U,                // bInterfaceProtocol = AT commands (v25ter)
    0U,                   // iInterface

    // Header Functional Descriptor (5 bytes)
    5U, 0x24U, 0x00U, 0x10U, 0x01U,

    // Call Management Functional Descriptor (5 bytes)
    5U, 0x24U, 0x01U, 0x00U, 0x01U,

    // ACM Functional Descriptor (4 bytes)
    4U, 0x24U, 0x02U, 0x02U,

    // Union Functional Descriptor (5 bytes)
    5U, 0x24U, 0x06U, 0x00U, 0x01U,

    // Endpoint 2 IN Descriptor (7 bytes)
    7U,                   // bLength
    0x05U,                // bDescriptorType (Endpoint)
    kCdcCmdEp,            // bEndpointAddress (0x82)
    0x03U,                // bmAttributes = Interrupt
    static_cast<uint8_t>(kCmdMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kCmdMaxPacketSize >> 8U) & 0xFFU),
    16U,                  // bInterval = 16 ms

    // Interface 1: CDC Data Interface (9 bytes)
    9U,                   // bLength
    0x04U,                // bDescriptorType (Interface)
    1U,                   // bInterfaceNumber = 1
    0U,                   // bAlternateSetting = 0
    2U,                   // bNumEndpoints = 2
    0x0AU,                // bInterfaceClass = CDC Data
    0x00U,                // bInterfaceSubClass
    0x00U,                // bInterfaceProtocol
    0U,                   // iInterface

    // Endpoint 1 OUT Descriptor (7 bytes)
    7U,                   // bLength
    0x05U,                // bDescriptorType (Endpoint)
    kCdcDataOutEp,        // bEndpointAddress (0x01)
    0x02U,                // bmAttributes = Bulk
    static_cast<uint8_t>(kBulkMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kBulkMaxPacketSize >> 8U) & 0xFFU),
    0U,                   // bInterval

    // Endpoint 1 IN Descriptor (7 bytes)
    7U,                   // bLength
    0x05U,                // bDescriptorType (Endpoint)
    kCdcDataInEp,         // bEndpointAddress (0x81)
    0x02U,                // bmAttributes = Bulk
    static_cast<uint8_t>(kBulkMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kBulkMaxPacketSize >> 8U) & 0xFFU),
    0U                    // bInterval
};

constexpr uint8_t kLangIdDescriptor[] = {
    4U, kDescString, 0x09U, 0x04U  // 0x0409 = English (US)
};

// Строковый дескриптор "BlackPill" в UTF-16LE
constexpr uint8_t kProductDescriptor[] = {
    20U, kDescString,
    'B', 0, 'l', 0, 'a', 0, 'c', 0, 'k', 0, 'P', 0, 'i', 0, 'l', 0, 'l', 0
};

// ============================================================================
// Состояние системного времени и USB CDC
// ============================================================================
volatile uint32_t g_millis = 0U;

volatile bool g_usb_configured = false;
volatile bool g_cdc_dtr_active = false;
volatile bool g_cdc_tx_busy = false;
volatile uint8_t g_pending_address = 0U;
volatile bool g_has_pending_address = false;
volatile uint16_t g_ep0_tx_remaining = 0U;
volatile const uint8_t *g_ep0_tx_ptr = NULL;
volatile bool g_awaiting_line_coding = false;

CdcLineCoding g_line_coding = {115200U, 0U, 0U, 8U};
uint8_t g_ep0_rx_buf[kEp0MaxPacketSize];
uint8_t g_cdc_rx_buf[kBulkMaxPacketSize];
uint8_t g_cdc_tx_buf[kBulkMaxPacketSize];

// ============================================================================
// Тактирование (HSE 25 MHz -> PLL 96 MHz, USB 48 MHz) и SysTick 1 ms
// ============================================================================
void configureClock96MhzUsb48Mhz() {
  FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;

  RCC->CR |= RCC_CR_HSEON;
  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
  }

  RCC->PLLCFGR = (25U << RCC_PLLCFGR_PLLM_Pos) |
                 (192U << RCC_PLLCFGR_PLLN_Pos) |
                 (0U << RCC_PLLCFGR_PLLP_Pos) |
                 (RCC_PLLCFGR_PLLSRC_HSE) |
                 (4U << RCC_PLLCFGR_PLLQ_Pos);

  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
  }

  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }

  SystemCoreClock = kSysClkHz;
}

void initSysTick1ms() {
  SysTick->LOAD = (kSysClkHz / 1000U) - 1U;
  SysTick->VAL = 0U;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

uint32_t millis() {
  __disable_irq();
  const uint32_t now = g_millis;
  __enable_irq();
  return now;
}

// ============================================================================
// Светодиод на PC13 (1 сек / 1 сек)
// ============================================================================
class OnboardLed {
public:
  OnboardLed(GPIO_TypeDef *port, uint32_t pin, bool active_low, uint32_t period_ms)
      : port_(port),
        pin_(pin),
        active_low_(active_low),
        period_ms_(period_ms),
        last_toggle_ms_(0U),
        on_(false) {}

  void init(uint32_t now_ms) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    port_->MODER &= ~(3U << (pin_ * 2U));
    port_->MODER |= (1U << (pin_ * 2U));
    port_->OTYPER &= ~(1U << pin_);
    port_->PUPDR &= ~(3U << (pin_ * 2U));

    last_toggle_ms_ = now_ms;
    set(false);
  }

  void set(bool on) {
    on_ = on;
    const bool drive_high = active_low_ ? !on : on;
    if (drive_high) {
      port_->BSRR = (1U << pin_);
    } else {
      port_->BSRR = (1U << (pin_ + 16U));
    }
  }

  void update(uint32_t now_ms) {
    if ((now_ms - last_toggle_ms_) >= period_ms_) {
      last_toggle_ms_ = now_ms;
      set(!on_);
    }
  }

private:
  GPIO_TypeDef *port_;
  uint32_t pin_;
  bool active_low_;
  uint32_t period_ms_;
  uint32_t last_toggle_ms_;
  bool on_;
};

// ============================================================================
// Низкоуровневый драйвер USB OTG FS (CDC ACM)
// ============================================================================
void writePacketToTxFifo(uint8_t ep_num, const uint8_t *src, uint16_t len) {
  USB_OTG_INEndpointTypeDef *in_ep = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep_num * 0x20U));

  const uint16_t pkt_cnt = (len == 0U) ? 1U : static_cast<uint16_t>((len + 63U) / 64U);
  in_ep->DIEPTSIZ = (pkt_cnt << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | len;
  in_ep->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

  volatile uint32_t *fifo = reinterpret_cast<volatile uint32_t *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep_num * 0x1000U));

  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  for (uint16_t i = 0U; i < words; ++i) {
    uint32_t word = 0U;
    copyBytes(&word, src + (i * 4U), sizeof(word));
    *fifo = word;
  }
}

void readPacketFromRxFifo(uint8_t *dst, uint16_t len) {
  volatile uint32_t *fifo = reinterpret_cast<volatile uint32_t *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE);

  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  uint16_t copied = 0U;
  for (uint16_t i = 0U; i < words; ++i) {
    const uint32_t word = *fifo;
    const uint16_t chunk = (len - copied >= 4U) ? 4U : static_cast<uint16_t>(len - copied);
    if ((dst != NULL) && (chunk > 0U)) {
      copyBytes(dst + copied, &word, chunk);
    }
    copied = static_cast<uint16_t>(copied + chunk);
  }
}

void armEp0OutSetup() {
  USB_OTG_OUTEndpointTypeDef *out0 = reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE);
  out0->DOEPTSIZ = (1U << USB_OTG_DOEPTSIZ0_STUPCNT_Pos) |
                   (1U << USB_OTG_DOEPTSIZ0_PKTCNT_Pos) |
                   kEp0MaxPacketSize;
  out0->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}

void armBulkOutEp1() {
  USB_OTG_OUTEndpointTypeDef *out1 = reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (1U * 0x20U));
  out1->DOEPTSIZ = (1U << USB_OTG_DOEPTSIZ1_PKTCNT_Pos) | kBulkMaxPacketSize;
  out1->DOEPCTL = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK |
                  (2U << USB_OTG_DOEPCTL_EPTYP_Pos) | kBulkMaxPacketSize;
}

void sendEp0Data(const uint8_t *data, uint16_t total_len, uint16_t requested_len) {
  const uint16_t send_len = (total_len < requested_len) ? total_len : requested_len;
  const uint16_t first_chunk = (send_len > kEp0MaxPacketSize) ? kEp0MaxPacketSize : send_len;

  g_ep0_tx_ptr = data + first_chunk;
  g_ep0_tx_remaining = static_cast<uint16_t>(send_len - first_chunk);

  writePacketToTxFifo(0U, data, first_chunk);
  armEp0OutSetup();
}

void sendEp0ZeroLengthStatus() {
  g_ep0_tx_ptr = NULL;
  g_ep0_tx_remaining = 0U;
  writePacketToTxFifo(0U, NULL, 0U);
  armEp0OutSetup();
}

void handleUsbReset() {
  g_usb_configured = false;
  g_cdc_dtr_active = false;
  g_cdc_tx_busy = false;
  g_has_pending_address = false;
  g_awaiting_line_coding = false;

  USB_OTG_GlobalTypeDef *usb = USB_OTG_FS;
  USB_OTG_DeviceTypeDef *dev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

  dev->DCFG &= ~USB_OTG_DCFG_DAD;

  usb->GRXFSIZ = 128U;
  usb->DIEPTXF0_HNPTXFSIZ = (64U << USB_OTG_TX0FSIZ_FD_Pos) | 128U;
  usb->DIEPTXF[0U] = (64U << USB_OTG_DIEPTXF_INEPTXFD_Pos) | 192U;
  usb->DIEPTXF[1U] = (32U << USB_OTG_DIEPTXF_INEPTXFD_Pos) | 256U;

  dev->DAINTMSK = (1U << 0U) | (1U << 1U) | (1U << 2U) |
                  (1U << 16U) | (1U << 17U);
  dev->DOEPMSK = USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_STUPM;
  dev->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;

  armEp0OutSetup();
}

void handleSetupPacket(const UsbSetupPacket &pkt) {
  if (pkt.bRequest == kReqGetDescriptor) {
    const uint8_t desc_type = static_cast<uint8_t>((pkt.wValue >> 8U) & 0xFFU);
    if (desc_type == kDescDevice) {
      sendEp0Data(kDeviceDescriptor, sizeof(kDeviceDescriptor), pkt.wLength);
      return;
    }
    if (desc_type == kDescConfiguration) {
      sendEp0Data(kConfigDescriptor, sizeof(kConfigDescriptor), pkt.wLength);
      return;
    }
    if (desc_type == kDescString) {
      const uint8_t desc_idx = static_cast<uint8_t>(pkt.wValue & 0xFFU);
      if (desc_idx == 0U) {
        sendEp0Data(kLangIdDescriptor, sizeof(kLangIdDescriptor), pkt.wLength);
        return;
      }
      sendEp0Data(kProductDescriptor, sizeof(kProductDescriptor), pkt.wLength);
      return;
    }
  } else if (pkt.bRequest == kReqSetAddress) {
    g_pending_address = static_cast<uint8_t>(pkt.wValue & 0x7FU);
    g_has_pending_address = true;
    sendEp0ZeroLengthStatus();
    return;
  } else if (pkt.bRequest == kReqSetConfiguration) {
    const uint8_t cfg = static_cast<uint8_t>(pkt.wValue & 0xFFU);
    g_usb_configured = (cfg != 0U);
    if (g_usb_configured) {
      USB_OTG_INEndpointTypeDef *in1 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (1U * 0x20U));
      in1->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                     (2U << USB_OTG_DIEPCTL_EPTYP_Pos) |
                     (1U << USB_OTG_DIEPCTL_TXFNUM_Pos) | kBulkMaxPacketSize;

      USB_OTG_INEndpointTypeDef *in2 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (2U * 0x20U));
      in2->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                     (3U << USB_OTG_DIEPCTL_EPTYP_Pos) |
                     (2U << USB_OTG_DIEPCTL_TXFNUM_Pos) | kCmdMaxPacketSize;

      armBulkOutEp1();
    }
    sendEp0ZeroLengthStatus();
    return;
  } else if (pkt.bRequest == kCdcSetLineCoding) {
    g_awaiting_line_coding = true;
    armEp0OutSetup();
    return;
  } else if (pkt.bRequest == kCdcGetLineCoding) {
    sendEp0Data(reinterpret_cast<const uint8_t *>(&g_line_coding),
                 sizeof(g_line_coding), pkt.wLength);
    return;
  } else if (pkt.bRequest == kCdcSetControlLine) {
    g_cdc_dtr_active = ((pkt.wValue & 0x01U) != 0U);
    sendEp0ZeroLengthStatus();
    return;
  } else if ((pkt.bRequest == kReqGetStatus) ||
             (pkt.bRequest == kReqClearFeature) ||
             (pkt.bRequest == kReqSetFeature)) {
    static constexpr uint8_t kZeroStatus[2] = {0U, 0U};
    sendEp0Data(kZeroStatus, sizeof(kZeroStatus), pkt.wLength);
    return;
  }

  USB_OTG_INEndpointTypeDef *in0 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE);
  in0->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  armEp0OutSetup();
}

void initUsbCdcFs() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~((3U << (11U * 2U)) | (3U << (12U * 2U)));
  GPIOA->MODER |= ((2U << (11U * 2U)) | (2U << (12U * 2U)));
  GPIOA->OSPEEDR |= ((3U << (11U * 2U)) | (3U << (12U * 2U)));
  GPIOA->AFR[1U] &= ~((0xFU << ((11U - 8U) * 4U)) | (0xFU << ((12U - 8U) * 4U)));
  GPIOA->AFR[1U] |= ((10U << ((11U - 8U) * 4U)) | (10U << ((12U - 8U) * 4U)));

  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

  USB_OTG_GlobalTypeDef *usb = USB_OTG_FS;
  USB_OTG_DeviceTypeDef *dev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

  usb->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  while ((usb->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U) {
  }

  usb->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
  usb->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL;
  for (volatile uint32_t i = 0U; i < 200000U; ++i) {
  }

  dev->DCFG |= (3U << USB_OTG_DCFG_DSPD_Pos);
  usb->GCCFG |= USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;

  usb->GINTMSK = USB_OTG_GINTMSK_USBRST |
                 USB_OTG_GINTMSK_RXFLVLM |
                 USB_OTG_GINTMSK_IEPINT |
                 USB_OTG_GINTMSK_OEPINT;
  usb->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

  NVIC_SetPriority(OTG_FS_IRQn, 5U);
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

bool cdcSendBytes(const uint8_t *data, uint16_t len) {
  if (!g_usb_configured || (data == NULL) || (len == 0U) || (len > kBulkMaxPacketSize)) {
    return false;
  }

  __disable_irq();
  if (g_cdc_tx_busy) {
    __enable_irq();
    return false;
  }
  g_cdc_tx_busy = true;
  copyBytes(g_cdc_tx_buf, data, len);
  writePacketToTxFifo(1U, g_cdc_tx_buf, len);
  __enable_irq();

  return true;
}

class PeriodicCdcHello {
public:
  PeriodicCdcHello(const char *msg, uint32_t interval_ms)
      : msg_(msg), interval_ms_(interval_ms), last_send_ms_(0U) {}

  void init(uint32_t now_ms) {
    last_send_ms_ = now_ms;
  }

  void update(uint32_t now_ms) {
    if ((now_ms - last_send_ms_) >= interval_ms_) {
      const size_t len = cstrLen(msg_);
      if ((len > 0U) && (len <= kBulkMaxPacketSize)) {
        if (cdcSendBytes(reinterpret_cast<const uint8_t *>(msg_), static_cast<uint16_t>(len))) {
          last_send_ms_ = now_ms;
        }
      }
    }
  }

private:
  const char *msg_;
  uint32_t interval_ms_;
  uint32_t last_send_ms_;
};

OnboardLed g_led(GPIOC, 13U, kLedActiveLow, kLedBlinkIntervalMs);
PeriodicCdcHello g_hello(kHelloPayload, kCdcSendIntervalMs);

}  // namespace

// ============================================================================
// Обработчики прерываний Cortex-M4
// ============================================================================
extern "C" void SysTick_Handler(void) {
  ++g_millis;
}

extern "C" void OTG_FS_IRQHandler(void) {
  USB_OTG_GlobalTypeDef *usb = USB_OTG_FS;
  USB_OTG_DeviceTypeDef *dev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(
      USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

  const uint32_t int_status = usb->GINTSTS & usb->GINTMSK;

  if ((int_status & USB_OTG_GINTSTS_USBRST) != 0U) {
    handleUsbReset();
    usb->GINTSTS = USB_OTG_GINTSTS_USBRST;
  }

  if ((int_status & USB_OTG_GINTSTS_RXFLVL) != 0U) {
    const uint32_t rx_status = usb->GRXSTSP;
    const uint8_t ep_num = static_cast<uint8_t>(rx_status & USB_OTG_GRXSTSP_EPNUM_Msk);
    const uint16_t byte_cnt = static_cast<uint16_t>(
        (rx_status & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos);
    const uint8_t pkt_sts = static_cast<uint8_t>(
        (rx_status & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos);

    if (pkt_sts == 0x06U) {
      readPacketFromRxFifo(g_ep0_rx_buf, byte_cnt);
    } else if (pkt_sts == 0x02U) {
      if (ep_num == 0U) {
        readPacketFromRxFifo(g_ep0_rx_buf, byte_cnt);
      } else if (ep_num == 1U) {
        const uint16_t to_read = (byte_cnt > kBulkMaxPacketSize) ? kBulkMaxPacketSize : byte_cnt;
        readPacketFromRxFifo(g_cdc_rx_buf, to_read);
      } else {
        readPacketFromRxFifo(NULL, byte_cnt);
      }
    }
  }

  if ((int_status & USB_OTG_GINTSTS_OEPINT) != 0U) {
    const uint32_t oep_int = dev->DAINT;

    if ((oep_int & (1U << 16U)) != 0U) {
      USB_OTG_OUTEndpointTypeDef *out0 = reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE);
      const uint32_t flags = out0->DOEPINT;
      out0->DOEPINT = flags;

      if ((flags & USB_OTG_DOEPINT_STUP) != 0U) {
        UsbSetupPacket setup_pkt{};
        zeroBytes(&setup_pkt, sizeof(setup_pkt));
        copyBytes(&setup_pkt, g_ep0_rx_buf, sizeof(setup_pkt));
        handleSetupPacket(setup_pkt);
      } else if (((flags & USB_OTG_DOEPINT_XFRC) != 0U) && g_awaiting_line_coding) {
        g_awaiting_line_coding = false;
        copyBytes(&g_line_coding, g_ep0_rx_buf, sizeof(g_line_coding));
        sendEp0ZeroLengthStatus();
      }
    }

    if ((oep_int & (1U << 17U)) != 0U) {
      USB_OTG_OUTEndpointTypeDef *out1 = reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (1U * 0x20U));
      out1->DOEPINT = out1->DOEPINT;
      armBulkOutEp1();
    }
  }

  if ((int_status & USB_OTG_GINTSTS_IEPINT) != 0U) {
    const uint32_t iep_int = dev->DAINT;

    if ((iep_int & (1U << 0U)) != 0U) {
      USB_OTG_INEndpointTypeDef *in0 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE);
      const uint32_t flags = in0->DIEPINT;
      in0->DIEPINT = flags;

      if ((flags & USB_OTG_DIEPINT_XFRC) != 0U) {
        if (g_has_pending_address) {
          dev->DCFG &= ~USB_OTG_DCFG_DAD;
          dev->DCFG |= (static_cast<uint32_t>(g_pending_address) << USB_OTG_DCFG_DAD_Pos);
          g_has_pending_address = false;
        }
        if (g_ep0_tx_remaining > 0U) {
          const uint16_t chunk = (g_ep0_tx_remaining > kEp0MaxPacketSize)
                                      ? kEp0MaxPacketSize
                                      : g_ep0_tx_remaining;
          writePacketToTxFifo(0U, const_cast<const uint8_t *>(g_ep0_tx_ptr), chunk);
          g_ep0_tx_ptr += chunk;
          g_ep0_tx_remaining = static_cast<uint16_t>(g_ep0_tx_remaining - chunk);
        }
      }
    }

    if ((iep_int & (1U << 1U)) != 0U) {
      USB_OTG_INEndpointTypeDef *in1 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (1U * 0x20U));
      in1->DIEPINT = in1->DIEPINT;
      g_cdc_tx_busy = false;
    }

    if ((iep_int & (1U << 2U)) != 0U) {
      USB_OTG_INEndpointTypeDef *in2 = reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
          USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (2U * 0x20U));
      in2->DIEPINT = in2->DIEPINT;
    }
  }
}

// ============================================================================
// Точка входа
// ============================================================================
int main(void) {
  configureClock96MhzUsb48Mhz();
  initSysTick1ms();

  const uint32_t start_ms = millis();
  g_led.init(start_ms);
  g_hello.init(start_ms);
  initUsbCdcFs();

  while (1) {
    const uint32_t now_ms = millis();
    g_led.update(now_ms);
    g_hello.update(now_ms);
  }
}
