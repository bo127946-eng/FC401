/**
 * @file main.cpp
 * @brief Self-contained STM32F401 Black Pill USB CDC ("Hello" every 2s) + LED (1s/1s)
 *        Zero external HAL, zero C++ STL, zero CMSIS header dependencies.
 */

#include <stddef.h>
#include <stdint.h>

namespace {

// ============================================================================
// Конфигурация платы и таймингов
// ============================================================================
constexpr uint32_t kSysClkHz           = 96000000U;
constexpr uint32_t kHseStartupTimeout  = 200000U;
constexpr uint32_t kCdcSendIntervalMs  = 2000U;
constexpr uint32_t kLedBlinkIntervalMs = 1000U;
constexpr bool     kLedActiveLow       = true;
constexpr char     kHelloPayload[]     = "Hello\r\n";

// ============================================================================
// Адреса периферии и битовые маски STM32F401 (Cortex-M4 + USB_OTG_FS)
// ============================================================================
constexpr uintptr_t kPeriplhBase       = 0x40000000UL;
constexpr uintptr_t kAhb1PeriphBase    = kPeriplhBase + 0x00020000UL;
constexpr uintptr_t kAhb2PeriphBase    = 0x50000000UL;
constexpr uintptr_t kFlashStartAddr    = 0x08000000UL;
constexpr uintptr_t kUsbOtgFsBase      = kAhb2PeriphBase + 0x00000UL;
constexpr uintptr_t kUsbInEpBase       = 0x900UL;
constexpr uintptr_t kUsbOutEpBase      = 0xB00UL;
constexpr uintptr_t kUsbFifoBase       = 0x1000UL;
constexpr uintptr_t kScsBase           = 0xE000E000UL;
constexpr uintptr_t kScbBase           = kScsBase + 0x0D00UL;
constexpr uintptr_t kSysTickBase       = kScsBase + 0x0010UL;
constexpr uintptr_t kNvicBase          = kScsBase + 0x0100UL;

constexpr int32_t   kOtgFsIrqn         = 67;

struct GpioRegs {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
};

struct RccRegs {
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t RESERVED0;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t RESERVED2;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
};

struct FlashRegs {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t OPTCR;
};

struct ScbRegs {
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  uint32_t RESERVED0[18];
  volatile uint32_t CPACR;
};

struct SysTickRegs {
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile uint32_t CALIB;
};

struct NvicRegs {
  volatile uint32_t ISER[8];
  uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];
  uint32_t RESERVED1[24];
  volatile uint32_t ISPR[8];
  uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];
  uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];
  uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];
};

struct UsbOtgGlobalRegs {
  volatile uint32_t GOTGCTL;
  volatile uint32_t GOTGINT;
  volatile uint32_t GAHBCFG;
  volatile uint32_t GUSBCFG;
  volatile uint32_t GRSTCTL;
  volatile uint32_t GINTSTS;
  volatile uint32_t GINTMSK;
  volatile uint32_t GRXSTSR;
  volatile uint32_t GRXSTSP;
  volatile uint32_t GRXFSIZ;
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;
  volatile uint32_t HNPTXSTS;
  uint32_t RESERVED0[2];
  volatile uint32_t GCCFG;
  volatile uint32_t CID;
  uint32_t RESERVED1[48];
  volatile uint32_t HPTXFSIZ;
  volatile uint32_t DIEPTXF[15];
};

struct UsbOtgDeviceRegs {
  volatile uint32_t DCFG;
  volatile uint32_t DCTL;
  volatile uint32_t DSTS;
  uint32_t RESERVED0;
  volatile uint32_t DIEPMSK;
  volatile uint32_t DOEPMSK;
  volatile uint32_t DAINT;
  volatile uint32_t DAINTMSK;
  uint32_t RESERVED1[2];
  volatile uint32_t DVBUSDIS;
  volatile uint32_t DVBUSPULSE;
  volatile uint32_t DTHRCTL;
  volatile uint32_t DIEPEMPMSK;
  volatile uint32_t DEACHINT;
  volatile uint32_t DEACHINTMSK;
  uint32_t RESERVED2;
  volatile uint32_t DIEPEACHMSK1;
  uint32_t RESERVED3[14];
  volatile uint32_t DOEPEACHMSK1;
};

struct UsbOtgInEpRegs {
  volatile uint32_t DIEPCTL;
  uint32_t RESERVED0;
  volatile uint32_t DIEPINT;
  uint32_t RESERVED1;
  volatile uint32_t DIEPTSIZ;
  volatile uint32_t DIEPDMA;
  volatile uint32_t DTXFSTS;
  uint32_t RESERVED2;
};

struct UsbOtgOutEpRegs {
  volatile uint32_t DOEPCTL;
  uint32_t RESERVED0;
  volatile uint32_t DOEPINT;
  uint32_t RESERVED1;
  volatile uint32_t DOEPTSIZ;
  volatile uint32_t DOEPDMA;
  uint32_t RESERVED2[2];
};

constexpr uint32_t kFlashAcrLat3ws        = (3U << 0U);
constexpr uint32_t kFlashAcrPrften       = (1U << 8U);
constexpr uint32_t kFlashAcrIcen         = (1U << 9U);
constexpr uint32_t kFlashAcrDcen         = (1U << 10U);

constexpr uint32_t kScbCpacrFpuEnable    = (0xFU << 20U);

constexpr uint32_t kRccCrHsion           = (1U << 0U);
constexpr uint32_t kRccCrHsirdy          = (1U << 1U);
constexpr uint32_t kRccCrHseon           = (1U << 16U);
constexpr uint32_t kRccCrHserdy          = (1U << 17U);
constexpr uint32_t kRccCrPllon           = (1U << 24U);
constexpr uint32_t kRccCrPllrdy          = (1U << 25U);

constexpr uint32_t kRccPllcfgrPllmPos    = 0U;
constexpr uint32_t kRccPllcfgrPllnPos    = 6U;
constexpr uint32_t kRccPllcfgrPllpPos    = 16U;
constexpr uint32_t kRccPllcfgrPllsrcHsi  = (0U << 22U);
constexpr uint32_t kRccPllcfgrPllsrcHse  = (1U << 22U);
constexpr uint32_t kRccPllcfgrPllqPos    = 24U;

constexpr uint32_t kRccCfgrSwMask        = (3U << 0U);
constexpr uint32_t kRccCfgrSwPll         = (2U << 0U);
constexpr uint32_t kRccCfgrSwsMask       = (3U << 2U);
constexpr uint32_t kRccCfgrSwsPll        = (2U << 2U);
constexpr uint32_t kRccCfgrPpre1Div2     = (4U << 10U);
constexpr uint32_t kRccCfgrPpre2Div1     = (0U << 13U);

constexpr uint32_t kRccAhb1EnGpioa       = (1U << 0U);
constexpr uint32_t kRccAhb1EnGpioc       = (1U << 2U);
constexpr uint32_t kRccAhb2EnOtgfs       = (1U << 7U);

constexpr uint32_t kSysTickEnable        = (1U << 0U);
constexpr uint32_t kSysTickTickInt       = (1U << 1U);
constexpr uint32_t kSysTickClkSource     = (1U << 2U);

constexpr uint32_t kGahbcfgGint          = (1U << 0U);
constexpr uint32_t kGusbcflgFdmod        = (1U << 30U);
constexpr uint32_t kGrstctlCsrst         = (1U << 0U);

constexpr uint32_t kGintstsUsbreset      = (1U << 12U);
constexpr uint32_t kGintstsRxflvl        = (1U << 4U);
constexpr uint32_t kGintstsOepint        = (1U << 19U);
constexpr uint32_t kGintstsIepint        = (1U << 18U);

constexpr uint32_t kGintmskUsbreset      = (1U << 12U);
constexpr uint32_t kGintmskRxflvl        = (1U << 4U);
constexpr uint32_t kGintmskOepint        = (1U << 19U);
constexpr uint32_t kGintmskIepint        = (1U << 18U);

constexpr uint32_t kGrxstspEpnumMask     = 0x0FU;
constexpr uint32_t kGrxstspBcntPos       = 4U;
constexpr uint32_t kGrxstspBcntMask      = (0x7FFU << kGrxstspBcntPos);
constexpr uint32_t kGrxstspPktstsPos     = 17U;
constexpr uint32_t kGrxstspPktstsMask    = (0xFU << kGrxstspPktstsPos);

constexpr uint32_t kGccfgPwrdwn          = (1U << 16U);
constexpr uint32_t kGccfgVbden           = (1U << 21U);

constexpr uint32_t kDcfgDspdPos          = 0U;
constexpr uint32_t kDcfgDadPos           = 4U;
constexpr uint32_t kDcfgDadMask          = (0x7FU << kDcfgDadPos);

constexpr uint32_t kDiepmskXfrcm         = (1U << 0U);
constexpr uint32_t kDoepmskXfrcm         = (1U << 0U);
constexpr uint32_t kDoepmskStupm         = (1U << 3U);

constexpr uint32_t kDiepintXfrcm         = (1U << 0U);
constexpr uint32_t kDoepintXfrcm         = (1U << 0U);
constexpr uint32_t kDoepintStup          = (1U << 3U);

constexpr uint32_t kDiepctlEpena         = (1U << 31U);
constexpr uint32_t kDiepctlCnak          = (1U << 26U);
constexpr uint32_t kDiepctlStall         = (1U << 21U);
constexpr uint32_t kDiepctlTxfnumPos     = 22U;
constexpr uint32_t kDiepctlEptypPos      = 18U;
constexpr uint32_t kDiepctlSd0pid        = (1U << 28U);
constexpr uint32_t kDiepctlUsbaep        = (1U << 15U);

constexpr uint32_t kDoepctlEpena         = (1U << 31U);
constexpr uint32_t kDoepctlCnak          = (1U << 26U);
constexpr uint32_t kDoepctlEptypPos      = 18U;

constexpr uint32_t kDieptsizPktcntPos    = 19U;
constexpr uint32_t kDoeptsiz0PktcntPos   = 19U;
constexpr uint32_t kDoeptsiz0StupcntPos = 29U;
constexpr uint32_t kDoeptsiz1PktcntPos   = 19U;

inline GpioRegs *gpioA() {
  return reinterpret_cast<GpioRegs *>(kAhb1PeriphBase + 0x0000UL);
}
inline GpioRegs *gpioC() {
  return reinterpret_cast<GpioRegs *>(kAhb1PeriphBase + 0x0800UL);
}
inline RccRegs *rcc() {
  return reinterpret_cast<RccRegs *>(kAhb1PeriphBase + 0x3800UL);
}
inline FlashRegs *flash() {
  return reinterpret_cast<FlashRegs *>(kAhb1PeriphBase + 0x3C00UL);
}
inline ScbRegs *scb() {
  return reinterpret_cast<ScbRegs *>(kScbBase);
}
inline SysTickRegs *sysTick() {
  return reinterpret_cast<SysTickRegs *>(kSysTickBase);
}
inline NvicRegs *nvic() {
  return reinterpret_cast<NvicRegs *>(kNvicBase);
}
inline UsbOtgGlobalRegs *usbGlobal() {
  return reinterpret_cast<UsbOtgGlobalRegs *>(kUsbOtgFsBase);
}
inline UsbOtgDeviceRegs *usbDevice() {
  return reinterpret_cast<UsbOtgDeviceRegs *>(kUsbOtgFsBase + 0x800UL);
}
inline UsbOtgInEpRegs *usbInEp(uint8_t ep_num) {
  return reinterpret_cast<UsbOtgInEpRegs *>(kUsbOtgFsBase + kUsbInEpBase + (ep_num * 0x20UL));
}
inline UsbOtgOutEpRegs *usbOutEp(uint8_t ep_num) {
  return reinterpret_cast<UsbOtgOutEpRegs *>(kUsbOtgFsBase + kUsbOutEpBase + (ep_num * 0x20UL));
}
inline volatile uint32_t *usbFifo(uint8_t ep_num) {
  return reinterpret_cast<volatile uint32_t *>(kUsbOtgFsBase + kUsbFifoBase + (ep_num * 0x1000UL));
}

inline void enableIrq(int32_t irqn) {
  if (irqn < 0) {
    return;
  }
  const uint32_t u_irqn = static_cast<uint32_t>(irqn);
  const uint32_t idx = u_irqn >> 5U;
  const uint32_t bit = 1U << (u_irqn & 0x1FU);
  nvic()->ISER[idx] = bit;
}

inline void setIrqPriority(int32_t irqn, uint8_t priority) {
  if (irqn < 0) {
    return;
  }
  const uint32_t u_irqn = static_cast<uint32_t>(irqn);
  nvic()->IP[u_irqn] = static_cast<uint8_t>((priority << 4U) & 0xF0U);
}

inline void disableIrqGlobal() {
  __asm volatile("cpsid i" ::: "memory");
}

inline void enableIrqGlobal() {
  __asm volatile("cpsie i" ::: "memory");
}

constexpr uint8_t  kCdcDataInEp         = 0x81U;
constexpr uint8_t  kCdcDataOutEp        = 0x01U;
constexpr uint8_t  kCdcCmdEp            = 0x82U;
constexpr uint16_t kEp0MaxPacketSize    = 64U;
constexpr uint16_t kBulkMaxPacketSize   = 64U;
constexpr uint16_t kCmdMaxPacketSize    = 8U;

constexpr uint8_t  kReqGetStatus        = 0x00U;
constexpr uint8_t  kReqClearFeature     = 0x01U;
constexpr uint8_t  kReqSetFeature       = 0x03U;
constexpr uint8_t  kReqSetAddress       = 0x05U;
constexpr uint8_t  kReqGetDescriptor    = 0x06U;
constexpr uint8_t  kReqSetConfiguration = 0x09U;

constexpr uint8_t  kDescDevice          = 0x01U;
constexpr uint8_t  kDescConfiguration   = 0x02U;
constexpr uint8_t  kDescString          = 0x03U;

constexpr uint8_t  kCdcSetLineCoding    = 0x20U;
constexpr uint8_t  kCdcGetLineCoding    = 0x21U;
constexpr uint8_t  kCdcSetControlLine   = 0x22U;

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

constexpr uint8_t kDeviceDescriptor[] = {
    18U, kDescDevice, 0x00U, 0x02U, 0x02U, 0x02U, 0x00U, kEp0MaxPacketSize,
    0x83U, 0x04U, 0x40U, 0x57U, 0x00U, 0x02U, 1U, 2U, 3U, 1U
};

constexpr uint8_t kConfigDescriptor[] = {
    9U, kDescConfiguration, 67U, 0x00U, 2U, 1U, 0U, 0xC0U, 50U,

    9U, 0x04U, 0U, 0U, 1U, 0x02U, 0x02U, 0x01U, 0U,
    5U, 0x24U, 0x00U, 0x10U, 0x01U,
    5U, 0x24U, 0x01U, 0x00U, 0x01U,
    4U, 0x24U, 0x02U, 0x02U,
    5U, 0x24U, 0x06U, 0x00U, 0x01U,
    7U, 0x05U, kCdcCmdEp, 0x03U,
    static_cast<uint8_t>(kCmdMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kCmdMaxPacketSize >> 8U) & 0xFFU),
    16U,

    9U, 0x04U, 1U, 0U, 2U, 0x0AU, 0x00U, 0x00U, 0U,
    7U, 0x05U, kCdcDataOutEp, 0x02U,
    static_cast<uint8_t>(kBulkMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kBulkMaxPacketSize >> 8U) & 0xFFU),
    0U,
    7U, 0x05U, kCdcDataInEp, 0x02U,
    static_cast<uint8_t>(kBulkMaxPacketSize & 0xFFU),
    static_cast<uint8_t>((kBulkMaxPacketSize >> 8U) & 0xFFU),
    0U
};

constexpr uint8_t kLangIdDescriptor[] = {
    4U, kDescString, 0x09U, 0x04U
};

constexpr uint8_t kProductDescriptor[] = {
    20U, kDescString,
    'B', 0, 'l', 0, 'a', 0, 'c', 0, 'k', 0, 'P', 0, 'i', 0, 'l', 0, 'l', 0
};

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

bool tryStartPllFromHse25Mhz() {
  rcc()->CR |= kRccCrHseon;
  uint32_t timeout = kHseStartupTimeout;
  while (((rcc()->CR & kRccCrHserdy) == 0U) && (timeout > 0U)) {
    --timeout;
  }
  if ((rcc()->CR & kRccCrHserdy) == 0U) {
    return false;
  }

  // VCO = 25 / 25 * 192 = 192 MHz; SYSCLK = 192 / 2 = 96 MHz; USB48 = 192 / 4 = 48 MHz
  rcc()->PLLCFGR = (25U << kRccPllcfgrPllmPos) |
                   (192U << kRccPllcfgrPllnPos) |
                   (0U << kRccPllcfgrPllpPos) |
                   kRccPllcfgrPllsrcHse |
                   (4U << kRccPllcfgrPllqPos);

  rcc()->CR |= kRccCrPllon;
  timeout = kHseStartupTimeout;
  while (((rcc()->CR & kRccCrPllrdy) == 0U) && (timeout > 0U)) {
    --timeout;
  }
  return ((rcc()->CR & kRccCrPllrdy) != 0U);
}

void startPllFromHsi16Mhz() {
  rcc()->CR |= kRccCrHsion;
  while ((rcc()->CR & kRccCrHsirdy) == 0U) {
  }

  rcc()->CR &= ~kRccCrPllon;
  while ((rcc()->CR & kRccCrPllrdy) != 0U) {
  }

  // VCO = 16 / 16 * 192 = 192 MHz; SYSCLK = 192 / 2 = 96 MHz; USB48 = 192 / 4 = 48 MHz
  rcc()->PLLCFGR = (16U << kRccPllcfgrPllmPos) |
                   (192U << kRccPllcfgrPllnPos) |
                   (0U << kRccPllcfgrPllpPos) |
                   kRccPllcfgrPllsrcHsi |
                   (4U << kRccPllcfgrPllqPos);

  rcc()->CR |= kRccCrPllon;
  while ((rcc()->CR & kRccCrPllrdy) == 0U) {
  }
}

void configureClock96MhzUsb48Mhz() {
  flash()->ACR = kFlashAcrIcen | kFlashAcrDcen | kFlashAcrPrften | kFlashAcrLat3ws;

  if (!tryStartPllFromHse25Mhz()) {
    startPllFromHsi16Mhz();
  }

  rcc()->CFGR |= kRccCfgrPpre1Div2 | kRccCfgrPpre2Div1;

  rcc()->CFGR &= ~kRccCfgrSwMask;
  rcc()->CFGR |= kRccCfgrSwPll;
  while ((rcc()->CFGR & kRccCfgrSwsMask) != kRccCfgrSwsPll) {
  }
}

void initSysTick1ms() {
  sysTick()->LOAD = (kSysClkHz / 1000U) - 1U;
  sysTick()->VAL = 0U;
  sysTick()->CTRL = kSysTickClkSource | kSysTickTickInt | kSysTickEnable;
}

uint32_t millis() {
  disableIrqGlobal();
  const uint32_t now = g_millis;
  enableIrqGlobal();
  return now;
}

class OnboardLed {
public:
  void configure(GpioRegs *port, uint32_t pin, bool active_low, uint32_t period_ms) {
    port_ = port;
    pin_ = pin;
    active_low_ = active_low;
    period_ms_ = period_ms;
    last_toggle_ms_ = 0U;
    on_ = false;
  }

  void init(uint32_t now_ms) {
    rcc()->AHB1ENR |= kRccAhb1EnGpioc;

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
  GpioRegs *port_ = NULL;
  uint32_t pin_ = 13U;
  bool active_low_ = true;
  uint32_t period_ms_ = 1000U;
  uint32_t last_toggle_ms_ = 0U;
  bool on_ = false;
};

void writePacketToTxFifo(uint8_t ep_num, const uint8_t *src, uint16_t len) {
  UsbOtgInEpRegs *in_ep = usbInEp(ep_num);
  const uint16_t pkt_cnt = (len == 0U) ? 1U : static_cast<uint16_t>((len + 63U) / 64U);
  in_ep->DIEPTSIZ = (static_cast<uint32_t>(pkt_cnt) << kDieptsizPktcntPos) | len;
  in_ep->DIEPCTL |= kDiepctlCnak | kDiepctlEpena;

  volatile uint32_t *fifo = usbFifo(ep_num);
  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  for (uint16_t i = 0U; i < words; ++i) {
    uint32_t word = 0U;
    copyBytes(&word, src + (i * 4U), sizeof(word));
    *fifo = word;
  }
}

void readPacketFromRxFifo(uint8_t *dst, uint16_t len) {
  volatile uint32_t *fifo = usbFifo(0U);
  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  uint16_t copied = 0U;
  for (uint16_t i = 0U; i < words; ++i) {
    const uint32_t word = *fifo;
    const uint16_t rem = static_cast<uint16_t>(len - copied);
    const uint16_t chunk = (rem >= 4U) ? 4U : rem;
    if ((dst != NULL) && (chunk > 0U)) {
      copyBytes(dst + copied, &word, chunk);
    }
    copied = static_cast<uint16_t>(copied + chunk);
  }
}

void armEp0OutSetup() {
  UsbOtgOutEpRegs *out0 = usbOutEp(0U);
  out0->DOEPTSIZ = (1U << kDoeptsiz0StupcntPos) |
                   (1U << kDoeptsiz0PktcntPos) |
                   kEp0MaxPacketSize;
  out0->DOEPCTL |= kDoepctlCnak | kDoepctlEpena;
}

void armBulkOutEp1() {
  UsbOtgOutEpRegs *out1 = usbOutEp(1U);
  out1->DOEPTSIZ = (1U << kDoeptsiz1PktcntPos) | kBulkMaxPacketSize;
  out1->DOEPCTL = kDoepctlEpena | kDoepctlCnak |
                  (2U << kDoepctlEptypPos) | kBulkMaxPacketSize;
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

  UsbOtgGlobalRegs *usb = usbGlobal();
  UsbOtgDeviceRegs *dev = usbDevice();

  dev->DCFG &= ~kDcfgDadMask;

  usb->GRXFSIZ = 128U;
  usb->DIEPTXF0_HNPTXFSIZ = (64U << 16U) | 128U;
  usb->DIEPTXF[0U] = (64U << 16U) | 192U;
  usb->DIEPTXF[1U] = (32U << 16U) | 256U;

  dev->DAINTMSK = (1U << 0U) | (1U << 1U) | (1U << 2U) |
                  (1U << 16U) | (1U << 17U);
  dev->DOEPMSK = kDoepmskXfrcm | kDoepmskStupm;
  dev->DIEPMSK = kDiepmskXfrcm;

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
      UsbOtgInEpRegs *in1 = usbInEp(1U);
      in1->DIEPCTL = kDiepctlUsbaep | kDiepctlSd0pid |
                     (2U << kDiepctlEptypPos) |
                     (1U << kDiepctlTxfnumPos) | kBulkMaxPacketSize;

      UsbOtgInEpRegs *in2 = usbInEp(2U);
      in2->DIEPCTL = kDiepctlUsbaep | kDiepctlSd0pid |
                     (3U << kDiepctlEptypPos) |
                     (2U << kDiepctlTxfnumPos) | kCmdMaxPacketSize;

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

  usbInEp(0U)->DIEPCTL |= kDiepctlStall;
  armEp0OutSetup();
}

void initUsbCdcFs() {
  rcc()->AHB1ENR |= kRccAhb1EnGpioa;
  GpioRegs *pa = gpioA();
  pa->MODER &= ~((3U << (11U * 2U)) | (3U << (12U * 2U)));
  pa->MODER |= ((2U << (11U * 2U)) | (2U << (12U * 2U)));
  pa->OSPEEDR |= ((3U << (11U * 2U)) | (3U << (12U * 2U)));
  pa->AFR[1U] &= ~((0xFU << ((11U - 8U) * 4U)) | (0xFU << ((12U - 8U) * 4U)));
  pa->AFR[1U] |= ((10U << ((11U - 8U) * 4U)) | (10U << ((12U - 8U) * 4U)));

  rcc()->AHB2ENR |= kRccAhb2EnOtgfs;

  UsbOtgGlobalRegs *usb = usbGlobal();
  UsbOtgDeviceRegs *dev = usbDevice();

  usb->GRSTCTL |= kGrstctlCsrst;
  while ((usb->GRSTCTL & kGrstctlCsrst) != 0U) {
  }

  usb->GUSBCFG |= kGusbcflgFdmod;
  for (volatile uint32_t i = 0U; i < 200000U; ++i) {
  }

  dev->DCFG |= (3U << kDcfgDspdPos);
  usb->GCCFG |= kGccfgPwrdwn | kGccfgVbden;

  usb->GINTMSK = kGintmskUsbreset |
                 kGintmskRxflvl |
                 kGintmskIepint |
                 kGintmskOepint;
  usb->GAHBCFG |= kGahbcfgGint;

  setIrqPriority(kOtgFsIrqn, 5U);
  enableIrq(kOtgFsIrqn);
}

bool cdcSendBytes(const uint8_t *data, uint16_t len) {
  if (!g_usb_configured || (data == NULL) || (len == 0U) || (len > kBulkMaxPacketSize)) {
    return false;
  }

  disableIrqGlobal();
  if (g_cdc_tx_busy) {
    enableIrqGlobal();
    return false;
  }
  g_cdc_tx_busy = true;
  copyBytes(g_cdc_tx_buf, data, len);
  writePacketToTxFifo(1U, g_cdc_tx_buf, len);
  enableIrqGlobal();

  return true;
}

class PeriodicCdcHello {
public:
  void configure(const char *msg, uint32_t interval_ms) {
    msg_ = msg;
    interval_ms_ = interval_ms;
    last_send_ms_ = 0U;
  }

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
  const char *msg_ = NULL;
  uint32_t interval_ms_ = 2000U;
  uint32_t last_send_ms_ = 0U;
};

OnboardLed g_led;
PeriodicCdcHello g_hello;

}  // namespace

extern "C" void SystemInit(void) {
  scb()->CPACR |= kScbCpacrFpuEnable;
  scb()->VTOR = kFlashStartAddr;
}

extern "C" void SysTick_Handler(void) {
  ++g_millis;
}

extern "C" void OTG_FS_IRQHandler(void) {
  UsbOtgGlobalRegs *usb = usbGlobal();
  UsbOtgDeviceRegs *dev = usbDevice();

  const uint32_t int_status = usb->GINTSTS & usb->GINTMSK;

  if ((int_status & kGintstsUsbreset) != 0U) {
    handleUsbReset();
    usb->GINTSTS = kGintstsUsbreset;
  }

  if ((int_status & kGintstsRxflvl) != 0U) {
    const uint32_t rx_status = usb->GRXSTSP;
    const uint8_t ep_num = static_cast<uint8_t>(rx_status & kGrxstspEpnumMask);
    const uint16_t byte_cnt = static_cast<uint16_t>(
        (rx_status & kGrxstspBcntMask) >> kGrxstspBcntPos);
    const uint8_t pkt_sts = static_cast<uint8_t>(
        (rx_status & kGrxstspPktstsMask) >> kGrxstspPktstsPos);

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

  if ((int_status & kGintstsOepint) != 0U) {
    const uint32_t oep_int = dev->DAINT;

    if ((oep_int & (1U << 16U)) != 0U) {
      UsbOtgOutEpRegs *out0 = usbOutEp(0U);
      const uint32_t flags = out0->DOEPINT;
      out0->DOEPINT = flags;

      if ((flags & kDoepintStup) != 0U) {
        UsbSetupPacket setup_pkt{};
        zeroBytes(&setup_pkt, sizeof(setup_pkt));
        copyBytes(&setup_pkt, g_ep0_rx_buf, sizeof(setup_pkt));
        handleSetupPacket(setup_pkt);
      } else if (((flags & kDoepintXfrcm) != 0U) && g_awaiting_line_coding) {
        g_awaiting_line_coding = false;
        copyBytes(&g_line_coding, g_ep0_rx_buf, sizeof(g_line_coding));
        sendEp0ZeroLengthStatus();
      }
    }

    if ((oep_int & (1U << 17U)) != 0U) {
      UsbOtgOutEpRegs *out1 = usbOutEp(1U);
      out1->DOEPINT = out1->DOEPINT;
      armBulkOutEp1();
    }
  }

  if ((int_status & kGintstsIepint) != 0U) {
    const uint32_t iep_int = dev->DAINT;

    if ((iep_int & (1U << 0U)) != 0U) {
      UsbOtgInEpRegs *in0 = usbInEp(0U);
      const uint32_t flags = in0->DIEPINT;
      in0->DIEPINT = flags;

      if ((flags & kDiepintXfrcm) != 0U) {
        if (g_has_pending_address) {
          dev->DCFG &= ~kDcfgDadMask;
          dev->DCFG |= (static_cast<uint32_t>(g_pending_address) << kDcfgDadPos);
          g_has_pending_address = false;
        }
        if (g_ep0_tx_remaining > 0U) {
          const uint16_t chunk = (g_ep0_tx_remaining > kEp0MaxPacketSize)
                                      ? kEp0MaxPacketSize
                                      : g_ep0_tx_remaining;
          const uint8_t *next_ptr = const_cast<const uint8_t *>(g_ep0_tx_ptr);
          writePacketToTxFifo(0U, next_ptr, chunk);
          g_ep0_tx_ptr = next_ptr + chunk;
          g_ep0_tx_remaining = static_cast<uint16_t>(g_ep0_tx_remaining - chunk);
        }
      }
    }

    if ((iep_int & (1U << 1U)) != 0U) {
      UsbOtgInEpRegs *in1 = usbInEp(1U);
      in1->DIEPINT = in1->DIEPINT;
      g_cdc_tx_busy = false;
    }

    if ((iep_int & (1U << 2U)) != 0U) {
      UsbOtgInEpRegs *in2 = usbInEp(2U);
      in2->DIEPINT = in2->DIEPINT;
    }
  }
}

extern "C" int _close(int file) {
  return -1;
}

extern "C" int _lseek(int file, int ptr, int dir) {
  return 0;
}

extern "C" int _read(int file, char *ptr, int len) {
  return 0;
}

extern "C" int _write(int file, char *ptr, int len) {
  return len;
}

int main(void) {
  configureClock96MhzUsb48Mhz();
  initSysTick1ms();

  const uint32_t start_ms = millis();
  g_led.configure(gpioC(), 13U, kLedActiveLow, kLedBlinkIntervalMs);
  g_led.init(start_ms);

  g_hello.configure(kHelloPayload, kCdcSendIntervalMs);
  g_hello.init(start_ms);

  initUsbCdcFs();

  while (1) {
    const uint32_t now_ms = millis();
    g_led.update(now_ms);
    g_hello.update(now_ms);
  }
}
