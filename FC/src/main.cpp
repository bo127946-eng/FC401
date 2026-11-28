#include <stddef.h>
#include <stdint.h>

namespace {

constexpr uint32_t kSysClkHz      = 48000000U;
constexpr uint32_t kBlinkPeriodMs = 1000U;
constexpr uint32_t kHelloPeriodMs = 2000U;
constexpr uint32_t kClockTimeout  = 500000U;

constexpr char kMsg25M[] = "SRC: 25MHz | SYS: 48MHz\r\n";
constexpr char kMsg16M[] = "SRC: 16MHz | SYS: 48MHz\r\n";

constexpr uintptr_t kAhb1Base  = 0x40020000UL;
constexpr uintptr_t kAhb2Base  = 0x50000000UL;
constexpr uintptr_t kFlashBase = 0x08000000UL;
constexpr uintptr_t kUsbBase   = kAhb2Base + 0x00000UL;
constexpr uintptr_t kScsBase   = 0xE000E000UL;

constexpr uint8_t kEp0Mps    = 64U;
constexpr uint8_t kBulkMps   = 64U;
constexpr uint8_t kCmdMps    = 8U;
constexpr int32_t kOtgFsIrqn = 67;

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
  uint32_t          RES0;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t          RES1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t          RES2;
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
  uint32_t          RES0[18];
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
  uint32_t          RES0[24];
  volatile uint32_t ICER[8];
  uint32_t          RES1[24];
  volatile uint32_t ISPR[8];
  uint32_t          RES2[24];
  volatile uint32_t ICPR[8];
  uint32_t          RES3[24];
  volatile uint32_t IABR[8];
  uint32_t          RES4[56];
  volatile uint8_t  IP[240];
};

struct UsbGlobalRegs {
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
  volatile uint32_t DIEPTXF0;
  volatile uint32_t HNPTXSTS;
  uint32_t          RES0[2];
  volatile uint32_t GCCFG;
  volatile uint32_t CID;
  uint32_t          RES1[48];
  volatile uint32_t HPTXFSIZ;
  volatile uint32_t DIEPTXF[15];
};

struct UsbDeviceRegs {
  volatile uint32_t DCFG;
  volatile uint32_t DCTL;
  volatile uint32_t DSTS;
  uint32_t          RES0;
  volatile uint32_t DIEPMSK;
  volatile uint32_t DOEPMSK;
  volatile uint32_t DAINT;
  volatile uint32_t DAINTMSK;
};

struct UsbInEpRegs {
  volatile uint32_t DIEPCTL;
  uint32_t          RES0;
  volatile uint32_t DIEPINT;
  uint32_t          RES1;
  volatile uint32_t DIEPTSIZ;
  volatile uint32_t DIEPDMA;
  volatile uint32_t DTXFSTS;
  uint32_t          RES2;
};

struct UsbOutEpRegs {
  volatile uint32_t DOEPCTL;
  uint32_t          RES0;
  volatile uint32_t DOEPINT;
  uint32_t          RES1;
  volatile uint32_t DOEPTSIZ;
  volatile uint32_t DOEPDMA;
  uint32_t          RES2[2];
};

struct __attribute__((packed)) SetupPkt {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
};

struct __attribute__((packed)) LineCoding {
  uint32_t dwDTERate;
  uint8_t  bCharFormat;
  uint8_t  bParityType;
  uint8_t  bDataBits;
};

inline GpioRegs *gpioA()      { return reinterpret_cast<GpioRegs *>(kAhb1Base + 0x0000UL); }
inline GpioRegs *gpioC()      { return reinterpret_cast<GpioRegs *>(kAhb1Base + 0x0800UL); }
inline RccRegs *rcc()         { return reinterpret_cast<RccRegs *>(kAhb1Base + 0x3800UL); }
inline FlashRegs *flash()     { return reinterpret_cast<FlashRegs *>(kAhb1Base + 0x3C00UL); }
inline ScbRegs *scb()         { return reinterpret_cast<ScbRegs *>(kScsBase + 0x0D00UL); }
inline SysTickRegs *systick() { return reinterpret_cast<SysTickRegs *>(kScsBase + 0x0010UL); }
inline NvicRegs *nvic()       { return reinterpret_cast<NvicRegs *>(kScsBase + 0x0100UL); }
inline UsbGlobalRegs *usbG()  { return reinterpret_cast<UsbGlobalRegs *>(kUsbBase); }
inline UsbDeviceRegs *usbD()  { return reinterpret_cast<UsbDeviceRegs *>(kUsbBase + 0x800UL); }
inline UsbInEpRegs *epIn(uint8_t n) {
  return reinterpret_cast<UsbInEpRegs *>(kUsbBase + 0x900UL + (n * 0x20UL));
}
inline UsbOutEpRegs *epOut(uint8_t n) {
  return reinterpret_cast<UsbOutEpRegs *>(kUsbBase + 0xB00UL + (n * 0x20UL));
}
inline volatile uint32_t *fifo(uint8_t n) {
  return reinterpret_cast<volatile uint32_t *>(kUsbBase + 0x1000UL + (n * 0x1000UL));
}

constexpr uint8_t kDevDesc[] = {
    18U, 0x01U, 0x00U, 0x02U, 0x02U, 0x02U, 0x00U, kEp0Mps,
    0x83U, 0x04U, 0x40U, 0x57U, 0x00U, 0x02U, 1U, 2U, 3U, 1U
};

constexpr uint8_t kCfgDesc[] = {
    9U, 0x02U, 67U, 0x00U, 2U, 1U, 0U, 0xC0U, 50U,
    9U, 0x04U, 0U, 0U, 1U, 0x02U, 0x02U, 0x01U, 0U,
    5U, 0x24U, 0x00U, 0x10U, 0x01U,
    5U, 0x24U, 0x01U, 0x00U, 0x01U,
    4U, 0x24U, 0x02U, 0x02U,
    5U, 0x24U, 0x06U, 0x00U, 0x01U,
    7U, 0x05U, 0x82U, 0x03U, kCmdMps, 0x00U, 16U,
    9U, 0x04U, 1U, 0U, 2U, 0x0AU, 0x00U, 0x00U, 0U,
    7U, 0x05U, 0x01U, 0x02U, kBulkMps, 0x00U, 0U,
    7U, 0x05U, 0x81U, 0x02U, kBulkMps, 0x00U, 0U
};

constexpr uint8_t kLangDesc[] = { 4U, 0x03U, 0x09U, 0x04U };
constexpr uint8_t kProdDesc[] = {
    22U, 0x03U,
    'B', 0, 'l', 0, 'a', 0, 'c', 0, 'k', 0, 'P', 0, 'i', 0, 'l', 0, 'l', 0
};

volatile uint32_t g_ms = 0U;
volatile bool     g_configured = false;
volatile bool     g_tx_busy = false;
volatile bool     g_has_addr = false;
volatile uint8_t  g_addr = 0U;
volatile bool     g_set_lc = false;

LineCoding g_lc = {115200U, 0U, 0U, 8U};
uint8_t    g_ep0_buf[kEp0Mps];
uint8_t    g_rx_buf[kBulkMps];
uint8_t    g_tx_buf[kBulkMps];

void copy(void *dst, const void *src, size_t n) {
  uint8_t *d = static_cast<uint8_t *>(dst);
  const uint8_t *s = static_cast<const uint8_t *>(src);
  for (size_t i = 0U; i < n; ++i) d[i] = s[i];
}

size_t strLen(const char *s) {
  size_t n = 0U;
  while (s[n] != '\0') ++n;
  return n;
}

void ledInit() {
  rcc()->AHB1ENR |= (1U << 2U);  // GPIOCEN
  gpioC()->MODER &= ~(3U << (13U * 2U));
  gpioC()->MODER |= (1U << (13U * 2U));
  gpioC()->BSRR = (1U << 13U);  // LED off (active low)
}

void ledSet(bool on) {
  gpioC()->BSRR = on ? (1U << (13U + 16U)) : (1U << 13U);
}

void spinWait(volatile uint32_t n) {
  while (n > 0U) --n;
}

bool waitFlagSet(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
  while (((*reg & mask) == 0U) && (timeout > 0U)) --timeout;
  return ((*reg & mask) != 0U);
}

bool waitFlagClear(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
  while (((*reg & mask) != 0U) && (timeout > 0U)) --timeout;
  return ((*reg & mask) == 0U);
}

bool clockInit48Mhz() {
  flash()->ACR = (1U << 0U) | (1U << 8U) | (1U << 9U) | (1U << 10U);

  rcc()->CR |= (1U << 16U);  // HSEON (25 MHz)
  const bool hse_ok = waitFlagSet(&rcc()->CR, (1U << 17U), kClockTimeout);

  rcc()->CR &= ~(1U << 24U);  // PLLOFF
  waitFlagClear(&rcc()->CR, (1U << 25U), kClockTimeout);

  if (hse_ok) {
    // 25 MHz -> VCO 192 MHz -> SYSCLK 48 MHz (/4), USB48 48 MHz (/4)
    rcc()->PLLCFGR = (25U << 0U) | (192U << 6U) | (1U << 16U) | (1U << 22U) | (4U << 24U);
  } else {
    // 16 MHz -> VCO 192 MHz -> SYSCLK 48 MHz (/4), USB48 48 MHz (/4)
    rcc()->CR |= (1U << 0U);
    waitFlagSet(&rcc()->CR, (1U << 1U), kClockTimeout);
    rcc()->PLLCFGR = (16U << 0U) | (192U << 6U) | (1U << 16U) | (0U << 22U) | (4U << 24U);
  }

  rcc()->CR |= (1U << 24U);  // PLLON
  if (waitFlagSet(&rcc()->CR, (1U << 25U), kClockTimeout)) {
    rcc()->CFGR = (rcc()->CFGR & ~3U) | 2U;  // SW = PLL
    waitFlagSet(&rcc()->CFGR, (2U << 2U), kClockTimeout);
  }
  return hse_ok;
}

void tickInit() {
  systick()->LOAD = (kSysClkHz / 1000U) - 1U;
  systick()->VAL = 0U;
  systick()->CTRL = (1U << 2U) | (1U << 1U) | (1U << 0U);
}

uint32_t millis() {
  __asm volatile("cpsid i" ::: "memory");
  const uint32_t t = g_ms;
  __asm volatile("cpsie i" ::: "memory");
  return t;
}

void writeFifo(uint8_t ep, const uint8_t *src, uint16_t len) {
  UsbInEpRegs *in = epIn(ep);
  const uint16_t pkt = (len == 0U) ? 1U : static_cast<uint16_t>((len + 63U) / 64U);
  in->DIEPTSIZ = (static_cast<uint32_t>(pkt) << 19U) | len;
  in->DIEPCTL |= (1U << 26U) | (1U << 31U);

  volatile uint32_t *f = fifo(ep);
  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  for (uint16_t i = 0U; i < words; ++i) {
    uint32_t w = 0U;
    copy(&w, src + (i * 4U), sizeof(w));
    *f = w;
  }
}

void readFifo(uint8_t *dst, uint16_t len) {
  volatile uint32_t *f = fifo(0U);
  const uint16_t words = static_cast<uint16_t>((len + 3U) / 4U);
  uint16_t off = 0U;
  for (uint16_t i = 0U; i < words; ++i) {
    const uint32_t w = *f;
    const uint16_t rem = static_cast<uint16_t>(len - off);
    const uint16_t n = (rem >= 4U) ? 4U : rem;
    if (dst && n > 0U) copy(dst + off, &w, n);
    off = static_cast<uint16_t>(off + n);
  }
}

void armEp0() {
  UsbOutEpRegs *out0 = epOut(0U);
  out0->DOEPTSIZ = (1U << 29U) | (1U << 19U) | kEp0Mps;
  out0->DOEPCTL |= (1U << 26U) | (1U << 31U);
}

void armEp1Out() {
  UsbOutEpRegs *out1 = epOut(1U);
  out1->DOEPTSIZ = (1U << 19U) | kBulkMps;
  out1->DOEPCTL = (1U << 31U) | (1U << 26U) | (2U << 18U) | kBulkMps;
}

void ep0Send(const uint8_t *buf, uint16_t len, uint16_t req_len) {
  const uint16_t n = (len < req_len) ? len : req_len;
  const uint16_t chunk = (n > kEp0Mps) ? kEp0Mps : n;
  writeFifo(0U, buf, chunk);
  armEp0();
}

void ep0Ack() {
  writeFifo(0U, NULL, 0U);
  armEp0();
}

void usbReset() {
  g_configured = false;
  g_tx_busy = false;
  g_has_addr = false;
  g_set_lc = false;
  usbD()->DCFG &= ~(0x7FU << 4U);

  usbG()->GRXFSIZ = 128U;
  usbG()->DIEPTXF0 = (64U << 16U) | 128U;
  usbG()->DIEPTXF[0U] = (64U << 16U) | 192U;
  usbG()->DIEPTXF[1U] = (32U << 16U) | 256U;

  usbD()->DAINTMSK = (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 16U) | (1U << 17U);
  usbD()->DOEPMSK = (1U << 0U) | (1U << 3U);
  usbD()->DIEPMSK = (1U << 0U);
  armEp0();
}

void handleSetup(const SetupPkt &p) {
  if (p.bRequest == 0x06U) {
    const uint8_t t = static_cast<uint8_t>(p.wValue >> 8U);
    if (t == 0x01U) return ep0Send(kDevDesc, sizeof(kDevDesc), p.wLength);
    if (t == 0x02U) return ep0Send(kCfgDesc, sizeof(kCfgDesc), p.wLength);
    if (t == 0x03U) {
      return ep0Send((p.wValue & 0xFFU) == 0U ? kLangDesc : kProdDesc,
                     (p.wValue & 0xFFU) == 0U ? sizeof(kLangDesc) : sizeof(kProdDesc),
                     p.wLength);
    }
  } else if (p.bRequest == 0x05U) {
    g_addr = static_cast<uint8_t>(p.wValue & 0x7FU);
    g_has_addr = true;
    return ep0Ack();
  } else if (p.bRequest == 0x09U) {
    g_configured = ((p.wValue & 0xFFU) != 0U);
    if (g_configured) {
      epIn(1U)->DIEPCTL = (1U << 15U) | (1U << 28U) | (2U << 18U) | (1U << 22U) | kBulkMps;
      epIn(2U)->DIEPCTL = (1U << 15U) | (1U << 28U) | (3U << 18U) | (2U << 22U) | kCmdMps;
      armEp1Out();
    }
    return ep0Ack();
  } else if (p.bRequest == 0x20U) {
    g_set_lc = true;
    armEp0();
    return;
  } else if (p.bRequest == 0x21U) {
    return ep0Send(reinterpret_cast<const uint8_t *>(&g_lc), sizeof(g_lc), p.wLength);
  } else if (p.bRequest == 0x22U) {
    return ep0Ack();
  }
  epIn(0U)->DIEPCTL |= (1U << 21U);
  armEp0();
}

void usbInit() {
  rcc()->AHB1ENR |= (1U << 0U);  // GPIOAEN
  gpioA()->MODER = (gpioA()->MODER & ~((3U << 22U) | (3U << 24U))) | ((2U << 22U) | (2U << 24U));
  gpioA()->OSPEEDR |= (3U << 22U) | (3U << 24U);
  gpioA()->AFR[1U] = (gpioA()->AFR[1U] & ~((0xFU << 12U) | (0xFU << 16U))) |
                     ((10U << 12U) | (10U << 16U));

  rcc()->AHB2ENR |= (1U << 7U);  // OTGFSEN
  usbG()->GRSTCTL |= (1U << 0U);
  waitFlagClear(&usbG()->GRSTCTL, (1U << 0U), kClockTimeout);

  usbG()->GUSBCFG |= (1U << 30U);  // FDMOD
  spinWait(200000U);

  usbD()->DCFG = (usbD()->DCFG & ~3U) | 3U;
  usbD()->DCTL &= ~(1U << 1U);  // Clear SDIS
  usbG()->GCCFG |= (1U << 16U) | (1U << 21U);  // PWRDWN | NOVBUSSENS
  usbG()->GINTMSK = (1U << 12U) | (1U << 4U) | (1U << 18U) | (1U << 19U);
  usbG()->GAHBCFG |= (1U << 0U);

  nvic()->IP[kOtgFsIrqn] = (5U << 4U);
  nvic()->ISER[kOtgFsIrqn >> 5U] = (1U << (kOtgFsIrqn & 31U));
}

bool cdcSend(const char *s) {
  if (!g_configured || g_tx_busy) return false;
  const size_t n = strLen(s);
  if (n == 0U || n > kBulkMps) return false;

  __asm volatile("cpsid i" ::: "memory");
  g_tx_busy = true;
  copy(g_tx_buf, s, n);
  writeFifo(1U, g_tx_buf, static_cast<uint16_t>(n));
  __asm volatile("cpsie i" ::: "memory");
  return true;
}

}  // namespace

extern "C" void SystemInit(void) {
  scb()->CPACR |= (0xFU << 20U);
  scb()->VTOR = kFlashBase;
}

extern "C" void SysTick_Handler(void) { ++g_ms; }

extern "C" void OTG_FS_IRQHandler(void) {
  const uint32_t st = usbG()->GINTSTS & usbG()->GINTMSK;

  if (st & (1U << 12U)) {
    usbReset();
    usbG()->GINTSTS = (1U << 12U);
  }
  if (st & (1U << 4U)) {
    const uint32_t rx = usbG()->GRXSTSP;
    const uint8_t ep = static_cast<uint8_t>(rx & 0x0FU);
    const uint16_t cnt = static_cast<uint16_t>((rx >> 4U) & 0x7FFU);
    const uint8_t pkt = static_cast<uint8_t>((rx >> 17U) & 0x0FU);
    if (pkt == 6U || (pkt == 2U && ep == 0U)) readFifo(g_ep0_buf, cnt);
    else if (pkt == 2U && ep == 1U) readFifo(g_rx_buf, cnt > kBulkMps ? kBulkMps : cnt);
    else if (pkt == 2U) readFifo(NULL, cnt);
  }
  if (st & (1U << 19U)) {
    const uint32_t da = usbD()->DAINT;
    if (da & (1U << 16U)) {
      UsbOutEpRegs *o0 = epOut(0U);
      const uint32_t f = o0->DOEPINT;
      o0->DOEPINT = f;
      if (f & (1U << 3U)) {
        SetupPkt p{};
        copy(&p, g_ep0_buf, sizeof(p));
        handleSetup(p);
      } else if ((f & (1U << 0U)) && g_set_lc) {
        g_set_lc = false;
        copy(&g_lc, g_ep0_buf, sizeof(g_lc));
        ep0Ack();
      }
    }
    if (da & (1U << 17U)) {
      UsbOutEpRegs *o1 = epOut(1U);
      o1->DOEPINT = o1->DOEPINT;
      armEp1Out();
    }
  }
  if (st & (1U << 18U)) {
    const uint32_t da = usbD()->DAINT;
    if (da & (1U << 0U)) {
      UsbInEpRegs *i0 = epIn(0U);
      const uint32_t f = i0->DIEPINT;
      i0->DIEPINT = f;
      if ((f & (1U << 0U)) && g_has_addr) {
        usbD()->DCFG = (usbD()->DCFG & ~(0x7FU << 4U)) | (static_cast<uint32_t>(g_addr) << 4U);
        g_has_addr = false;
      }
    }
    if (da & (1U << 1U)) {
      epIn(1U)->DIEPINT = epIn(1U)->DIEPINT;
      g_tx_busy = false;
    }
    if (da & (1U << 2U)) epIn(2U)->DIEPINT = epIn(2U)->DIEPINT;
  }
}

extern "C" int _close(int) { return -1; }
extern "C" int _lseek(int, int, int) { return 0; }
extern "C" int _read(int, char *, int) { return 0; }
extern "C" int _write(int, char *, int n) { return n; }

int main(void) {
  // Стартовая вспышка ~1 сек на внутреннем HSI (16 МГц)
  ledInit();
  ledSet(true);
  spinWait(1500000U);
  ledSet(false);

  const bool hse_ok = clockInit48Mhz();
  const char *msg = hse_ok ? kMsg25M : kMsg16M;

  tickInit();
  usbInit();

  uint32_t last_led = millis();
  uint32_t last_tx = last_led;
  bool led_on = false;

  while (1) {
    const uint32_t now = millis();
    if ((now - last_led) >= kBlinkPeriodMs) {
      last_led = now;
      led_on = !led_on;
      ledSet(led_on);
    }
    if ((now - last_tx) >= kHelloPeriodMs) {
      if (cdcSend(msg)) last_tx = now;
    }
  }
}
