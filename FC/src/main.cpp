#include <stddef.h>
#include <stdint.h>

namespace {

constexpr uint32_t kSysClkHz      = 48000000U;
constexpr uint32_t kBlinkPeriodMs = 1000U;
constexpr uint32_t kHelloPeriodMs = 2000U;
constexpr uint32_t kClockTimeout  = 1500000U;
constexpr uint32_t kStepSpin16M   = 1500000U;
constexpr uint32_t kStepSpin48M   = kStepSpin16M * 3U;

constexpr uintptr_t kAhb1Base     = 0x40020000UL;
constexpr uintptr_t kAhb2Base     = 0x50000000UL;
constexpr uintptr_t kFlashBase    = 0x08000000UL;
constexpr uintptr_t kUsbBase      = kAhb2Base + 0x00000UL;
constexpr uintptr_t kScsBase      = 0xE000E000UL;
constexpr uintptr_t kDbgIdcodeAdr = 0xE0042000UL;
constexpr uintptr_t kFlashSizeAdr = 0x1FFF7A22UL;

constexpr uint8_t kEp0Mps    = 64U;
constexpr uint8_t kBulkMps   = 64U;
constexpr uint8_t kCmdMps    = 8U;
constexpr int32_t kOtgFsIrqn = 67;

enum class ChipType : uint8_t {
  Unknown = 0U,
  F401    = 1U,
  F411    = 2U
};

struct XtalPllCfg {
  uint16_t mhz;
  uint32_t pll_m;
  uint32_t pll_n;
  uint32_t pll_p_div;
  uint32_t pll_q;
};

constexpr XtalPllCfg kXtalCandidates[] = {
    {25U, 25U, 192U, 4U, 4U},
    {8U,   8U, 192U, 4U, 4U},
    {12U, 12U, 192U, 4U, 4U},
    {16U, 16U, 192U, 4U, 4U},
};

struct DiagInfo {
  ChipType chip;
  uint16_t dev_id;
  uint16_t flash_kb;
  uint16_t src_mhz;
  bool     from_hsi;
};

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

void appendStr(char *dst, size_t &pos, size_t cap, const char *s) {
  for (size_t i = 0U; s[i] != '\0' && pos + 1U < cap; ++i) {
    dst[pos++] = s[i];
  }
  dst[pos] = '\0';
}

void appendUint(char *dst, size_t &pos, size_t cap, uint32_t val) {
  char tmp[12];
  size_t n = 0U;
  if (val == 0U) {
    tmp[n++] = '0';
  } else {
    while (val > 0U && n < sizeof(tmp)) {
      tmp[n++] = static_cast<char>('0' + (val % 10U));
      val /= 10U;
    }
  }
  while (n > 0U) {
    --n;
    if (pos + 1U < cap) dst[pos++] = tmp[n];
  }
  dst[pos] = '\0';
}

void appendHex16(char *dst, size_t &pos, size_t cap, uint16_t val) {
  static const char kHex[] = "0123456789ABCDEF";
  appendStr(dst, pos, cap, "0x");
  for (int shift = 12; shift >= 0; shift -= 4) {
    if (pos + 1U < cap) dst[pos++] = kHex[(val >> shift) & 0xFU];
  }
  dst[pos] = '\0';
}

void ledInit() {
  rcc()->AHB1ENR |= (1U << 2U);
  gpioC()->MODER &= ~(3U << (13U * 2U));
  gpioC()->MODER |= (1U << (13U * 2U));
  gpioC()->BSRR = (1U << 13U);
}

void ledSet(bool on) {
  gpioC()->BSRR = on ? (1U << (13U + 16U)) : (1U << 13U);
}

void spinWait(volatile uint32_t n) {
  while (n > 0U) --n;
}

void blinkOnce(uint32_t spin_count) {
  ledSet(true);
  spinWait(spin_count);
  ledSet(false);
  spinWait(spin_count);
}

uint16_t readDevId() {
  const uint32_t raw = *reinterpret_cast<const volatile uint32_t *>(kDbgIdcodeAdr);
  return static_cast<uint16_t>(raw & 0x0FFFU);
}

ChipType classifyChip(uint16_t dev_id) {
  if (dev_id == 0x0431U) return ChipType::F411;
  if (dev_id == 0x0423U || dev_id == 0x0433U) return ChipType::F401;
  return ChipType::Unknown;
}

uint16_t detectFlashKb(ChipType chip) {
  const uint16_t kb = *reinterpret_cast<const volatile uint16_t *>(kFlashSizeAdr);
  if (kb > 0U && kb <= 2048U) return kb;
  return (chip == ChipType::F411) ? 512U : 256U;
}

const char *chipName(ChipType chip) {
  if (chip == ChipType::F411) return "F411";
  if (chip == ChipType::F401) return "F401";
  return "F4xx";
}

bool waitFlagSet(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
  while (((*reg & mask) == 0U) && (timeout > 0U)) --timeout;
  return ((*reg & mask) != 0U);
}

bool waitFlagClear(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
  while (((*reg & mask) != 0U) && (timeout > 0U)) --timeout;
  return ((*reg & mask) == 0U);
}

bool startPll48FromCfg(uint32_t pll_m, uint32_t pll_n, uint32_t pll_p_div, uint32_t pll_q,
                       bool use_hse) {
  const uint32_t pllp_bits = ((pll_p_div / 2U) - 1U) & 3U;
  const uint32_t pllsrc_bit = use_hse ? (1U << 22U) : 0U;

  rcc()->CR &= ~(1U << 24U);
  waitFlagClear(&rcc()->CR, (1U << 25U), kClockTimeout);

  rcc()->PLLCFGR = (pll_m << 0U) | (pll_n << 6U) | (pllp_bits << 16U) |
                   pllsrc_bit | (pll_q << 24U);

  rcc()->CR |= (1U << 24U);
  if (!waitFlagSet(&rcc()->CR, (1U << 25U), kClockTimeout)) return false;

  rcc()->CFGR = (rcc()->CFGR & ~3U) | 2U;
  uint32_t sw_t = kClockTimeout;
  while ((((rcc()->CFGR >> 2U) & 3U) != 2U) && (sw_t > 0U)) --sw_t;
  return (((rcc()->CFGR >> 2U) & 3U) == 2U);
}

void clockInitAuto48Mhz(DiagInfo &info) {
  flash()->ACR = (1U << 0U) | (1U << 8U) | (1U << 9U) | (1U << 10U);

  rcc()->CR |= (1U << 16U);
  const bool hse_rdy = waitFlagSet(&rcc()->CR, (1U << 17U), kClockTimeout);

  if (hse_rdy) {
    for (size_t i = 0U; i < (sizeof(kXtalCandidates) / sizeof(kXtalCandidates[0])); ++i) {
      const XtalPllCfg &c = kXtalCandidates[i];
      if (startPll48FromCfg(c.pll_m, c.pll_n, c.pll_p_div, c.pll_q, true)) {
        info.src_mhz = c.mhz;
        info.from_hsi = false;
        return;
      }
    }
  }

  rcc()->CR |= (1U << 0U);
  waitFlagSet(&rcc()->CR, (1U << 1U), kClockTimeout);
  startPll48FromCfg(16U, 192U, 4U, 4U, false);
  info.src_mhz = 16U;
  info.from_hsi = true;
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
    if (src != NULL) {
      const size_t off = static_cast<size_t>(i) * 4U;
      const size_t rem = (len > off) ? static_cast<size_t>(len - off) : 0U;
      const size_t n = (rem >= 4U) ? 4U : rem;
      if (n > 0U) copy(&w, src + off, n);
    }
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
    if (dst != NULL && n > 0U) copy(dst + off, &w, n);
    off = static_cast<uint16_t>(off + n);
  }
}

void armEp0() {
  UsbOutEpRegs *out0 = epOut(0U);
  out0->DOEPTSIZ = (1U << 29U) | (1U << 19U) | kEp0Mps;
  out0->DOEPCTL = (out0->DOEPCTL & ~3U) | (1U << 26U) | (1U << 31U);
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

void setupEndpointsAndFifo() {
  g_configured = false;
  g_tx_busy = false;
  g_has_addr = false;
  g_set_lc = false;
  usbD()->DCFG &= ~(0x7FU << 4U);

  epIn(0U)->DIEPINT = 0xFFU;
  epIn(1U)->DIEPINT = 0xFFU;
  epIn(2U)->DIEPINT = 0xFFU;
  epOut(0U)->DOEPINT = 0xFFU;
  epOut(1U)->DOEPINT = 0xFFU;

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

void configureUsbPhyForChip(ChipType chip) {
  if (chip == ChipType::F411) {
    usbG()->GCCFG = (usbG()->GCCFG & ~((1U << 18U) | (1U << 19U) | (1U << 21U))) | (1U << 16U);
  } else {
    usbG()->GCCFG = (usbG()->GCCFG & ~((1U << 18U) | (1U << 19U))) | (1U << 16U) | (1U << 21U);
  }
}

void usbInit(const DiagInfo &info) {
  // Этап 5 (48 МГц): GPIOA (PA11/PA12) настроен
  rcc()->AHB1ENR |= (1U << 0U);
  gpioA()->MODER = (gpioA()->MODER & ~((3U << 22U) | (3U << 24U))) | ((2U << 22U) | (2U << 24U));
  gpioA()->OSPEEDR |= (3U << 22U) | (3U << 24U);
  gpioA()->AFR[1U] = (gpioA()->AFR[1U] & ~((0xFU << 12U) | (0xFU << 16U))) |
                     ((10U << 12U) | (10U << 16U));
  blinkOnce(kStepSpin48M);

  // Этап 6 (48 МГц): OTG_FS включен, сброшен, FDMOD и GCCFG под камень заданы
  rcc()->AHB2ENR |= (1U << 7U);
  usbG()->GRSTCTL |= (1U << 0U);
  waitFlagClear(&usbG()->GRSTCTL, (1U << 0U), kClockTimeout);

  usbG()->GUSBCFG = (usbG()->GUSBCFG & ~(1U << 30U)) | (1U << 29U);
  spinWait(600000U);

  configureUsbPhyForChip(info.chip);
  spinWait(300000U);
  blinkOnce(kStepSpin48M);

  // Этап 7 (48 МГц): DCFG, DCTL, FIFO и EP0 готовы
  usbD()->DCFG = (usbD()->DCFG & ~3U) | 3U;
  usbD()->DCTL &= ~(1U << 1U);
  setupEndpointsAndFifo();
  blinkOnce(kStepSpin48M);

  // Этап 8 (48 МГц): флаги очищены, прерывания USB и NVIC включены
  usbG()->GINTSTS = 0xFFFFFFFFU;
  usbG()->GINTMSK = (1U << 12U) | (1U << 4U) | (1U << 18U) | (1U << 19U);
  usbG()->GAHBCFG |= (1U << 0U);

  nvic()->ICPR[kOtgFsIrqn >> 5U] = (1U << (kOtgFsIrqn & 31U));
  nvic()->IP[kOtgFsIrqn] = (5U << 4U);
  nvic()->ISER[kOtgFsIrqn >> 5U] = (1U << (kOtgFsIrqn & 31U));
  blinkOnce(kStepSpin48M);
}

bool cdcSend(const char *s) {
  if (!g_configured || g_tx_busy) return false;
  size_t n = 0U;
  while (s[n] != '\0') ++n;
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

extern "C" void HardFault_Handler(void) {
  ledInit();
  while (1) {
    blinkOnce(750000U);
  }
}

extern "C" void SysTick_Handler(void) { ++g_ms; }

extern "C" void OTG_FS_IRQHandler(void) {
  const uint32_t st = usbG()->GINTSTS & usbG()->GINTMSK;

  if (st & (1U << 12U)) {
    usbG()->GINTSTS = (1U << 12U);
    setupEndpointsAndFifo();
  }
  if (st & (1U << 4U)) {
    const uint32_t rx = usbG()->GRXSTSP;
    const uint8_t ep = static_cast<uint8_t>(rx & 0x0FU);
    const uint16_t cnt = static_cast<uint16_t>((rx >> 4U) & 0x7FFU);
    const uint8_t pkt = static_cast<uint8_t>((rx >> 17U) & 0x0FU);
    if (pkt == 6U || (pkt == 2U && ep == 0U)) {
      readFifo(g_ep0_buf, cnt > kEp0Mps ? kEp0Mps : cnt);
    } else if (pkt == 2U && ep == 1U) {
      readFifo(g_rx_buf, cnt > kBulkMps ? kBulkMps : cnt);
    } else if (pkt == 2U && cnt > 0U) {
      readFifo(NULL, cnt);
    }
  }
  if (st & (1U << 19U)) {
    const uint32_t da = usbD()->DAINT;
    if (da & (1U << 16U)) {
      UsbOutEpRegs *o0 = epOut(0U);
      const uint32_t f = o0->DOEPINT;
      if (f & (1U << 3U)) {
        o0->DOEPINT = (1U << 3U);
        SetupPkt p{};
        copy(&p, g_ep0_buf, sizeof(p));
        handleSetup(p);
      } else if ((f & (1U << 0U)) && g_set_lc) {
        o0->DOEPINT = (1U << 0U);
        g_set_lc = false;
        copy(&g_lc, g_ep0_buf, sizeof(g_lc));
        ep0Ack();
      } else if (f & (1U << 0U)) {
        o0->DOEPINT = (1U << 0U);
      }
    }
    if (da & (1U << 17U)) {
      UsbOutEpRegs *o1 = epOut(1U);
      if (o1->DOEPINT & (1U << 0U)) o1->DOEPINT = (1U << 0U);
      armEp1Out();
    }
  }
  if (st & (1U << 18U)) {
    const uint32_t da = usbD()->DAINT;
    if (da & (1U << 0U)) {
      UsbInEpRegs *i0 = epIn(0U);
      const uint32_t f = i0->DIEPINT;
      if (f & (1U << 0U)) {
        i0->DIEPINT = (1U << 0U);
        if (g_has_addr) {
          usbD()->DCFG = (usbD()->DCFG & ~(0x7FU << 4U)) | (static_cast<uint32_t>(g_addr) << 4U);
          g_has_addr = false;
        }
      }
    }
    if (da & (1U << 1U)) {
      UsbInEpRegs *i1 = epIn(1U);
      if (i1->DIEPINT & (1U << 0U)) {
        i1->DIEPINT = (1U << 0U);
        g_tx_busy = false;
      }
    }
    if (da & (1U << 2U)) {
      UsbInEpRegs *i2 = epIn(2U);
      if (i2->DIEPINT & (1U << 0U)) i2->DIEPINT = (1U << 0U);
    }
  }
}

extern "C" int _close(int) { return -1; }
extern "C" int _lseek(int, int, int) { return 0; }
extern "C" int _read(int, char *, int) { return 0; }
extern "C" int _write(int, char *, int n) { return n; }

int main(void) {
  DiagInfo info{ChipType::Unknown, 0U, 0U, 16U, true};

  // Этап 1 (16 МГц): старт PC13
  ledInit();
  blinkOnce(kStepSpin16M);

  // Этап 2 (16 МГц): определение камня и Flash
  info.dev_id = readDevId();
  info.chip = classifyChip(info.dev_id);
  info.flash_kb = detectFlashKb(info.chip);
  blinkOnce(kStepSpin16M);

  // Этап 3 (48 МГц): кварц 25/8/12/16 МГц или HSI 16 МГц -> 48 МГц
  clockInitAuto48Mhz(info);
  blinkOnce(kStepSpin48M);

  // Этап 4 (48 МГц): запуск SysTick
  tickInit();
  blinkOnce(kStepSpin48M);

  // Этапы 5–8 (48 МГц): подробная инициализация USB CDC внутри usbInit()
  usbInit(info);

  char msg[kBulkMps] = {0};
  size_t pos = 0U;
  appendStr(msg, pos, sizeof(msg), "CHIP: ");
  appendStr(msg, pos, sizeof(msg), chipName(info.chip));
  appendStr(msg, pos, sizeof(msg), "(");
  appendHex16(msg, pos, sizeof(msg), info.dev_id);
  appendStr(msg, pos, sizeof(msg), ") | FL:");
  appendUint(msg, pos, sizeof(msg), info.flash_kb);
  appendStr(msg, pos, sizeof(msg), "K | ");
  appendUint(msg, pos, sizeof(msg), info.src_mhz);
  appendStr(msg, pos, sizeof(msg), info.from_hsi ? "M(HSI)->48M\r\n" : "M->48M\r\n");

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
