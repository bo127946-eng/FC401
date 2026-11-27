/*
 * Project: FPV Flight Controller Firmware (ST Bare-Metal / C++17)
 * Version: 4.52-F401-CDC
 * Target: STM32F401CCU6 Black Pill (84 MHz SYSCLK, 48 MHz USB OTG FS, FPU, DMA1/DMA2)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace Reg {
  struct GPIO_TypeDef {
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

  struct RCC_TypeDef {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t RESERVED2[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED3[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
  };

  struct FLASH_TypeDef {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
  };

  struct TIM_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
  };

  struct USART_TypeDef {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
  };

  struct SPI_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
  };

  struct I2C_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
  };

  struct ADC_TypeDef {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
  };

  struct ADC_Common_TypeDef {
    volatile uint32_t CSR;
    volatile uint32_t CCR;
    volatile uint32_t CDR;
  };

  struct DMA_Stream_TypeDef {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
  };

  struct DMA_TypeDef {
    volatile uint32_t LISR;
    volatile uint32_t HISR;
    volatile uint32_t LIFCR;
    volatile uint32_t HIFCR;
    DMA_Stream_TypeDef S[8];
  };

  struct OTG_FS_INEP_TypeDef {
    volatile uint32_t DIEPCTL;
    uint32_t RESERVED0;
    volatile uint32_t DIEPINT;
    uint32_t RESERVED1;
    volatile uint32_t DIEPTSIZ;
    volatile uint32_t DIEPDMA;
    volatile uint32_t DTXFSTS;
    uint32_t RESERVED2;
  };

  struct OTG_FS_OUTEP_TypeDef {
    volatile uint32_t DOEPCTL;
    uint32_t RESERVED0;
    volatile uint32_t DOEPINT;
    uint32_t RESERVED1;
    volatile uint32_t DOEPTSIZ;
    volatile uint32_t DOEPDMA;
    uint32_t RESERVED2[2];
  };

  struct OTG_FS_TypeDef {
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
    uint32_t RESERVED0[2];
    volatile uint32_t GCCFG;
    volatile uint32_t CID;
    uint32_t RESERVED1[48];
    volatile uint32_t HPTXFSIZ;
    volatile uint32_t DIEPTXF[3];
    uint32_t RESERVED2[188];
    volatile uint32_t DCFG;
    volatile uint32_t DCTL;
    volatile uint32_t DSTS;
    uint32_t RESERVED3;
    volatile uint32_t DIEPMSK;
    volatile uint32_t DOEPMSK;
    volatile uint32_t DAINT;
    volatile uint32_t DAINTMSK;
    uint32_t RESERVED4[2];
    volatile uint32_t DVBUSDIS;
    volatile uint32_t DVBUSPULSE;
    uint32_t RESERVED5;
    volatile uint32_t DIEPEMPMSK;
    uint32_t RESERVED6[50];
    OTG_FS_INEP_TypeDef INEP[4];
    uint32_t RESERVED7[160];
    OTG_FS_OUTEP_TypeDef OUTEP[4];
    uint32_t RESERVED8[144];
    volatile uint32_t PCGCCTL;
  };

  struct STK_TypeDef {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
  };

  struct SCB_TypeDef {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR[3];
    volatile uint32_t SHCSR;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR;
    volatile uint32_t DFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
    volatile uint32_t RESERVED[18];
    volatile uint32_t CPACR;
  };

  constexpr uintptr_t PERIPH_BASE     = 0x40000000UL;
  constexpr uintptr_t APB1PERIPH_BASE = PERIPH_BASE;
  constexpr uintptr_t APB2PERIPH_BASE = PERIPH_BASE + 0x00010000UL;
  constexpr uintptr_t AHB1PERIPH_BASE = PERIPH_BASE + 0x00020000UL;
  constexpr uintptr_t OTG_FS_BASE     = 0x50000000UL;

  inline TIM_TypeDef*        const TIM2        = reinterpret_cast<TIM_TypeDef*>(APB1PERIPH_BASE + 0x0000UL);
  inline SPI_TypeDef*        const SPI2        = reinterpret_cast<SPI_TypeDef*>(APB1PERIPH_BASE + 0x3800UL);
  inline I2C_TypeDef*        const I2C1        = reinterpret_cast<I2C_TypeDef*>(APB1PERIPH_BASE + 0x5400UL);

  inline USART_TypeDef*      const USART1      = reinterpret_cast<USART_TypeDef*>(APB2PERIPH_BASE + 0x1000UL);
  inline USART_TypeDef*      const USART6      = reinterpret_cast<USART_TypeDef*>(APB2PERIPH_BASE + 0x1400UL);
  inline ADC_TypeDef*        const ADC1        = reinterpret_cast<ADC_TypeDef*>(APB2PERIPH_BASE + 0x2000UL);
  inline ADC_Common_TypeDef* const ADC1_COMMON = reinterpret_cast<ADC_Common_TypeDef*>(APB2PERIPH_BASE + 0x2300UL);

  inline GPIO_TypeDef*       const GPIOA       = reinterpret_cast<GPIO_TypeDef*>(AHB1PERIPH_BASE + 0x0000UL);
  inline GPIO_TypeDef*       const GPIOB       = reinterpret_cast<GPIO_TypeDef*>(AHB1PERIPH_BASE + 0x0400UL);
  inline GPIO_TypeDef*       const GPIOC       = reinterpret_cast<GPIO_TypeDef*>(AHB1PERIPH_BASE + 0x0800UL);
  inline RCC_TypeDef*        const RCC         = reinterpret_cast<RCC_TypeDef*>(AHB1PERIPH_BASE + 0x3800UL);
  inline FLASH_TypeDef*      const FLASH       = reinterpret_cast<FLASH_TypeDef*>(AHB1PERIPH_BASE + 0x3C00UL);
  inline DMA_TypeDef*        const DMA1        = reinterpret_cast<DMA_TypeDef*>(AHB1PERIPH_BASE + 0x6000UL);
  inline DMA_TypeDef*        const DMA2        = reinterpret_cast<DMA_TypeDef*>(AHB1PERIPH_BASE + 0x6400UL);

  inline OTG_FS_TypeDef*     const OTG_FS      = reinterpret_cast<OTG_FS_TypeDef*>(OTG_FS_BASE);

  inline volatile uint32_t* otgFifoPtr(uint8_t ep_num) {
    return reinterpret_cast<volatile uint32_t*>(OTG_FS_BASE + 0x1000UL + ((uint32_t)ep_num * 0x1000UL));
  }

  inline STK_TypeDef*        const STK         = reinterpret_cast<STK_TypeDef*>(0xE000E010UL);
  inline SCB_TypeDef*        const SCB         = reinterpret_cast<SCB_TypeDef*>(0xE000ED00UL);
}

static volatile uint32_t g_ms_ticks = 0;
constexpr uint32_t SYSCLK_HZ = 84000000UL;
constexpr uint32_t APB1_HZ   = 42000000UL;
constexpr uint32_t APB2_HZ   = 84000000UL;

constexpr uint32_t FLASH_CFG_ADDR   = 0x0803C000UL;
constexpr uint32_t FLASH_CFG_SECTOR = 5U;

extern "C" void SystemInit(void) {
  Reg::SCB->CPACR |= (0xFUL << 20);
  __asm volatile("dsb");
  __asm volatile("isb");

  Reg::RCC->CR |= (1U << 16);
  while (!(Reg::RCC->CR & (1U << 17))) {}

  Reg::FLASH->ACR = (1U << 8) | (1U << 9) | (1U << 10) | 2U;

  Reg::RCC->PLLCFGR = (25U << 0) | (336U << 6) | (1U << 16) | (1U << 22) | (7U << 24);
  Reg::RCC->CR |= (1U << 24);
  while (!(Reg::RCC->CR & (1U << 25))) {}

  Reg::RCC->CFGR &= ~((0xFU << 4) | (0x7U << 10) | (0x7U << 13));
  Reg::RCC->CFGR |= (0x4U << 10);

  Reg::RCC->CFGR &= ~0x3U;
  Reg::RCC->CFGR |= 0x2U;
  while ((Reg::RCC->CFGR & 0xCU) != 0x8U) {}

  Reg::STK->LOAD = (SYSCLK_HZ / 1000U) - 1U;
  Reg::STK->VAL  = 0U;
  Reg::STK->CTRL = 0x07U;
}

extern "C" void SysTick_Handler(void) {
  g_ms_ticks++;
}

static inline uint32_t millis() {
  return g_ms_ticks;
}

static inline uint32_t micros() {
  uint32_t ms, val;
  do {
    ms = g_ms_ticks;
    val = Reg::STK->VAL;
  } while (ms != g_ms_ticks);
  uint32_t load = Reg::STK->LOAD;
  return ms * 1000UL + ((load - val) * 1000UL) / (load + 1UL);
}

static void delay(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {}
}

static inline float constrainF(float v, float lo, float hi) {
  if (!isfinite(v)) return lo;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline long constrainL(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int constrainI(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

enum GpioMode : uint8_t {
  GPIO_INPUT  = 0x0,
  GPIO_OUTPUT = 0x1,
  GPIO_AF     = 0x2,
  GPIO_ANALOG = 0x3
};

enum GpioPull : uint8_t {
  GPIO_NO_PULL = 0x0,
  GPIO_PULL_UP = 0x1,
  GPIO_PULL_DN = 0x2
};

static void gpioConfigPin(Reg::GPIO_TypeDef* port, uint8_t pin, GpioMode mode,
                          GpioPull pull = GPIO_NO_PULL, uint8_t af = 0, bool open_drain = false) {
  uint32_t sh2 = pin * 2U;
  port->MODER &= ~(0x3U << sh2);
  port->MODER |= ((uint32_t)mode << sh2);

  if (open_drain) port->OTYPER |= (1U << pin);
  else port->OTYPER &= ~(1U << pin);

  port->OSPEEDR &= ~(0x3U << sh2);
  port->OSPEEDR |= (0x2U << sh2);

  port->PUPDR &= ~(0x3U << sh2);
  port->PUPDR |= ((uint32_t)pull << sh2);

  uint32_t afr_idx = (pin >> 3) & 0x1U;
  uint32_t afr_sh  = (pin & 0x7U) * 4U;
  port->AFR[afr_idx] &= ~(0xFU << afr_sh);
  port->AFR[afr_idx] |= ((uint32_t)(af & 0xFU) << afr_sh);
}

static inline void gpioWrite(Reg::GPIO_TypeDef* port, uint8_t pin, bool high) {
  if (high) port->BSRR = (1U << pin);
  else port->BSRR = (1U << (pin + 16U));
}

static inline bool gpioRead(Reg::GPIO_TypeDef* port, uint8_t pin) {
  return (port->IDR & (1U << pin)) != 0;
}

/* ============================================================================
 * Bare-Metal USB OTG FS CDC ACM Driver (EP0 Ctrl, EP1 OUT RX, EP1 IN TX, EP2 IN INT)
 * ============================================================================ */
namespace UsbCdcDriver {
  constexpr uint8_t EP0_MPS     = 64;
  constexpr uint8_t CDC_DATA_MPS  = 64;
  constexpr uint8_t CDC_INT_MPS   = 8;
  constexpr uint8_t EP_CDC_OUT    = 1;
  constexpr uint8_t EP_CDC_IN     = 1;
  constexpr uint8_t EP_CDC_INT    = 2;

  constexpr size_t RX_RING_SIZE   = 512;
  constexpr size_t TX_RING_SIZE   = 1024;

  static volatile uint8_t rx_ring[RX_RING_SIZE];
  static volatile size_t  rx_head = 0, rx_tail = 0;

  static volatile uint8_t tx_ring[TX_RING_SIZE];
  static volatile size_t  tx_head = 0, tx_tail = 0;

  static volatile bool usb_configured = false;
  static volatile bool dtr_active     = false;
  static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

  static uint8_t ep0_ctrl_buf[64];
  static const uint8_t* ep0_tx_ptr = nullptr;
  static uint16_t ep0_tx_rem = 0;
  static uint8_t ep0_rx_wreq = 0;
  static uint16_t ep0_rx_wlen = 0;
  static bool ep0_rx_pending = false;
  static uint8_t ep1_tx_chunk[CDC_DATA_MPS];
  static volatile bool ep1_tx_busy = false;

  static const uint8_t kDeviceDesc[18] = {
    18, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, EP0_MPS,
    0x83, 0x04, 0x40, 0x57, 0x00, 0x02,
    1, 2, 3, 1
  };

  static const uint8_t kConfigDesc[67] = {
    9, 0x02, 67, 0x00, 2, 1, 0, 0x80, 50,
    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,
    5, 0x24, 0x01, 0x00, 1,
    4, 0x24, 0x02, 0x02,
    5, 0x24, 0x06, 0x00, 1,
    7, 0x05, 0x80 | EP_CDC_INT, 0x03, CDC_INT_MPS, 0x00, 16,
    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 0x05, EP_CDC_OUT, 0x02, CDC_DATA_MPS, 0x00, 0,
    7, 0x05, 0x80 | EP_CDC_IN, 0x02, CDC_DATA_MPS, 0x00, 0
  };

  static const uint8_t kLangDesc[4]   = {4, 0x03, 0x09, 0x04};
  static const uint8_t kMfgDesc[16]   = {16, 0x03, 'S',0, 'T',0, 'M',0, '3',0, '2',0, 'F',0, '4',0};
  static const uint8_t kProdDesc[16]  = {16, 0x03, 'F',0, 'C',0, '_',0, 'C',0, 'D',0, 'C',0, ' ',0};
  static const uint8_t kSerDesc[10]   = {10, 0x03, '0',0, '0',0, '0',0, '1',0};

  static inline size_t rxCount() {
    size_t h = rx_head, t = rx_tail;
    return (h >= t) ? (h - t) : (RX_RING_SIZE - t + h);
  }

  static inline void rxPush(uint8_t b) {
    size_t nxt = (rx_head + 1U) % RX_RING_SIZE;
    if (nxt != rx_tail) {
      rx_ring[rx_head] = b;
      rx_head = nxt;
    }
  }

  static inline bool rxPop(uint8_t &out) {
    if (rx_tail == rx_head) return false;
    out = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_RING_SIZE;
    return true;
  }

  static inline size_t txCount() {
    size_t h = tx_head, t = tx_tail;
    return (h >= t) ? (h - t) : (TX_RING_SIZE - t + h);
  }

  static inline bool txPush(uint8_t b) {
    size_t nxt = (tx_head + 1U) % TX_RING_SIZE;
    if (nxt == tx_tail) return false;
    tx_ring[tx_head] = b;
    tx_head = nxt;
    return true;
  }

  static inline bool txPop(uint8_t &out) {
    if (tx_tail == tx_head) return false;
    out = tx_ring[tx_tail];
    tx_tail = (tx_tail + 1U) % TX_RING_SIZE;
    return true;
  }

  static void writePacketToFifo(uint8_t ep_num, const uint8_t* src, uint16_t len) {
    volatile uint32_t* fifo = Reg::otgFifoPtr(ep_num);
    uint16_t words = (len + 3U) / 4U;
    for (uint16_t i = 0; i < words; i++) {
      uint32_t w = 0;
      for (uint8_t b = 0; b < 4; b++) {
        uint16_t idx = (uint16_t)(i * 4U + b);
        if (idx < len) w |= ((uint32_t)src[idx]) << (b * 8U);
      }
      *fifo = w;
    }
  }

  static void readPacketFromFifo(uint8_t* dst, uint16_t len) {
    volatile uint32_t* fifo = Reg::otgFifoPtr(0);
    uint16_t words = (len + 3U) / 4U;
    uint16_t pos = 0;
    for (uint16_t i = 0; i < words; i++) {
      uint32_t w = *fifo;
      for (uint8_t b = 0; b < 4 && pos < len; b++) {
        dst[pos++] = (uint8_t)((w >> (b * 8U)) & 0xFFU);
      }
    }
  }

  static void flushTxFifo(uint8_t fnum) {
    Reg::OTG_FS->GRSTCTL = ((uint32_t)fnum << 6) | (1U << 5);
    uint32_t s = micros();
    while ((Reg::OTG_FS->GRSTCTL & (1U << 5)) && ((micros() - s) < 2000U)) {}
  }

  static void flushRxFifo() {
    Reg::OTG_FS->GRSTCTL = (1U << 4);
    uint32_t s = micros();
    while ((Reg::OTG_FS->GRSTCTL & (1U << 4)) && ((micros() - s) < 2000U)) {}
  }

  static void ep0ArmOutSetup(uint16_t cnt) {
    Reg::OTG_FS->OUTEP[0].DOEPTSIZ = (1U << 29) | (1U << 19) | cnt;
    Reg::OTG_FS->OUTEP[0].DOEPCTL  = (1U << 31) | (1U << 26);
  }

  static void ep0StartIn(const uint8_t* data, uint16_t len, uint16_t wlen) {
    uint16_t send_len = (len < wlen) ? len : wlen;
    ep0_tx_ptr = data;
    ep0_tx_rem = send_len;
    uint16_t pkt = (send_len > EP0_MPS) ? EP0_MPS : send_len;
    Reg::OTG_FS->INEP[0].DIEPTSIZ = (1U << 19) | pkt;
    Reg::OTG_FS->INEP[0].DIEPCTL  = (1U << 31) | (1U << 26);
    if (pkt > 0) writePacketToFifo(0, ep0_tx_ptr, pkt);
  }

  static void ep0SendZeroStatus() {
    ep0_tx_ptr = nullptr;
    ep0_tx_rem = 0;
    Reg::OTG_FS->INEP[0].DIEPTSIZ = (1U << 19) | 0U;
    Reg::OTG_FS->INEP[0].DIEPCTL  = (1U << 31) | (1U << 26);
  }

  static void ep0Stall() {
    Reg::OTG_FS->INEP[0].DIEPCTL  |= (1U << 21);
    Reg::OTG_FS->OUTEP[0].DOEPCTL |= (1U << 21);
    ep0ArmOutSetup(EP0_MPS);
  }

  static void ep1ArmOut() {
    Reg::OTG_FS->OUTEP[EP_CDC_OUT].DOEPTSIZ = (1U << 19) | CDC_DATA_MPS;
    Reg::OTG_FS->OUTEP[EP_CDC_OUT].DOEPCTL  = (1U << 31) | (1U << 26) | (1U << 28);
  }

  static void kickTx() {
    if (!usb_configured || ep1_tx_busy) return;
    size_t n = txCount();
    if (n == 0) return;
    uint16_t chunk = (n > CDC_DATA_MPS) ? CDC_DATA_MPS : (uint16_t)n;
    for (uint16_t i = 0; i < chunk; i++) {
      uint8_t b = 0;
      if (!txPop(b)) { chunk = i; break; }
      ep1_tx_chunk[i] = b;
    }
    if (chunk == 0) return;
    ep1_tx_busy = true;
    Reg::OTG_FS->INEP[EP_CDC_IN].DIEPTSIZ = (1U << 19) | chunk;
    Reg::OTG_FS->INEP[EP_CDC_IN].DIEPCTL  = (1U << 31) | (1U << 26) | (1U << 28);
    writePacketToFifo(EP_CDC_IN, ep1_tx_chunk, chunk);
  }

  static void handleSetupPacket(const uint8_t* s) {
    uint8_t  bmReq = s[0];
    uint8_t  bReq  = s[1];
    uint16_t wVal  = (uint16_t)s[2] | ((uint16_t)s[3] << 8);
    uint16_t wLen  = (uint16_t)s[6] | ((uint16_t)s[7] << 8);

    if ((bmReq & 0x60U) == 0x00U) {
      if (bReq == 0x06U) {
        uint8_t dtype = (uint8_t)(wVal >> 8);
        uint8_t didx  = (uint8_t)(wVal & 0xFF);
        if (dtype == 0x01U) { ep0StartIn(kDeviceDesc, sizeof(kDeviceDesc), wLen); return; }
        if (dtype == 0x02U) { ep0StartIn(kConfigDesc, sizeof(kConfigDesc), wLen); return; }
        if (dtype == 0x03U) {
          if (didx == 0) { ep0StartIn(kLangDesc, sizeof(kLangDesc), wLen); return; }
          if (didx == 1) { ep0StartIn(kMfgDesc, sizeof(kMfgDesc), wLen); return; }
          if (didx == 2) { ep0StartIn(kProdDesc, sizeof(kProdDesc), wLen); return; }
          if (didx == 3) { ep0StartIn(kSerDesc, sizeof(kSerDesc), wLen); return; }
        }
        ep0Stall();
        return;
      }
      if (bReq == 0x05U) {
        Reg::OTG_FS->DCFG = (Reg::OTG_FS->DCFG & ~(0x7FU << 4)) | (((uint32_t)wVal & 0x7FU) << 4);
        ep0SendZeroStatus();
        return;
      }
      if (bReq == 0x09U) {
        usb_configured = (wVal != 0);
        ep1_tx_busy = false;
        Reg::OTG_FS->INEP[EP_CDC_IN].DIEPCTL  = (1U << 28) | (2U << 18) | (1U << 15) | CDC_DATA_MPS;
        Reg::OTG_FS->OUTEP[EP_CDC_OUT].DOEPCTL = (1U << 28) | (2U << 18) | (1U << 15) | CDC_DATA_MPS;
        Reg::OTG_FS->INEP[EP_CDC_INT].DIEPCTL = (1U << 28) | (3U << 18) | (1U << 15) | CDC_INT_MPS;
        ep1ArmOut();
        ep0SendZeroStatus();
        kickTx();
        return;
      }
      if (bReq == 0x00U) {
        static const uint8_t st0[2] = {0, 0};
        ep0StartIn(st0, 2, wLen);
        return;
      }
    } else if ((bmReq & 0x60U) == 0x20U) {
      if (bReq == 0x20U) {
        ep0_rx_wreq = bReq;
        ep0_rx_wlen = (wLen > sizeof(ep0_ctrl_buf)) ? sizeof(ep0_ctrl_buf) : wLen;
        ep0_rx_pending = true;
        ep0ArmOutSetup(ep0_rx_wlen);
        return;
      }
      if (bReq == 0x21U) {
        ep0StartIn(line_coding, sizeof(line_coding), wLen);
        return;
      }
      if (bReq == 0x22U) {
        dtr_active = (wVal & 0x01U) != 0;
        ep0SendZeroStatus();
        return;
      }
    }
    ep0SendZeroStatus();
  }

  static void onUsbReset() {
    usb_configured = false;
    ep1_tx_busy = false;
    ep0_tx_ptr = nullptr;
    ep0_tx_rem = 0;
    ep0_rx_pending = false;

    Reg::OTG_FS->DCFG &= ~(0x7FU << 4);
    flushRxFifo();
    flushTxFifo(0);
    flushTxFifo(1);
    flushTxFifo(2);

    Reg::OTG_FS->GRXFSIZ    = 128U;
    Reg::OTG_FS->DIEPTXF0   = (32U << 16) | 128U;
    Reg::OTG_FS->DIEPTXF[0] = (128U << 16) | 160U;
    Reg::OTG_FS->DIEPTXF[1] = (32U << 16) | 288U;

    Reg::OTG_FS->INEP[0].DIEPCTL  = 0x00U;
    Reg::OTG_FS->OUTEP[0].DOEPCTL = 0x00U;
    ep0ArmOutSetup(EP0_MPS);
  }

  void init() {
    Reg::RCC->AHB2ENR |= (1U << 7);
    gpioConfigPin(Reg::GPIOA, 11, GPIO_AF, GPIO_NO_PULL, 10, false);
    gpioConfigPin(Reg::GPIOA, 12, GPIO_AF, GPIO_NO_PULL, 10, false);

    Reg::OTG_FS->GAHBCFG &= ~1U;
    Reg::OTG_FS->GUSBCFG |= (1U << 6);
    delay(2);
    Reg::OTG_FS->GRSTCTL |= (1U << 0);
    while (Reg::OTG_FS->GRSTCTL & (1U << 0)) {}
    delay(2);

    Reg::OTG_FS->GCCFG = (1U << 21) | (1U << 16);
    Reg::OTG_FS->DCFG  = 0x03U;
    Reg::OTG_FS->GINTMSK = (1U << 12) | (1U << 4) | (1U << 18) | (1U << 19);
    Reg::OTG_FS->DAINTMSK = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 16) | (1U << 17);
    Reg::OTG_FS->DIEPMSK  = (1U << 0);
    Reg::OTG_FS->DOEPMSK  = (1U << 0) | (1U << 3);

    Reg::OTG_FS->DCTL &= ~(1U << 1);
    onUsbReset();
  }

  void poll() {
    uint32_t gint = Reg::OTG_FS->GINTSTS;

    if (gint & (1U << 12)) {
      Reg::OTG_FS->GINTSTS = (1U << 12);
      onUsbReset();
    }

    while (Reg::OTG_FS->GINTSTS & (1U << 4)) {
      uint32_t rxst = Reg::OTG_FS->GRXSTSP;
      uint8_t ep_num = (uint8_t)(rxst & 0x0FU);
      uint16_t bcnt  = (uint16_t)((rxst >> 4) & 0x7FFU);
      uint8_t pktsts = (uint8_t)((rxst >> 17) & 0x0FU);

      if (pktsts == 6U) {
        uint8_t sb[8] = {0};
        if (bcnt > 0) readPacketFromFifo(sb, (bcnt > 8) ? 8 : bcnt);
        if (ep_num == 0) handleSetupPacket(sb);
      } else if (pktsts == 2U) {
        uint8_t tmp[CDC_DATA_MPS];
        uint16_t rlen = (bcnt > sizeof(tmp)) ? (uint16_t)sizeof(tmp) : bcnt;
        if (rlen > 0) readPacketFromFifo(tmp, rlen);
        if (ep_num == 0 && ep0_rx_pending) {
          uint16_t copy_n = (rlen < ep0_rx_wlen) ? rlen : ep0_rx_wlen;
          if (copy_n > 0) memcpy(ep0_ctrl_buf, tmp, copy_n);
          if (ep0_rx_wreq == 0x20U && copy_n >= 7) {
            memcpy(line_coding, ep0_ctrl_buf, 7);
          }
          ep0_rx_pending = false;
        } else if (ep_num == EP_CDC_OUT) {
          for (uint16_t i = 0; i < rlen; i++) rxPush(tmp[i]);
        }
      }
    }

    if (Reg::OTG_FS->GINTSTS & (1U << 19)) {
      if (Reg::OTG_FS->OUTEP[0].DOEPINT & (1U << 3)) {
        Reg::OTG_FS->OUTEP[0].DOEPINT = (1U << 3);
      }
      if (Reg::OTG_FS->OUTEP[0].DOEPINT & (1U << 0)) {
        Reg::OTG_FS->OUTEP[0].DOEPINT = (1U << 0);
        if (!ep0_rx_pending && ep0_rx_wreq == 0x20U) {
          ep0_rx_wreq = 0;
          ep0SendZeroStatus();
        } else {
          ep0ArmOutSetup(EP0_MPS);
        }
      }
      if (Reg::OTG_FS->OUTEP[EP_CDC_OUT].DOEPINT & (1U << 0)) {
        Reg::OTG_FS->OUTEP[EP_CDC_OUT].DOEPINT = (1U << 0);
        ep1ArmOut();
      }
    }

    if (Reg::OTG_FS->GINTSTS & (1U << 18)) {
      if (Reg::OTG_FS->INEP[0].DIEPINT & (1U << 0)) {
        Reg::OTG_FS->INEP[0].DIEPINT = (1U << 0);
        if (ep0_tx_ptr && ep0_tx_rem > 0) {
          uint16_t adv = (ep0_tx_rem > EP0_MPS) ? EP0_MPS : ep0_tx_rem;
          ep0_tx_ptr += adv;
          ep0_tx_rem = (uint16_t)(ep0_tx_rem - adv);
          uint16_t nxt = (ep0_tx_rem > EP0_MPS) ? EP0_MPS : ep0_tx_rem;
          Reg::OTG_FS->INEP[0].DIEPTSIZ = (1U << 19) | nxt;
          Reg::OTG_FS->INEP[0].DIEPCTL  = (1U << 31) | (1U << 26);
          if (nxt > 0) writePacketToFifo(0, ep0_tx_ptr, nxt);
        } else {
          ep0_tx_ptr = nullptr;
          ep0ArmOutSetup(EP0_MPS);
        }
      }
      if (Reg::OTG_FS->INEP[EP_CDC_IN].DIEPINT & (1U << 0)) {
        Reg::OTG_FS->INEP[EP_CDC_IN].DIEPINT = (1U << 0);
        ep1_tx_busy = false;
        kickTx();
      }
      if (Reg::OTG_FS->INEP[EP_CDC_INT].DIEPINT & (1U << 0)) {
        Reg::OTG_FS->INEP[EP_CDC_INT].DIEPINT = (1U << 0);
      }
    }

    kickTx();
  }
}

class UsbCdcPort {
public:
  void begin(uint32_t baud = 115200) {
    (void)baud;
    UsbCdcDriver::init();
  }

  void flush() {
    uint32_t s = millis();
    while (UsbCdcDriver::txCount() > 0 || UsbCdcDriver::ep1_tx_busy) {
      UsbCdcDriver::poll();
      if ((millis() - s) > 25U) break;
    }
  }

  int available() {
    UsbCdcDriver::poll();
    return (int)UsbCdcDriver::rxCount();
  }

  int read() {
    UsbCdcDriver::poll();
    uint8_t b = 0;
    if (!UsbCdcDriver::rxPop(b)) return -1;
    return (int)b;
  }

  size_t write(uint8_t b) {
    UsbCdcDriver::poll();
    if (!UsbCdcDriver::usb_configured) return 0;
    if (!UsbCdcDriver::txPush(b)) {
      UsbCdcDriver::poll();
      if (!UsbCdcDriver::txPush(b)) return 0;
    }
    UsbCdcDriver::poll();
    return 1;
  }

  size_t write(const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return 0;
    UsbCdcDriver::poll();
    if (!UsbCdcDriver::usb_configured) return 0;
    size_t n = 0;
    for (size_t i = 0; i < len; i++) {
      if (!UsbCdcDriver::txPush(buf[i])) {
        UsbCdcDriver::poll();
        if (!UsbCdcDriver::txPush(buf[i])) break;
      }
      n++;
    }
    UsbCdcDriver::poll();
    return n;
  }

  void print(const char* s) {
    if (!s) return;
    write(reinterpret_cast<const uint8_t*>(s), strlen(s));
  }

  void print(int v) {
    char b[16];
    int idx = 0;
    unsigned int uv;
    if (v < 0) {
      b[idx++] = '-';
      uv = (unsigned int)(-(long)v);
    } else {
      uv = (unsigned int)v;
    }
    char tmp[12];
    int t = 0;
    do {
      tmp[t++] = (char)('0' + (uv % 10U));
      uv /= 10U;
    } while (uv > 0U && t < 11);
    while (t > 0) b[idx++] = tmp[--t];
    b[idx] = '\0';
    print(b);
  }

  void print(unsigned int v) {
    char b[16];
    char tmp[12];
    int t = 0, idx = 0;
    do {
      tmp[t++] = (char)('0' + (v % 10U));
      v /= 10U;
    } while (v > 0U && t < 11);
    while (t > 0) b[idx++] = tmp[--t];
    b[idx] = '\0';
    print(b);
  }

  void print(float v, int dec = 2) {
    if (!isfinite(v)) { print("nan"); return; }
    if (dec < 0) dec = 0;
    if (dec > 6) dec = 6;
    if (v < 0.0f) { print("-"); v = -v; }
    uint32_t ip = (uint32_t)v;
    print((unsigned int)ip);
    if (dec > 0) {
      print(".");
      float rem = v - (float)ip;
      uint32_t scale = 1U;
      for (int i = 0; i < dec; i++) scale *= 10U;
      uint32_t fp = (uint32_t)lroundf(rem * (float)scale);
      if (fp >= scale) fp = scale - 1U;
      for (int i = dec - 1; i >= 0; i--) {
        uint32_t p10 = 1U;
        for (int k = 0; k < i; k++) p10 *= 10U;
        char c = (char)('0' + ((fp / p10) % 10U));
        write((const uint8_t*)&c, 1);
      }
    }
  }

  void println(const char* s = "") { print(s); print("\r\n"); }
  void println(int v) { print(v); print("\r\n"); }
  void println(unsigned int v) { print(v); print("\r\n"); }
  void println(float v, int dec = 2) { print(v, dec); print("\r\n"); }
};

static UsbCdcPort usb_cdc;

constexpr size_t UART1_DMA_RX_SIZE = 128;
static uint8_t uart1_dma_rx_buf[UART1_DMA_RX_SIZE];
static size_t  uart1_rx_read_idx = 0;

class UartDmaPort {
public:
  UartDmaPort(Reg::USART_TypeDef* dev, uint32_t pclk_hz, bool use_dma12)
      : dev_(dev), pclk_(pclk_hz), use_dma12_(use_dma12) {}

  void begin(uint32_t baud) {
    if (baud == 0) baud = 115200;
    dev_->CR1 &= ~(1U << 13);
    dev_->BRR = (pclk_ + (baud / 2U)) / baud;
    dev_->CR1 = (1U << 2) | (1U << 3);

    if (use_dma12_) {
      Reg::DMA2->S[2].CR &= ~1U;
      while (Reg::DMA2->S[2].CR & 1U) {}
      Reg::DMA2->LIFCR = (0x3DU << 16);
      Reg::DMA2->S[2].PAR  = reinterpret_cast<uint32_t>(&dev_->DR);
      Reg::DMA2->S[2].M0AR = reinterpret_cast<uint32_t>(uart1_dma_rx_buf);
      Reg::DMA2->S[2].NDTR = UART1_DMA_RX_SIZE;
      Reg::DMA2->S[2].CR   = (4U << 25) | (1U << 10) | (1U << 8);
      Reg::DMA2->S[2].CR  |= 1U;
      dev_->CR3 |= (1U << 6);
      uart1_rx_read_idx = 0;
    }

    dev_->CR1 |= (1U << 13);
  }

  void end() {
    if (use_dma12_) {
      Reg::DMA2->S[2].CR &= ~1U;
      Reg::DMA2->S[7].CR &= ~1U;
      dev_->CR3 &= ~((1U << 6) | (1U << 7));
    }
    dev_->CR1 &= ~(1U << 13);
  }

  void flush() {
    uint32_t s = millis();
    while (!(dev_->SR & (1U << 6))) {
      if ((millis() - s) > 25) break;
    }
  }

  int available() {
    if (use_dma12_) {
      size_t write_idx = UART1_DMA_RX_SIZE - (size_t)Reg::DMA2->S[2].NDTR;
      write_idx %= UART1_DMA_RX_SIZE;
      return (int)((write_idx + UART1_DMA_RX_SIZE - uart1_rx_read_idx) % UART1_DMA_RX_SIZE);
    }
    return (dev_->SR & (1U << 5)) ? 1 : 0;
  }

  int read() {
    if (use_dma12_) {
      if (available() <= 0) return -1;
      uint8_t b = uart1_dma_rx_buf[uart1_rx_read_idx];
      uart1_rx_read_idx = (uart1_rx_read_idx + 1U) % UART1_DMA_RX_SIZE;
      return (int)b;
    }
    if (!(dev_->SR & (1U << 5))) return -1;
    return (int)(dev_->DR & 0xFFU);
  }

  size_t write(uint8_t b) {
    uint32_t s = millis();
    while (!(dev_->SR & (1U << 7))) {
      if ((millis() - s) > 20) return 0;
    }
    dev_->DR = b;
    return 1;
  }

  size_t write(const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return 0;
    if (use_dma12_ && len >= 4) {
      uint32_t s = millis();
      while (Reg::DMA2->S[7].CR & 1U) {
        if ((millis() - s) > 25) break;
      }
      if (!(Reg::DMA2->S[7].CR & 1U)) {
        Reg::DMA2->HIFCR = (0x3DU << 22);
        Reg::DMA2->S[7].PAR  = reinterpret_cast<uint32_t>(&dev_->DR);
        Reg::DMA2->S[7].M0AR = reinterpret_cast<uint32_t>(buf);
        Reg::DMA2->S[7].NDTR = (uint16_t)len;
        Reg::DMA2->S[7].CR   = (4U << 25) | (1U << 10) | (1U << 6);
        dev_->CR3 |= (1U << 7);
        Reg::DMA2->S[7].CR  |= 1U;
        uint32_t w = millis();
        while (!(Reg::DMA2->HISR & (1U << 27))) {
          if ((millis() - w) > 30) break;
        }
        Reg::DMA2->S[7].CR &= ~1U;
        Reg::DMA2->HIFCR = (0x3DU << 22);
        return len;
      }
    }
    size_t n = 0;
    for (size_t i = 0; i < len; i++) {
      if (write(buf[i]) == 1) n++;
      else break;
    }
    return n;
  }

  void print(const char* s) {
    if (!s) return;
    write(reinterpret_cast<const uint8_t*>(s), strlen(s));
  }

  void println(const char* s = "") { print(s); print("\r\n"); }

private:
  Reg::USART_TypeDef* dev_;
  uint32_t pclk_;
  bool use_dma12_;
};

static UartDmaPort uart6(Reg::USART6, APB2_HZ, false);

static void initSpi2() {
  gpioConfigPin(Reg::GPIOB, 13, GPIO_AF, GPIO_NO_PULL, 5, false);
  gpioConfigPin(Reg::GPIOB, 14, GPIO_AF, GPIO_NO_PULL, 5, false);
  gpioConfigPin(Reg::GPIOB, 15, GPIO_AF, GPIO_NO_PULL, 5, false);
  gpioConfigPin(Reg::GPIOB, 12, GPIO_OUTPUT, GPIO_PULL_UP, 0, false);
  gpioConfigPin(Reg::GPIOB, 0,  GPIO_OUTPUT, GPIO_PULL_UP, 0, false);
  gpioConfigPin(Reg::GPIOB, 1,  GPIO_INPUT, GPIO_PULL_DN, 0, false);
  gpioWrite(Reg::GPIOB, 12, true);
  gpioWrite(Reg::GPIOB, 0, true);

  Reg::SPI2->CR1 = (1U << 2) | (2U << 3) | (1U << 8) | (1U << 9) | (1U << 6);
  Reg::SPI2->CR2 = (1U << 1);
}

static inline void spi2BeginTx() {
  gpioWrite(Reg::GPIOB, 12, false);
}

static inline void spi2EndTx() {
  uint32_t s = micros();
  while (Reg::SPI2->SR & (1U << 7)) {
    if ((micros() - s) > 200) break;
  }
  gpioWrite(Reg::GPIOB, 12, true);
}

static inline uint8_t spi2Transfer(uint8_t b) {
  uint32_t s = micros();
  while (!(Reg::SPI2->SR & (1U << 1))) { if ((micros() - s) > 100) return 0; }
  Reg::SPI2->DR = b;
  s = micros();
  while (!(Reg::SPI2->SR & (1U << 0))) { if ((micros() - s) > 100) return 0; }
  return (uint8_t)(Reg::SPI2->DR & 0xFFU);
}

static void spi2WriteDma(const uint8_t* data, uint16_t len) {
  if (!data || len == 0) return;
  Reg::DMA1->S[4].CR &= ~1U;
  while (Reg::DMA1->S[4].CR & 1U) {}
  Reg::DMA1->HIFCR = (0x3DU << 0);
  Reg::DMA1->S[4].PAR  = reinterpret_cast<uint32_t>(&Reg::SPI2->DR);
  Reg::DMA1->S[4].M0AR = reinterpret_cast<uint32_t>(data);
  Reg::DMA1->S[4].NDTR = len;
  Reg::DMA1->S[4].CR   = (0U << 25) | (1U << 10) | (1U << 6);
  Reg::DMA1->S[4].CR  |= 1U;

  uint32_t s = micros();
  while (!(Reg::DMA1->HISR & (1U << 5))) {
    if ((micros() - s) > 1500) break;
  }
  Reg::DMA1->S[4].CR &= ~1U;
  Reg::DMA1->HIFCR = (0x3DU << 0);
  s = micros();
  while (Reg::SPI2->SR & (1U << 7)) {
    if ((micros() - s) > 200) break;
  }
  while (Reg::SPI2->SR & (1U << 0)) { (void)Reg::SPI2->DR; }
}

static void initI2c1() {
  gpioConfigPin(Reg::GPIOB, 6, GPIO_AF, GPIO_PULL_UP, 4, true);
  gpioConfigPin(Reg::GPIOB, 7, GPIO_AF, GPIO_PULL_UP, 4, true);

  Reg::I2C1->CR1 |= (1U << 15);
  Reg::I2C1->CR1 &= ~(1U << 15);

  Reg::I2C1->CR2   = 42U;
  Reg::I2C1->CCR   = (1U << 15) | 35U;
  Reg::I2C1->TRISE = 13U;
  Reg::I2C1->CR1  |= 0x01U;
}

static bool i2c1WaitFlag(volatile uint32_t* reg, uint32_t mask, bool set, uint32_t timeout_us = 1000) {
  uint32_t s = micros();
  while ((((*reg) & mask) != 0) != set) {
    if ((micros() - s) > timeout_us) return false;
  }
  return true;
}

static bool i2c1WriteRead(uint8_t addr7, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
  uint32_t s = micros();
  while (Reg::I2C1->SR2 & (1U << 1)) { if ((micros() - s) > 1000) return false; }

  Reg::I2C1->CR1 |= (1U << 8);
  if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 0), true)) return false;

  Reg::I2C1->DR = (uint8_t)((addr7 << 1) | 0U);
  if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 1), true)) {
    Reg::I2C1->CR1 |= (1U << 9);
    return false;
  }
  (void)Reg::I2C1->SR2;

  for (size_t i = 0; i < tx_len; i++) {
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 7), true)) return false;
    Reg::I2C1->DR = tx[i];
  }
  if (tx_len > 0 && !i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 2), true)) return false;

  if (rx_len > 0) {
    Reg::I2C1->CR1 |= (1U << 8);
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 0), true)) return false;
    Reg::I2C1->DR = (uint8_t)((addr7 << 1) | 1U);
    if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 1), true)) return false;
    (void)Reg::I2C1->SR2;

    if (rx_len == 1) {
      Reg::I2C1->CR1 &= ~(1U << 10);
      Reg::I2C1->CR1 |= (1U << 9);
      if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 6), true)) return false;
      rx[0] = (uint8_t)(Reg::I2C1->DR & 0xFFU);
    } else {
      Reg::I2C1->CR1 |= (1U << 10);
      for (size_t i = 0; i < rx_len; i++) {
        if (i == rx_len - 1) {
          Reg::I2C1->CR1 &= ~(1U << 10);
          Reg::I2C1->CR1 |= (1U << 9);
        }
        if (!i2c1WaitFlag(&Reg::I2C1->SR1, (1U << 6), true)) return false;
        rx[i] = (uint8_t)(Reg::I2C1->DR & 0xFFU);
      }
    }
  } else {
    Reg::I2C1->CR1 |= (1U << 9);
  }
  return true;
}

static volatile uint16_t adc_dma_buf[2] = {0, 0};

static void initAdc1Dma() {
  gpioConfigPin(Reg::GPIOA, 4, GPIO_ANALOG);
  gpioConfigPin(Reg::GPIOA, 5, GPIO_ANALOG);

  Reg::ADC1_COMMON->CCR = (1U << 16);
  Reg::ADC1->CR1 = (1U << 8);
  Reg::ADC1->CR2 = (1U << 1) | (1U << 8) | (1U << 9);

  Reg::ADC1->SMPR2 = (3U << (4U * 3U)) | (3U << (5U * 3U));
  Reg::ADC1->SQR1  = (1U << 20);
  Reg::ADC1->SQR3  = (4U << 0) | (5U << 5);

  Reg::DMA2->S[0].CR &= ~1U;
  while (Reg::DMA2->S[0].CR & 1U) {}
  Reg::DMA2->LIFCR = (0x3DU << 0);
  Reg::DMA2->S[0].PAR  = reinterpret_cast<uint32_t>(&Reg::ADC1->DR);
  Reg::DMA2->S[0].M0AR = reinterpret_cast<uint32_t>(adc_dma_buf);
  Reg::DMA2->S[0].NDTR = 2U;
  Reg::DMA2->S[0].CR   = (0U << 25) | (1U << 13) | (1U << 11) | (1U << 10) | (1U << 8);
  Reg::DMA2->S[0].CR  |= 1U;

  Reg::ADC1->CR2 |= (1U << 0);
  delay(1);
  Reg::ADC1->CR2 |= (1U << 30);
}

static inline uint16_t adc1ReadChannel(uint8_t ch) {
  if (ch == 4) return (uint16_t)(adc_dma_buf[0] & 0x0FFFU);
  if (ch == 5) return (uint16_t)(adc_dma_buf[1] & 0x0FFFU);
  return 0U;
}

constexpr uint32_t LOOP_INTERVAL_US = 4000;
constexpr uint32_t LOOP_OVERRUN_MARGIN_US = 200;
constexpr float DT = 0.004f;
constexpr uint32_t AUX_SENSOR_PERIOD_MS = 40;
constexpr uint32_t MS5611_CONV_DELAY_MS = 10;
constexpr uint32_t BMP180_TEMP_DELAY_MS = 5;
constexpr uint32_t BMP180_PRESS_DELAY_MS = 8;
constexpr uint32_t RTX_TX_TIMEOUT_PRI_MS = 50;
constexpr uint32_t RTX_TX_TIMEOUT_BAK_MS = 220;
constexpr uint8_t RTX_SCAN_CHANNELS_PER_TICK = 10;
constexpr uint32_t RTX_SCAN_BUSY_TIMEOUT_US = 40;
constexpr uint32_t RTX_BUSY_SHORT_TIMEOUT_US = 30;
constexpr uint8_t RTX_RESP_QUEUE_SIZE = 4;
constexpr uint32_t ERR_202_REPEAT_MS = 1000;
constexpr uint32_t VTX_CMD_INTERVAL_MS = 70;
constexpr uint32_t VTX_PROBE_INTERVAL_MS = 2000;
constexpr uint32_t VTX_PROBE_TIMEOUT_MS = 250;
constexpr uint8_t RTX_RX_MAX_PAYLOAD_BYTES = 32;
constexpr uint32_t MAG_CAL_DURATION_MS = 8000;
constexpr uint32_t MAG_CAL_SAMPLE_MS = 20;
constexpr uint32_t MON_STREAM_PERIOD_MS = 80;
constexpr uint32_t BENCH_TEST_DURATION_MS = 10000;
constexpr float BENCH_TEST_THROTTLE = 100.0f;

constexpr float SHUNT_RESISTANCE_OHM = 0.01f;
constexpr uint32_t EEPROM_MAGIC_MARKER = 0xFACE1372UL;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t SMARTAUDIO_BAUD_RATE = 4800;
constexpr uint32_t TRAMP_BAUD_RATE = 9600;

constexpr float STICK_CENTER = 500.0f;
constexpr float INITIAL_BASE_HOVER = 150.0f;
constexpr float OUTPUT_MIN = 0.0f;
constexpr float OUTPUT_MAX = 1000.0f;
constexpr float MAX_RATE_DPS_PER_UNIT = 500.0f;
constexpr float DEFAULT_FLT_AMB = 0.05f;
constexpr float DEFAULT_FLT_AMB_MIN = 0.003f;
constexpr float PID_D_ALPHA = 0.4f;
constexpr float FC_DEG_TO_RAD = 0.0174532925f;
constexpr float FC_RAD_TO_DEG = 57.29577951f;
constexpr float SIN_45 = 0.70710678f;
constexpr float COS_45 = 0.70710678f;
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 1000;
constexpr uint8_t I2C_MAX_ERRORS = 10;
constexpr uint8_t RTX_MAX_ERRORS = 10;

constexpr float CFG_ANGLE_MAX_DEG = 85.0f;
constexpr float CFG_VZ_MAX_MPS = 10.0f;
constexpr float CFG_RATE_MAX = 2.5f;
constexpr float CFG_EXPO_MAX = 1.0f;
constexpr float CFG_DEADBAND_MAX = 200.0f;
constexpr float CFG_ALPHA_MIN = 0.001f;
constexpr float CFG_ALPHA_MAX = 1.0f;
constexpr float CFG_NOTCH_FREQ_MIN = 20.0f;
constexpr float CFG_NOTCH_FREQ_MAX = 120.0f;
constexpr float CFG_NOTCH_BW_MIN = 1.0f;
constexpr float CFG_NOTCH_BW_MAX = 100.0f;
constexpr float CFG_ESC_LIMIT_MIN = 1.0f;
constexpr float CFG_ESC_LIMIT_MAX = 250.0f;
constexpr float CFG_BAT_V_MAX_MIN = 7.0f;
constexpr float CFG_BAT_V_MAX_MAX = 25.2f;
constexpr float CFG_BAT_FS_V_MIN = 5.0f;
constexpr float CFG_BAT_FS_V_MAX = 25.0f;
constexpr float CFG_BAT_SCALE_V_MIN = 0.1f;
constexpr float CFG_BAT_SCALE_V_MAX = 50.0f;
constexpr float CFG_BAT_OFFSET_A_MIN = -200.0f;
constexpr float CFG_BAT_OFFSET_A_MAX = 200.0f;
constexpr float CFG_BAT_SCALE_A_MIN = 0.05f;
constexpr float CFG_BAT_SCALE_A_MAX = 20.0f;
constexpr float CFG_LIDAR_OFFSET_MIN = -5.0f;
constexpr float CFG_LIDAR_OFFSET_MAX = 5.0f;
constexpr float CFG_MAG_BIAS_MIN = -2000.0f;
constexpr float CFG_MAG_BIAS_MAX = 2000.0f;
constexpr float CFG_RATE_KP_MAX = 10.0f;
constexpr float CFG_RATE_KI_MAX = 5.0f;
constexpr float CFG_RATE_KD_MAX = 1.0f;
constexpr float CFG_VZ_KP_MAX = 50.0f;
constexpr float CFG_VZ_KI_MAX = 20.0f;
constexpr float CFG_VZ_KD_MAX = 10.0f;

enum FcErrorCode : uint8_t {
  FC_ERR_NONE = 0, FC_ERR_LOOP_OVERRUN = 1, FC_ERR_IMU_FAIL = 2, FC_ERR_I2C_BUS = 3,
  FC_ERR_SPI_RTX = 4, FC_ERR_RC_LOSS = 5, FC_ERR_BAT_LOW = 6, FC_ERR_TILT_DISARM = 7,
  FC_ERR_RTX_TX_TIMEOUT = 8, FC_ERR_NAN_INF_MATH = 9, FC_ERR_BARO_FAIL = 10,
  FC_ERR_LIDAR_FAIL = 11, FC_ERR_MAG_FAIL = 12, FC_ERR_EEPROM_RESET = 13,
  FC_ERR_BAT_SENSOR_FAIL = 14, FC_ERR_SA_VTX_FAIL = 16, FC_ERR_CALIB_MISSING = 18
};

enum VtxProtSetting : uint8_t { VTX_PROT_AUTO = 0, VTX_PROT_TRAMP = 1, VTX_PROT_SMARTAUDIO = 2 };
enum VtxActiveProto : uint8_t { VTX_PROTO_NONE = 0, VTX_PROTO_TRAMP = 1, VTX_PROTO_SMARTAUDIO = 2 };
enum RtxChipId : uint8_t { RTX_CHIP_AUTO = 0, RTX_CHIP_SX1262 = 1, RTX_CHIP_SX1268 = 2, RTX_CHIP_LLCC68 = 3 };
enum VtxProbeStep : uint8_t { VTX_STEP_IDLE = 0, VTX_STEP_WAIT_SA, VTX_STEP_WAIT_TRAMP, VTX_STEP_WAIT_CMD_ACK };
enum DelayedRtxRespType : uint8_t {
  RTX_RESP_NONE = 0, RTX_RESP_202_ERR_EVENT = 202, RTX_RESP_230_SCAN_BEST = 230,
  RTX_RESP_240_CHAN_SW = 240, RTX_RESP_250_VTX_ACK = 250
};
enum BenchTestMode : uint8_t { BENCH_TEST_NONE = 0, BENCH_TEST_MOTORS = 1, BENCH_TEST_PID = 2 };

struct DelayedRtxRespItem { DelayedRtxRespType type = RTX_RESP_NONE; uint16_t vtx_mhz = 0; uint8_t tier = 0; uint16_t vtx_mw = 0; };
struct PIDConfig { float kp, ki, kd; };

struct __attribute__((aligned(4))) FlightConfig {
  uint32_t magic_marker;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float base_temperature;
  float temp_coeff_x, temp_coeff_y, temp_coeff_z;
  float acc_bias_x, acc_bias_y, acc_bias_z;
  float mag_bias_x, mag_bias_y, mag_bias_z;
  float lidar_offset_m, bat_scale_v, bat_offset_a, bat_scale_a, bat_v_max, flt_esc_limit;
  float fm_roll_max, fm_pitch_max, fm_vz_speed_max;
  float gyro_filter_alpha, acc_filter_alpha, filter_notch_freq, filter_notch_bw, flt_amb, flt_amb_min;
  float fs_vz_speed;
  float rc_rate, rc_super_expo, rc_stick_expo;
  float rc_yaw_rate, rc_yaw_super_expo, rc_yaw_expo;
  float rc_deadband, rc_horizon_transition, rc_filter_smoothing;
  float bat_fs_v, bat_filter_alpha, lidar_filter_alpha;
  PIDConfig roll_pid, pitch_pid, yaw_pid, vz_pid;
  uint16_t vtx_mhz, vtx_mw, rtx_addr, rtx_sync_word;
  uint8_t bat_comp_en, vtx_en, vtx_prot, esc_prot, imu_align, com_align, fc_error, fm_mode;
  uint8_t flt_amb_en, flt_lpf_en, flt_not_en, fs_bat_en, fs_rtx_en, lidar_en, baro_en, com_en;
  uint8_t rtx_en, rtx_band, rtx_hw_crc_en, rtx_addr_en, rtx_iq_inv;
  uint8_t rtx_pri_chan, rtx_pri_sf, rtx_pri_bw, rtx_pri_cr;
  int8_t rtx_pri_power;
  uint8_t rtx_bak_chan, rtx_bak_sf, rtx_bak_bw, rtx_bak_cr;
  int8_t rtx_bak_power;
  uint8_t rtx_id, imu_id, bar_id, com_id, lid_id;
  uint8_t _pad[2];
};

static_assert((sizeof(FlightConfig) % 4) == 0, "FlightConfig must be 4-byte aligned");

struct Quaternion {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  void normalize() {
    float n2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (n2 > 1e-12f && isfinite(n2)) {
      float inv = 1.0f / sqrtf(n2);
      if (isfinite(inv)) { q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv; return; }
    }
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  }
};

struct LowPassFilter {
  float alpha = 0.1f, output = 0.0f;
  void reset(float v = 0.0f) { output = isfinite(v) ? v : 0.0f; }
  float update(float in) {
    if (!isfinite(in)) return output;
    float a = constrainF(alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    output = a * in + (1.0f - a) * output;
    return isfinite(output) ? output : 0.0f;
  }
};

struct BiquadNotchFilter {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
  float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
  float last_freq = -1.0f, last_bw = -1.0f;
  void reset() { x1 = x2 = y1 = y2 = 0.0f; }
  void setParameters(float sr, float freq, float bw, bool force = false) {
    if (!force && fabsf(freq - last_freq) < 1.5f && fabsf(bw - last_bw) < 0.5f) return;
    last_freq = freq; last_bw = bw;
    if (freq <= 0.0f || bw <= 0.0f || sr <= 0.0f) { b0 = 1.0f; b1 = b2 = a1 = a2 = 0.0f; return; }
    float w = 2.0f * (float)M_PI * freq / sr;
    float sn = sinf(w), cs = cosf(w);
    float sn_s = (fabsf(sn) < 1e-6f) ? ((sn >= 0.0f) ? 1e-6f : -1e-6f) : sn;
    float al = sn * sinhf(0.34657359f * bw * w / sn_s);
    float a0 = 1.0f + al;
    b0 = 1.0f / a0; b1 = (-2.0f * cs) / a0; b2 = (1.0f - al) / a0;
    a1 = b1; a2 = (1.0f - al) / a0;
  }
  float update(float in) {
    if (!isfinite(in)) return in;
    if (b0 == 1.0f && b1 == 0.0f && a1 == 0.0f && a2 == 0.0f) return in;
    float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = in; y2 = y1; y1 = out;
    return isfinite(out) ? out : in;
  }
};

struct AdaptiveLidarFilter {
  float output = 0.0f, h[3] = {0.0f, 0.0f, 0.0f};
  bool init_ok = false;
  void reset() { output = h[0] = h[1] = h[2] = 0.0f; init_ok = false; }
  float update(float raw_v, float alpha) {
    if (!isfinite(raw_v)) return output;
    float a = constrainF(alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    if (!init_ok) { output = h[0] = h[1] = h[2] = raw_v; init_ok = true; return output; }
    h[0] = h[1]; h[1] = h[2]; h[2] = raw_v;
    float med = h[1];
    if ((h[0] >= h[1] && h[0] <= h[2]) || (h[0] <= h[1] && h[0] >= h[2])) med = h[0];
    else if ((h[2] >= h[0] && h[2] <= h[1]) || (h[2] <= h[0] && h[2] >= h[1])) med = h[2];
    output = a * med + (1.0f - a) * output;
    return isfinite(output) ? output : raw_v;
  }
};

struct PID {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f, last_meas = 0.0f, last_deriv = 0.0f, int_limit = 200.0f;
  void init(const PIDConfig &cfg, float lim) { kp = cfg.kp; ki = cfg.ki; kd = cfg.kd; int_limit = lim; reset(); }
  void reset() { integral = last_meas = last_deriv = 0.0f; }
  void updateConfig(const PIDConfig &cfg) { kp = cfg.kp; ki = cfg.ki; kd = cfg.kd; }
  float update(float err, float meas, float dt, bool sat = false) {
    if (!isfinite(err) || !isfinite(meas) || !isfinite(dt)) return 0.0f;
    if (!isfinite(integral)) integral = 0.0f;
    if (!sat && int_limit > 0.0f && dt > 0.0f) integral = constrainF(integral + err * dt, -int_limit, int_limit);
    float raw_d = (dt > 0.0f) ? -(meas - last_meas) / dt : 0.0f;
    if (!isfinite(raw_d)) raw_d = 0.0f;
    last_meas = meas;
    last_deriv = PID_D_ALPHA * raw_d + (1.0f - PID_D_ALPHA) * last_deriv;
    if (!isfinite(last_deriv)) last_deriv = 0.0f;
    float out = kp * err + ki * integral + kd * last_deriv;
    return isfinite(out) ? out : 0.0f;
  }
};

FlightConfig currentConfig;
Quaternion orientation;

uint8_t fc_error_ram = FC_ERR_NONE, fc_latched_error_ram = FC_ERR_NONE, last_sent_202_err = FC_ERR_NONE;
uint32_t last_202_send_ms = 0;
bool prev_armed_state = false;

DelayedRtxRespItem rtx_resp_queue[RTX_RESP_QUEUE_SIZE];
uint8_t rtx_resp_head = 0, rtx_resp_tail = 0, rtx_resp_count = 0;

float cpu_load_pct = 0.0f, cpu_max_load_pct = 0.0f, vibration_index = 0.0f;
bool mon_stream_active = false;
uint32_t mon_last_send_ms = 0;
BenchTestMode bench_test_mode = BENCH_TEST_NONE;
uint32_t bench_test_start_ms = 0;
float prev_m1_out = 0.0f, prev_m2_out = 0.0f, prev_m3_out = 0.0f, prev_m4_out = 0.0f;

LowPassFilter lpf_ax, lpf_ay, lpf_az, lpf_gx, lpf_gy, lpf_gz;
LowPassFilter lpf_xy_hp_x{0.15f}, lpf_xy_hp_y{0.15f};
LowPassFilter lpf_temp{0.05f}, lpf_battery{0.05f}, lpf_current{0.15f};
LowPassFilter lpf_baro_alt{0.12f}, lpf_baro_vz{0.25f};
AdaptiveLidarFilter adaptive_lidar;
BiquadNotchFilter notch_gx, notch_gy, notch_gz;

float base_hover_throttle = INITIAL_BASE_HOVER, current_vz = 0.0f;
float baro_alt_m = 0.0f, baro_ground_alt_m = 0.0f, baro_vz_mps = 0.0f;
bool baro_cal_ready = false;

float v_batt = 14.8f, current_a = 0.0f, power_w = 0.0f, consumed_wh = 0.0f;
bool batt_valid = false;
uint8_t current_link_quality = 100;
float current_chip_temp = 25.0f;

bool vtx_uart_ready = false, vtx_hw_unavailable = false, vtx_initial_probe_done = false;
bool vtx_send_freq_req = false, vtx_send_power_req = false, vtx_link_ok = false;
uint8_t vtx_detected_proto = VTX_PROTO_NONE;
uint16_t vtx_actual_mw = 0;
int8_t vtx_actual_dbm = 0;
uint32_t vtx_last_tx_ms = 0, vtx_last_probe_ms = 0, vtx_baud = 0, vtx_step_start_ms = 0;
VtxProbeStep vtx_probe_step = VTX_STEP_IDLE;
uint8_t tramp_rx_buf[16], tramp_rx_idx = 0, sa_rx_buf[16], sa_rx_idx = 0;

uint8_t rtx_active_chip_id = RTX_CHIP_SX1262;
bool rtx_recovering = false;
bool mag_cal_active = false;
uint32_t mag_cal_start_ms = 0, mag_cal_last_sample_ms = 0;
float mag_cal_min_x = 1e9f, mag_cal_max_x = -1e9f, mag_cal_min_y = 1e9f, mag_cal_max_y = -1e9f, mag_cal_min_z = 1e9f, mag_cal_max_z = -1e9f;
float compass_heading_deg = 0.0f;
bool compass_valid = false;

uint8_t i2c_error_counter = 0, rtx_error_counter = 0;
uint8_t rtx_noise_floor[100], rtx_scan_cursor = 0, rtx_best_clean_chan = 15;
bool rtx_scan_active = false, rtx_switched_flag = false;
uint8_t rtx_new_switched_chan = 15, main_rtx_chan = 15, active_rtx_chan = 15;
bool is_on_backup = false, pending_channel_switch = false;
uint8_t target_pending_chan = 15;

bool rtx_tx_in_flight = false, rtx_apply_switch_after_tx = false, rtx_post_tx_backup = false;
uint32_t rtx_tx_start_ms = 0, rtx_tx_timeout_ms = RTX_TX_TIMEOUT_PRI_MS;
uint8_t rtx_post_tx_chan = 15;

PID vzPID, rollRatePID, pitchRatePID, yawRatePID;
bool is_armed = false, is_failsafe = false;
float arm_ramp_factor = 0.0f;
float smooth_throttle = 0.0f, smooth_roll = STICK_CENTER, smooth_pitch = STICK_CENTER, smooth_yaw = STICK_CENTER;
uint8_t mpu_dma_buffer[14];
uint32_t last_rc_packet_ms = 0;
bool send_telemetry_pending = false;
bool err_rc_loss = false, err_batt_low = false, err_imu_fail = false, err_aux_i2c_fail = false;
float dbg_lidar_m = 0.0f;
uint32_t last_lidar_packet_ms = 0;

void loadSettings();
void saveSettings();
void sanitizeConfig();
void resetToFactoryDefaults();
void applyFilterConfig();
void updatePidTunings();
void applyRtxConfiguration(bool use_backup_settings);
void rtx_set_rx_mode();
void startChannelScanNonBlocking();
void stepChannelScanNonBlocking();
void startTelemetryNonBlocking();
void pollTelemetryTxState();
void queueDelayedRtxResponse(DelayedRtxRespType type, uint16_t vtx_mhz, uint8_t tier, uint16_t vtx_mw);
bool hasQueuedRtxResponse();
bool popDelayedRtxResponse(DelayedRtxRespItem &out);
void setupHardwareTimers();
void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap = OUTPUT_MAX);
void resetMotorSlewState();
void flightControlTask();
void parseCli();
float calculateSuperExpoRate(float in, float r, float s, float e);
void setFcErrorRam(uint8_t err_code);
void vtxInit();
void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom);
void vtxSetPowerMw(uint16_t mw, bool send_now, bool save_eeprom);
void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom);
void vtxStepNonBlocking(uint32_t now_ms);
void stopBenchTest();

namespace CliAutoCal {
  void init();
  void pollButton(uint32_t now_ms);
  void start();
  void stop(uint8_t reason_code, uint8_t fc_err_code);
  bool isFlightActive();
  void onDisarmRequested();
  void onArmRequested();
  void onAttitudeAndMotion(float ax_g, float ay_g, float az_g, float roll_deg, float pitch_deg, float gx_dps, float gy_dps);
  bool stepFlight(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps,
                  float gx_rad, float gy_rad, float gz_rad,
                  float roll_deg, float pitch_deg,
                  float &m1, float &m2, float &m3, float &m4);
}

static void flashUnlock() {
  if (Reg::FLASH->CR & (1U << 31)) {
    Reg::FLASH->KEYR = 0x45670123UL;
    Reg::FLASH->KEYR = 0xCDEF89ABUL;
  }
}

static void flashLock() {
  Reg::FLASH->CR |= (1U << 31);
}

static void flashEraseConfigSector() {
  while (Reg::FLASH->SR & (1U << 16)) {}
  Reg::FLASH->CR &= ~((1U << 1) | (0xFU << 3) | (0x3U << 8));
  Reg::FLASH->CR |= (1U << 1) | (FLASH_CFG_SECTOR << 3) | (0x2U << 8);
  Reg::FLASH->CR |= (1U << 16);
  while (Reg::FLASH->SR & (1U << 16)) {}
  Reg::FLASH->CR &= ~((1U << 1) | (0xFU << 3));
  Reg::FLASH->SR  = 0xF3U;
}

static void flashWriteWord(uint32_t addr, uint32_t data) {
  while (Reg::FLASH->SR & (1U << 16)) {}
  Reg::FLASH->CR &= ~(0x3U << 8);
  Reg::FLASH->CR |= (0x2U << 8) | (1U << 0);
  *reinterpret_cast<volatile uint32_t*>(addr) = data;
  while (Reg::FLASH->SR & (1U << 16)) {}
  Reg::FLASH->CR &= ~(1U << 0);
  Reg::FLASH->SR  = 0xF3U;
}

static inline float roundToTenthVolt(float v) { return (float)lroundf(v * 10.0f) * 0.1f; }
static inline float getEffectiveEscDeltaLimit() {
  return constrainF(currentConfig.flt_esc_limit, CFG_ESC_LIMIT_MIN, CFG_ESC_LIMIT_MAX);
}
static inline float getEffectiveBatVMax() {
  return roundToTenthVolt(constrainF(currentConfig.bat_v_max, CFG_BAT_V_MAX_MIN, CFG_BAT_V_MAX_MAX));
}

void sanitizeConfig() {
  currentConfig.fm_mode = (uint8_t)constrainI(currentConfig.fm_mode, 0, 3);
  currentConfig.fm_roll_max = constrainF(currentConfig.fm_roll_max, 1.0f, CFG_ANGLE_MAX_DEG);
  currentConfig.fm_pitch_max = constrainF(currentConfig.fm_pitch_max, 1.0f, CFG_ANGLE_MAX_DEG);
  currentConfig.fm_vz_speed_max = constrainF(currentConfig.fm_vz_speed_max, 0.1f, CFG_VZ_MAX_MPS);

  currentConfig.rc_rate = constrainF(currentConfig.rc_rate, 0.01f, CFG_RATE_MAX);
  currentConfig.rc_super_expo = constrainF(currentConfig.rc_super_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_stick_expo = constrainF(currentConfig.rc_stick_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_yaw_rate = constrainF(currentConfig.rc_yaw_rate, 0.01f, CFG_RATE_MAX);
  currentConfig.rc_yaw_super_expo = constrainF(currentConfig.rc_yaw_super_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_yaw_expo = constrainF(currentConfig.rc_yaw_expo, 0.0f, CFG_EXPO_MAX);
  currentConfig.rc_deadband = constrainF(currentConfig.rc_deadband, 0.0f, CFG_DEADBAND_MAX);
  currentConfig.rc_horizon_transition = constrainF(currentConfig.rc_horizon_transition, 0.05f, 1.0f);
  currentConfig.rc_filter_smoothing = constrainF(currentConfig.rc_filter_smoothing, CFG_ALPHA_MIN, CFG_ALPHA_MAX);

  currentConfig.esc_prot = (uint8_t)constrainI(currentConfig.esc_prot, 0, 1);
  currentConfig.flt_esc_limit = getEffectiveEscDeltaLimit();
  currentConfig.flt_lpf_en = (uint8_t)constrainI(currentConfig.flt_lpf_en, 0, 1);
  currentConfig.flt_not_en = (uint8_t)constrainI(currentConfig.flt_not_en, 0, 1);
  currentConfig.flt_amb_en = (uint8_t)constrainI(currentConfig.flt_amb_en, 0, 1);
  currentConfig.gyro_filter_alpha = constrainF(currentConfig.gyro_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.acc_filter_alpha = constrainF(currentConfig.acc_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.filter_notch_freq = constrainF(currentConfig.filter_notch_freq, CFG_NOTCH_FREQ_MIN, CFG_NOTCH_FREQ_MAX);
  currentConfig.filter_notch_bw = constrainF(currentConfig.filter_notch_bw, CFG_NOTCH_BW_MIN, CFG_NOTCH_BW_MAX);
  currentConfig.flt_amb = constrainF(currentConfig.flt_amb, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.flt_amb_min = constrainF(currentConfig.flt_amb_min, 0.0005f, currentConfig.flt_amb);

  currentConfig.imu_id = (uint8_t)constrainI(currentConfig.imu_id, 0, 6);
  currentConfig.imu_align = (uint8_t)constrainI(currentConfig.imu_align, 0, 3);
  currentConfig.baro_en = (uint8_t)constrainI(currentConfig.baro_en, 0, 1);
  currentConfig.bar_id = (uint8_t)constrainI(currentConfig.bar_id, 0, 7);
  currentConfig.com_en = (uint8_t)constrainI(currentConfig.com_en, 0, 1);
  currentConfig.com_id = (uint8_t)constrainI(currentConfig.com_id, 0, 6);
  currentConfig.com_align = (uint8_t)constrainI(currentConfig.com_align, 0, 3);
  currentConfig.mag_bias_x = constrainF(currentConfig.mag_bias_x, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.mag_bias_y = constrainF(currentConfig.mag_bias_y, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.mag_bias_z = constrainF(currentConfig.mag_bias_z, CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
  currentConfig.lidar_en = (uint8_t)constrainI(currentConfig.lidar_en, 0, 1);
  currentConfig.lid_id = (uint8_t)constrainI(currentConfig.lid_id, 0, 4);
  currentConfig.lidar_filter_alpha = constrainF(currentConfig.lidar_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.lidar_offset_m = constrainF(currentConfig.lidar_offset_m, CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);

  currentConfig.bat_scale_v = constrainF(currentConfig.bat_scale_v, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX);
  currentConfig.bat_v_max = getEffectiveBatVMax();
  currentConfig.bat_comp_en = (uint8_t)constrainI(currentConfig.bat_comp_en, 0, 1);
  currentConfig.bat_filter_alpha = constrainF(currentConfig.bat_filter_alpha, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
  currentConfig.bat_offset_a = constrainF(currentConfig.bat_offset_a, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
  currentConfig.bat_scale_a = constrainF(currentConfig.bat_scale_a, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX);
  currentConfig.bat_fs_v = constrainF(currentConfig.bat_fs_v, CFG_BAT_FS_V_MIN, CFG_BAT_FS_V_MAX);
  currentConfig.fs_bat_en = (uint8_t)constrainI(currentConfig.fs_bat_en, 0, 1);
  currentConfig.fs_rtx_en = (uint8_t)constrainI(currentConfig.fs_rtx_en, 0, 1);
  currentConfig.fs_vz_speed = constrainF(currentConfig.fs_vz_speed, 0.05f, CFG_VZ_MAX_MPS);

  currentConfig.roll_pid.kp = constrainF(currentConfig.roll_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.roll_pid.ki = constrainF(currentConfig.roll_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.roll_pid.kd = constrainF(currentConfig.roll_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.pitch_pid.kp = constrainF(currentConfig.pitch_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.pitch_pid.ki = constrainF(currentConfig.pitch_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.pitch_pid.kd = constrainF(currentConfig.pitch_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.yaw_pid.kp = constrainF(currentConfig.yaw_pid.kp, 0.0f, CFG_RATE_KP_MAX);
  currentConfig.yaw_pid.ki = constrainF(currentConfig.yaw_pid.ki, 0.0f, CFG_RATE_KI_MAX);
  currentConfig.yaw_pid.kd = constrainF(currentConfig.yaw_pid.kd, 0.0f, CFG_RATE_KD_MAX);
  currentConfig.vz_pid.kp = constrainF(currentConfig.vz_pid.kp, 0.0f, CFG_VZ_KP_MAX);
  currentConfig.vz_pid.ki = constrainF(currentConfig.vz_pid.ki, 0.0f, CFG_VZ_KI_MAX);
  currentConfig.vz_pid.kd = constrainF(currentConfig.vz_pid.kd, 0.0f, CFG_VZ_KD_MAX);

  currentConfig.vtx_en = (uint8_t)constrainI(currentConfig.vtx_en, 0, 1);
  currentConfig.vtx_prot = (uint8_t)constrainI(currentConfig.vtx_prot, 0, 2);
  currentConfig.vtx_mhz = (uint16_t)constrainI(currentConfig.vtx_mhz, 5000, 6000);
  currentConfig.vtx_mw = (currentConfig.vtx_mw == 0) ? 25U : (uint16_t)constrainI(currentConfig.vtx_mw, 1, 10000);

  currentConfig.rtx_en = (uint8_t)constrainI(currentConfig.rtx_en, 0, 1);
  currentConfig.rtx_id = (uint8_t)constrainI(currentConfig.rtx_id, 0, 3);
  currentConfig.rtx_band = (uint8_t)constrainI(currentConfig.rtx_band, 1, 9);
  currentConfig.rtx_hw_crc_en = (uint8_t)constrainI(currentConfig.rtx_hw_crc_en, 0, 1);
  currentConfig.rtx_addr_en = (uint8_t)constrainI(currentConfig.rtx_addr_en, 0, 1);
  currentConfig.rtx_iq_inv = (uint8_t)constrainI(currentConfig.rtx_iq_inv, 0, 1);
  currentConfig.rtx_pri_chan = (uint8_t)constrainI(currentConfig.rtx_pri_chan, 0, 99);
  currentConfig.rtx_pri_sf = (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  currentConfig.rtx_pri_bw = (uint8_t)constrainI(currentConfig.rtx_pri_bw, 0, 9);
  currentConfig.rtx_pri_cr = (uint8_t)constrainI(currentConfig.rtx_pri_cr, 1, 4);
  currentConfig.rtx_pri_power = (int8_t)constrainI(currentConfig.rtx_pri_power, -9, 22);
  currentConfig.rtx_bak_chan = (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99);
  currentConfig.rtx_bak_sf = (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12);
  currentConfig.rtx_bak_bw = (uint8_t)constrainI(currentConfig.rtx_bak_bw, 0, 9);
  currentConfig.rtx_bak_cr = (uint8_t)constrainI(currentConfig.rtx_bak_cr, 1, 4);
  currentConfig.rtx_bak_power = (int8_t)constrainI(currentConfig.rtx_bak_power, -9, 22);
}

static inline float getBatteryCompensationFactor() {
  if (!currentConfig.bat_comp_en || !batt_valid || v_batt <= 2.0f || !isfinite(v_batt)) return 1.0f;
  return constrainF(getEffectiveBatVMax() / v_batt, 0.95f, 1.35f);
}
static inline bool isCriticalErrorCode(uint8_t e) {
  return e == FC_ERR_IMU_FAIL || e == FC_ERR_RC_LOSS || e == FC_ERR_BAT_LOW || e == FC_ERR_TILT_DISARM || e == FC_ERR_NAN_INF_MATH;
}
static inline bool hasBlockingArmError(uint8_t e) {
  return e == FC_ERR_IMU_FAIL || e == FC_ERR_CALIB_MISSING || e == FC_ERR_NAN_INF_MATH;
}

void setFcErrorRam(uint8_t err_code) {
  if (err_code == FC_ERR_NONE) return;
  fc_error_ram = err_code;
  fc_latched_error_ram = err_code;
  if (err_code != last_sent_202_err) {
    uint8_t flags = (isCriticalErrorCode(err_code) ? 0x01 : 0x00) | (is_armed ? 0x02 : 0x00);
    queueDelayedRtxResponse(RTX_RESP_202_ERR_EVENT, err_code, flags, 0);
    last_sent_202_err = err_code;
    last_202_send_ms = millis();
  }
}

void serviceError202Event(uint32_t now_ms) {
  if (fc_error_ram == FC_ERR_NONE) { last_sent_202_err = FC_ERR_NONE; return; }
  if (isCriticalErrorCode(fc_error_ram) && (now_ms - last_202_send_ms) >= ERR_202_REPEAT_MS) {
    queueDelayedRtxResponse(RTX_RESP_202_ERR_EVENT, fc_error_ram, (uint8_t)(0x01 | (is_armed ? 0x02 : 0x00)), 0);
    last_sent_202_err = fc_error_ram;
    last_202_send_ms = now_ms;
  }
}

void handleMotorArmStateAndEepromError() {
  if (!prev_armed_state && is_armed) {
    stopBenchTest();
    CliAutoCal::onArmRequested();
    if (hasBlockingArmError(fc_error_ram) || hasBlockingArmError(fc_latched_error_ram)) {
      is_armed = false; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
      prev_armed_state = false; return;
    }
    fc_error_ram = fc_latched_error_ram = last_sent_202_err = FC_ERR_NONE;
    consumed_wh = 0.0f;
    resetMotorSlewState();
  } else if (prev_armed_state && !is_armed) {
    resetMotorSlewState();
    if (fc_latched_error_ram != FC_ERR_NONE) {
      currentConfig.fc_error = fc_latched_error_ram;
      saveSettings();
    }
  }
  prev_armed_state = is_armed;
}

void applyFilterConfig() {
  sanitizeConfig();
  lpf_gx.alpha = lpf_gy.alpha = lpf_gz.alpha = currentConfig.gyro_filter_alpha;
  lpf_ax.alpha = lpf_ay.alpha = lpf_az.alpha = currentConfig.acc_filter_alpha;
  if (!currentConfig.flt_lpf_en) {
    lpf_gx.reset(); lpf_gy.reset(); lpf_gz.reset();
    lpf_ax.reset(); lpf_ay.reset(); lpf_az.reset();
  }
  lpf_battery.alpha = currentConfig.bat_filter_alpha;
  notch_gx.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gy.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  notch_gz.setParameters(250.0f, currentConfig.filter_notch_freq, currentConfig.filter_notch_bw, true);
  if (!currentConfig.flt_not_en) { notch_gx.reset(); notch_gy.reset(); notch_gz.reset(); }
}

void updatePidTunings() {
  sanitizeConfig();
  rollRatePID.updateConfig(currentConfig.roll_pid);
  pitchRatePID.updateConfig(currentConfig.pitch_pid);
  yawRatePID.updateConfig(currentConfig.yaw_pid);
  vzPID.updateConfig(currentConfig.vz_pid);
}

void resetToFactoryDefaults() {
  memset(&currentConfig, 0, sizeof(currentConfig));
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;
  currentConfig.vtx_en = 1; currentConfig.vtx_prot = VTX_PROT_AUTO;
  currentConfig.vtx_mhz = 5658; currentConfig.vtx_mw = 25;
  currentConfig.base_temperature = 25.0f;
  currentConfig.temp_coeff_x = currentConfig.temp_coeff_y = currentConfig.temp_coeff_z = 0.0005f;
  currentConfig.bat_scale_v = 4.85f; currentConfig.bat_scale_a = 1.0f; currentConfig.bat_comp_en = 1;
  currentConfig.bat_v_max = 16.8f; currentConfig.flt_esc_limit = 45.0f;
  currentConfig.fm_mode = 1; currentConfig.fm_roll_max = 25.0f; currentConfig.fm_pitch_max = 25.0f; currentConfig.fm_vz_speed_max = 1.2f;
  currentConfig.rc_rate = 0.35f; currentConfig.rc_super_expo = 0.03f; currentConfig.rc_stick_expo = 0.45f;
  currentConfig.rc_yaw_rate = 0.30f; currentConfig.rc_yaw_super_expo = 0.03f; currentConfig.rc_yaw_expo = 0.35f;
  currentConfig.rc_deadband = 15.0f; currentConfig.rc_horizon_transition = 0.75f; currentConfig.rc_filter_smoothing = 0.35f;
  currentConfig.fs_bat_en = 1; currentConfig.fs_rtx_en = 1; currentConfig.fs_vz_speed = 0.5f;
  currentConfig.bat_fs_v = 13.8f; currentConfig.bat_filter_alpha = 0.05f;
  currentConfig.lidar_en = 1; currentConfig.lidar_filter_alpha = 0.15f;
  currentConfig.baro_en = 1; currentConfig.com_en = 0;
  currentConfig.flt_amb_en = 1; currentConfig.flt_lpf_en = 1; currentConfig.flt_not_en = 1;
  currentConfig.gyro_filter_alpha = 0.10f; currentConfig.acc_filter_alpha = 0.10f;
  currentConfig.filter_notch_freq = 120.0f; currentConfig.filter_notch_bw = 20.0f;
  currentConfig.flt_amb = DEFAULT_FLT_AMB; currentConfig.flt_amb_min = DEFAULT_FLT_AMB_MIN;
  currentConfig.rtx_en = 1; currentConfig.rtx_band = 9; currentConfig.rtx_hw_crc_en = 1;
  currentConfig.rtx_addr_en = 1; currentConfig.rtx_sync_word = 0x1424;
  currentConfig.rtx_pri_chan = 15; currentConfig.rtx_pri_sf = 7; currentConfig.rtx_pri_bw = 7; currentConfig.rtx_pri_cr = 1; currentConfig.rtx_pri_power = 22;
  currentConfig.rtx_bak_chan = 25; currentConfig.rtx_bak_sf = 9; currentConfig.rtx_bak_bw = 7; currentConfig.rtx_bak_cr = 1; currentConfig.rtx_bak_power = 22;
  currentConfig.roll_pid = {0.45f, 0.010f, 0.018f};
  currentConfig.pitch_pid = {0.45f, 0.010f, 0.018f};
  currentConfig.yaw_pid = {0.60f, 0.010f, 0.0f};
  currentConfig.vz_pid = {7.0f, 1.2f, 0.30f};

  sanitizeConfig();
  vtx_link_ok = false; vtx_detected_proto = VTX_PROTO_NONE;
  main_rtx_chan = active_rtx_chan = currentConfig.rtx_pri_chan;
  is_on_backup = false;

  rollRatePID.init(currentConfig.roll_pid, 200.0f);
  pitchRatePID.init(currentConfig.pitch_pid, 200.0f);
  yawRatePID.init(currentConfig.yaw_pid, 200.0f);
  vzPID.init(currentConfig.vz_pid, 300.0f);
  adaptive_lidar.reset();
  resetMotorSlewState();
  applyFilterConfig();
  saveSettings();
}

void loadSettings() {
  memcpy(&currentConfig, reinterpret_cast<const void*>(FLASH_CFG_ADDR), sizeof(FlightConfig));
  if (currentConfig.magic_marker != EEPROM_MAGIC_MARKER) {
    resetToFactoryDefaults();
    setFcErrorRam(FC_ERR_EEPROM_RESET);
    return;
  }
  sanitizeConfig();
  rollRatePID.init(currentConfig.roll_pid, 200.0f);
  pitchRatePID.init(currentConfig.pitch_pid, 200.0f);
  yawRatePID.init(currentConfig.yaw_pid, 200.0f);
  vzPID.init(currentConfig.vz_pid, 300.0f);
  if (currentConfig.fc_error != FC_ERR_NONE) fc_latched_error_ram = currentConfig.fc_error;
}

void saveSettings() {
  if (is_armed || CliAutoCal::isFlightActive()) return;
  sanitizeConfig();
  currentConfig.magic_marker = EEPROM_MAGIC_MARKER;

  const uint8_t* src = reinterpret_cast<const uint8_t*>(&currentConfig);
  flashUnlock();
  flashEraseConfigSector();
  for (size_t i = 0; i < sizeof(FlightConfig); i += 4) {
    uint32_t w = (uint32_t)src[i] |
                 ((uint32_t)src[i + 1] << 8) |
                 ((uint32_t)src[i + 2] << 16) |
                 ((uint32_t)src[i + 3] << 24);
    flashWriteWord(FLASH_CFG_ADDR + i, w);
  }
  flashLock();
}

inline bool hasQueuedRtxResponse() { return (rtx_resp_count > 0) && (rtx_resp_queue[rtx_resp_head].type != RTX_RESP_NONE); }

void queueDelayedRtxResponse(DelayedRtxRespType type, uint16_t vtx_mhz, uint8_t tier, uint16_t vtx_mw) {
  if (type == RTX_RESP_NONE) return;
  if (rtx_resp_count >= RTX_RESP_QUEUE_SIZE) {
    if (type != RTX_RESP_202_ERR_EVENT) return;
    rtx_resp_head = (rtx_resp_head + 1) % RTX_RESP_QUEUE_SIZE;
    rtx_resp_count--;
  }
  rtx_resp_queue[rtx_resp_tail] = {type, vtx_mhz, tier, vtx_mw};
  rtx_resp_tail = (rtx_resp_tail + 1) % RTX_RESP_QUEUE_SIZE;
  rtx_resp_count++;
}

bool popDelayedRtxResponse(DelayedRtxRespItem &out) {
  if (rtx_resp_count == 0) { out.type = RTX_RESP_NONE; return false; }
  out = rtx_resp_queue[rtx_resp_head];
  rtx_resp_queue[rtx_resp_head].type = RTX_RESP_NONE;
  rtx_resp_head = (rtx_resp_head + 1) % RTX_RESP_QUEUE_SIZE;
  rtx_resp_count--;
  return true;
}

uint32_t getTelemetryTxTimeoutMs(bool backup_mode) {
  uint8_t sf = backup_mode ? (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12) : (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  uint32_t t = backup_mode ? RTX_TX_TIMEOUT_BAK_MS : RTX_TX_TIMEOUT_PRI_MS;
  if (sf >= 11) t += 140; else if (sf >= 10) t += 70;
  return t;
}

void checkRtxBusAndRecover() {
  if (rtx_recovering || rtx_error_counter < RTX_MAX_ERRORS) return;
  rtx_recovering = true;
  setFcErrorRam(FC_ERR_SPI_RTX);
  gpioWrite(Reg::GPIOB, 0, false);
  uint32_t s = micros(); while ((micros() - s) < 2000) {}
  gpioWrite(Reg::GPIOB, 0, true);
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_error_counter = 0;
  applyRtxConfiguration(is_on_backup);
  rtx_recovering = false;
}

inline bool rtx_try_ready_short(uint32_t timeout_us = RTX_BUSY_SHORT_TIMEOUT_US) {
  uint32_t s = micros();
  while (gpioRead(Reg::GPIOB, 1)) {
    if ((micros() - s) >= timeout_us) {
      rtx_error_counter++; setFcErrorRam(FC_ERR_SPI_RTX); checkRtxBusAndRecover(); return false;
    }
  }
  return true;
}

void rtx_write_command(uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  spi2BeginTx();
  spi2Transfer(cmd);
  if (data && len > 0) {
    if (len >= 2) spi2WriteDma(data, len);
    else {
      for (uint8_t i = 0; i < len; i++) spi2Transfer(data[i]);
    }
  }
  spi2EndTx();
  rtx_error_counter = 0;
}

void rtx_read_command(uint8_t cmd, uint8_t *data, uint8_t len) {
  if (!rtx_try_ready_short()) return;
  spi2BeginTx(); spi2Transfer(cmd); spi2Transfer(0x00);
  for (uint8_t i = 0; i < len; i++) data[i] = spi2Transfer(0x00);
  spi2EndTx(); rtx_error_counter = 0;
}

static uint8_t detectOrSelectRtxChip() {
  uint8_t req = currentConfig.rtx_id;
  if (req == RTX_CHIP_SX1262 || req == RTX_CHIP_SX1268 || req == RTX_CHIP_LLCC68) return req;
  uint8_t st[2] = {0, 0}; rtx_read_command(0xC0, st, 2);
  return (currentConfig.rtx_band <= 5) ? RTX_CHIP_SX1268 : RTX_CHIP_SX1262;
}

void rtx_init_hardware() {
  initSpi2();
  gpioWrite(Reg::GPIOB, 0, false); delay(5); gpioWrite(Reg::GPIOB, 0, true); delay(10);
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();
}

static uint32_t rtxFreqToReg(uint32_t freq_hz) {
  return (uint32_t)(((uint64_t)freq_hz << 25) / 32000000UL);
}

void applyRtxConfiguration(bool use_backup_settings) {
  if (!currentConfig.rtx_en || !rtx_try_ready_short()) return;
  rtx_tx_in_flight = rtx_apply_switch_after_tx = rtx_scan_active = false;
  rtx_active_chip_id = detectOrSelectRtxChip();

  uint8_t stby = 0x00; rtx_write_command(0x80, &stby, 1);
  uint8_t reg_mode = 0x01; rtx_write_command(0x96, &reg_mode, 1);
  uint8_t pkt_type = 0x01; rtx_write_command(0x8A, &pkt_type, 1);

  if (currentConfig.rtx_addr_en) {
    uint8_t ab[2] = {(uint8_t)(currentConfig.rtx_addr >> 8), (uint8_t)(currentConfig.rtx_addr & 0xFF)};
    rtx_write_command(0x6C, ab, 2);
  }
  uint8_t sb[2] = {(uint8_t)(currentConfig.rtx_sync_word >> 8), (uint8_t)(currentConfig.rtx_sync_word & 0xFF)};
  rtx_write_command(0x74, sb, 2);

  uint8_t band = (uint8_t)constrainI(currentConfig.rtx_band, 1, 9);
  uint8_t chan = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99) : (uint8_t)constrainI(active_rtx_chan, 0, 99);
  uint32_t freq_reg = rtxFreqToReg((uint32_t)band * 100000000UL + (uint32_t)chan * 1000000UL);
  uint8_t fb[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
  rtx_write_command(0x86, fb, 4);

  uint8_t sf = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_sf, 5, 12) : (uint8_t)constrainI(currentConfig.rtx_pri_sf, 5, 12);
  uint8_t bw = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_bw, 0, 9) : (uint8_t)constrainI(currentConfig.rtx_pri_bw, 0, 9);
  if (rtx_active_chip_id == RTX_CHIP_LLCC68) sf = (uint8_t)constrainI(sf, 5, 11);
  uint8_t cr = use_backup_settings ? (uint8_t)constrainI(currentConfig.rtx_bak_cr, 1, 4) : (uint8_t)constrainI(currentConfig.rtx_pri_cr, 1, 4);
  uint8_t mp[4] = {sf, bw, cr, 0x00};
  rtx_write_command(0x8B, mp, 4);

  uint8_t pp[6] = {0x00, 0x08, 0x00, RTX_RX_MAX_PAYLOAD_BYTES, (uint8_t)(currentConfig.rtx_hw_crc_en ? 1 : 0), (uint8_t)(currentConfig.rtx_iq_inv ? 1 : 0)};
  rtx_write_command(0x8C, pp, 6);

  int8_t pwr = use_backup_settings ? (int8_t)constrainI(currentConfig.rtx_bak_power, -9, 22) : (int8_t)constrainI(currentConfig.rtx_pri_power, -9, 22);
  uint8_t pa[4] = {0x04, 0x07, 0x00, 0x01}; rtx_write_command(0x95, pa, 4);
  uint8_t tp[2] = {(uint8_t)pwr, 0x02}; rtx_write_command(0x8E, tp, 2);

  rtx_set_rx_mode();
}

void rtx_set_rx_mode() {
  if (!currentConfig.rtx_en) return;
  uint8_t p[3] = {0xFF, 0xFF, 0xFF};
  rtx_write_command(0x82, p, 3);
}

void startChannelScanNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  rtx_scan_active = true; rtx_scan_cursor = 0;
}

void stepChannelScanNonBlocking() {
  if (!rtx_scan_active || !currentConfig.rtx_en || rtx_tx_in_flight || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint32_t base_reg = rtxFreqToReg((uint32_t)constrainI(currentConfig.rtx_band, 1, 9) * 100000000UL);
  uint32_t step_reg = rtxFreqToReg(1000000UL);
  uint8_t end_chan = (uint8_t)(rtx_scan_cursor + RTX_SCAN_CHANNELS_PER_TICK);
  if (end_chan > 100) end_chan = 100;
  uint32_t freq_reg = base_reg + ((uint32_t)rtx_scan_cursor * step_reg);

  for (uint8_t c = rtx_scan_cursor; c < end_chan; c++) {
    uint32_t w = micros();
    while (gpioRead(Reg::GPIOB, 1)) {
      if ((micros() - w) > RTX_SCAN_BUSY_TIMEOUT_US) { setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return; }
    }
    uint8_t fb[4] = {(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16), (uint8_t)(freq_reg >> 8), (uint8_t)freq_reg};
    spi2BeginTx(); spi2Transfer(0x86); spi2WriteDma(fb, 4); spi2EndTx();

    w = micros();
    while (gpioRead(Reg::GPIOB, 1)) {
      if ((micros() - w) > RTX_SCAN_BUSY_TIMEOUT_US) { setFcErrorRam(FC_ERR_SPI_RTX); rtx_scan_active = false; applyRtxConfiguration(is_on_backup); return; }
    }
    spi2BeginTx(); spi2Transfer(0x15); spi2Transfer(0x00); rtx_noise_floor[c] = spi2Transfer(0x00); spi2EndTx();
    freq_reg += step_reg;
  }

  rtx_scan_cursor = end_chan;
  if (rtx_scan_cursor >= 100) {
    uint8_t best = 0, min_n = 255;
    for (uint8_t i = 0; i < 100; i++) {
      if (rtx_noise_floor[i] < min_n) { min_n = rtx_noise_floor[i]; best = i; }
    }
    rtx_best_clean_chan = best; rtx_scan_active = false;
    queueDelayedRtxResponse(RTX_RESP_230_SCAN_BEST, rtx_best_clean_chan, 0, 0);

    uint32_t wr = base_reg + ((uint32_t)constrainI(active_rtx_chan, 0, 99) * step_reg);
    uint8_t wb[4] = {(uint8_t)(wr >> 24), (uint8_t)(wr >> 16), (uint8_t)(wr >> 8), (uint8_t)wr};
    rtx_write_command(0x86, wb, 4);
    rtx_set_rx_mode();
  }
}

void readRx() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint8_t irq_st[2] = {0, 0}; rtx_read_command(0x12, irq_st, 2);
  uint16_t irq = ((uint16_t)irq_st[0] << 8) | irq_st[1];
  if (irq & 0x02) {
    uint8_t clr[2] = {0x00, 0x02}; rtx_write_command(0x02, clr, 2);
    if (!rtx_try_ready_short()) { rtx_set_rx_mode(); return; }

    uint8_t bs[2] = {0, 0}; rtx_read_command(0x13, bs, 2);
    uint8_t plen = bs[0], sptr = bs[1];
    if (plen == 0 || plen > RTX_RX_MAX_PAYLOAD_BYTES) { rtx_set_rx_mode(); return; }

    spi2BeginTx(); spi2Transfer(0x1E); spi2Transfer(sptr);
    uint8_t raw[RTX_RX_MAX_PAYLOAD_BYTES] = {0};
    for (uint8_t i = 0; i < plen; i++) raw[i] = spi2Transfer(0x00);
    spi2EndTx();

    uint8_t idx = 0;
    if (currentConfig.rtx_addr_en) {
      if (plen < (idx + 2)) { rtx_set_rx_mode(); return; }
      uint16_t addr = ((uint16_t)raw[idx] << 8) | raw[idx + 1]; idx += 2;
      if (addr != currentConfig.rtx_addr) { rtx_set_rx_mode(); return; }
    }
    if (plen < (uint8_t)(idx + 2)) { rtx_set_rx_mode(); return; }

    uint8_t cmd = raw[idx++], lq = raw[idx++], rem = (uint8_t)(plen - idx);
    if (cmd == 110 && rem >= 4) {
      current_link_quality = lq;
      float t_v = (float)raw[idx++] * 10.0f, r_v = (float)raw[idx++] * 10.0f;
      float p_v = (float)raw[idx++] * 10.0f, y_v = (float)raw[idx++] * 10.0f;
      float a = constrainF(currentConfig.rc_filter_smoothing, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
      smooth_throttle = a * t_v + (1.0f - a) * smooth_throttle;
      smooth_roll = a * r_v + (1.0f - a) * smooth_roll;
      smooth_pitch = a * p_v + (1.0f - a) * smooth_pitch;
      smooth_yaw = a * y_v + (1.0f - a) * smooth_yaw;
      last_rc_packet_ms = millis(); send_telemetry_pending = true;
    } else if (cmd == 120 && rem >= 1) {
      current_link_quality = lq;
      if (raw[idx++] == 0) { CliAutoCal::onDisarmRequested(); is_armed = false; }
      else if (!is_failsafe) is_armed = true;
      last_rc_packet_ms = millis();
    } else if (cmd == 130) {
      current_link_quality = lq; startChannelScanNonBlocking(); last_rc_packet_ms = millis();
    } else if (cmd == 140 && rem >= 1) {
      current_link_quality = lq;
      uint8_t tc = (uint8_t)constrainI(raw[idx++], 0, 99);
      target_pending_chan = tc; pending_channel_switch = true;
      rtx_switched_flag = true; rtx_new_switched_chan = tc;
      queueDelayedRtxResponse(RTX_RESP_240_CHAN_SW, tc, 0, 0);
      last_rc_packet_ms = millis();
    } else if (cmd == 150 && rem >= 4) {
      current_link_quality = lq;
      uint16_t mhz = ((uint16_t)raw[idx] << 8) | raw[idx + 1];
      uint16_t mw = ((uint16_t)raw[idx + 2] << 8) | raw[idx + 3];
      vtxSetFrequencyAndPower(mhz, mw, true);
      queueDelayedRtxResponse(RTX_RESP_250_VTX_ACK, currentConfig.vtx_mhz, 0, currentConfig.vtx_mw);
      last_rc_packet_ms = millis();
    }
  }
  rtx_set_rx_mode();
}

static uint8_t getActiveStatusErrorCode() {
  return (fc_error_ram != FC_ERR_NONE) ? fc_error_ram : fc_latched_error_ram;
}

void startTelemetryNonBlocking() {
  if (!currentConfig.rtx_en || rtx_tx_in_flight || rtx_scan_active || rtx_recovering) return;
  if (gpioRead(Reg::GPIOB, 1)) return;

  uint8_t stby = 0x00; rtx_write_command(0x80, &stby, 1);
  if (!rtx_try_ready_short()) return;

  uint8_t txp[32]; uint8_t p = 0;
  if (currentConfig.rtx_addr_en) {
    txp[p++] = (uint8_t)(currentConfig.rtx_addr >> 8); txp[p++] = (uint8_t)(currentConfig.rtx_addr & 0xFF);
  }

  DelayedRtxRespItem qi;
  if (hasQueuedRtxResponse() && popDelayedRtxResponse(qi)) {
    txp[p++] = (uint8_t)qi.type; txp[p++] = current_link_quality;
    if (qi.type == RTX_RESP_202_ERR_EVENT) { txp[p++] = (uint8_t)qi.vtx_mhz; txp[p++] = qi.tier; }
    else if (qi.type == RTX_RESP_230_SCAN_BEST || qi.type == RTX_RESP_240_CHAN_SW) txp[p++] = (uint8_t)qi.vtx_mhz;
    else if (qi.type == RTX_RESP_250_VTX_ACK) {
      txp[p++] = (uint8_t)(qi.vtx_mhz >> 8); txp[p++] = (uint8_t)(qi.vtx_mhz & 0xFF);
      txp[p++] = (uint8_t)(qi.vtx_mw >> 8);  txp[p++] = (uint8_t)(qi.vtx_mw & 0xFF);
    }
  } else {
    txp[p++] = 210; txp[p++] = current_link_quality;
    uint16_t vc = (uint16_t)constrainL(lroundf(v_batt * 100.0f), 0L, 65535L);
    uint16_t ic = (uint16_t)constrainL(lroundf(current_a * 100.0f), 0L, 65535L);
    uint16_t wd = (uint16_t)constrainL(lroundf(power_w * 10.0f), 0L, 65535L);
    uint16_t whm = (uint16_t)constrainL(lroundf(consumed_wh * 1000.0f), 0L, 65535L);
    int16_t acm = (int16_t)constrainL(lroundf(baro_alt_m * 100.0f), -32768L, 32767L);
    uint16_t hd = (uint16_t)constrainL(lroundf(compass_heading_deg * 10.0f), 0L, 3600L);
    uint16_t lm = (uint16_t)constrainL(lroundf(dbg_lidar_m * 1000.0f), 0L, 65535L);
    int16_t tc10 = (int16_t)constrainL(lroundf(current_chip_temp * 10.0f), -32768L, 32767L);
    txp[p++] = (uint8_t)(vc >> 8);  txp[p++] = (uint8_t)(vc & 0xFF);
    txp[p++] = (uint8_t)(ic >> 8);  txp[p++] = (uint8_t)(ic & 0xFF);
    txp[p++] = (uint8_t)(wd >> 8);  txp[p++] = (uint8_t)(wd & 0xFF);
    txp[p++] = (uint8_t)(whm >> 8); txp[p++] = (uint8_t)(whm & 0xFF);
    txp[p++] = (uint8_t)((uint16_t)acm >> 8); txp[p++] = (uint8_t)((uint16_t)acm & 0xFF);
    txp[p++] = (uint8_t)(hd >> 8);  txp[p++] = (uint8_t)(hd & 0xFF);
    txp[p++] = (uint8_t)(lm >> 8);  txp[p++] = (uint8_t)(lm & 0xFF);
    txp[p++] = (uint8_t)((uint16_t)tc10 >> 8); txp[p++] = (uint8_t)((uint16_t)tc10 & 0xFF);
    txp[p++] = (uint8_t)(is_armed ? 1 : 0);
    txp[p++] = getActiveStatusErrorCode();
  }

  uint8_t bc[2] = {0x00, 0x00}; rtx_write_command(0x8F, bc, 2);
  spi2BeginTx(); spi2Transfer(0x0E); spi2Transfer(0x00);
  spi2WriteDma(txp, p);
  spi2EndTx();

  uint8_t pp[6] = {0x00, 0x08, 0x00, p, (uint8_t)(currentConfig.rtx_hw_crc_en ? 1 : 0), (uint8_t)(currentConfig.rtx_iq_inv ? 1 : 0)};
  rtx_write_command(0x8C, pp, 6);
  uint8_t txm[3] = {0x00, 0x00, 0x00}; rtx_write_command(0x83, txm, 3);

  rtx_tx_in_flight = true; rtx_tx_start_ms = millis();
  rtx_tx_timeout_ms = getTelemetryTxTimeoutMs(is_on_backup);
  send_telemetry_pending = false;

  if (pending_channel_switch) {
    pending_channel_switch = false; main_rtx_chan = target_pending_chan;
    rtx_apply_switch_after_tx = true; rtx_post_tx_chan = main_rtx_chan; rtx_post_tx_backup = false;
  }
}

void pollTelemetryTxState() {
  if (!rtx_tx_in_flight) return;
  uint32_t now_ms = millis();
  bool done = false, tout = false;

  if (!gpioRead(Reg::GPIOB, 1)) {
    uint8_t ist[2] = {0, 0}; rtx_read_command(0x12, ist, 2);
    if ((((uint16_t)ist[0] << 8) | ist[1]) & 0x01) {
      uint8_t clr[2] = {0x00, 0x01}; rtx_write_command(0x02, clr, 2); done = true;
    }
  }
  if (!done && ((now_ms - rtx_tx_start_ms) >= rtx_tx_timeout_ms)) {
    tout = true; rtx_error_counter++; setFcErrorRam(FC_ERR_RTX_TX_TIMEOUT);
    if (!is_on_backup) {
      is_on_backup = true; active_rtx_chan = (uint8_t)constrainI(currentConfig.rtx_bak_chan, 0, 99);
      rtx_apply_switch_after_tx = true; rtx_post_tx_chan = active_rtx_chan; rtx_post_tx_backup = true;
    }
  }
  if (done || tout) {
    rtx_tx_in_flight = false;
    if (rtx_apply_switch_after_tx) {
      rtx_apply_switch_after_tx = false; is_on_backup = rtx_post_tx_backup; active_rtx_chan = rtx_post_tx_chan;
      applyRtxConfiguration(is_on_backup);
    } else rtx_set_rx_mode();
  }
}

static inline uint16_t vtxClampMw(uint16_t mw) {
  return (mw == 0) ? 25U : (uint16_t)constrainI((int)mw, 1, 10000);
}

static uint8_t vtxMwToDbmInt(uint16_t mw) {
  mw = vtxClampMw(mw);
  if (mw >= 10000) return 40;
  uint8_t db = 0; uint32_t v = mw;
  while (v >= 10) { v /= 10; db += 10; }
  if (v >= 8) db += 9; else if (v >= 6) db += 8; else if (v >= 5) db += 7;
  else if (v >= 4) db += 6; else if (v >= 3) db += 5; else if (v >= 2) db += 3;
  return (uint8_t)constrainI((int)db, 0, 40);
}

static uint16_t vtxDbmToMwInt(uint8_t dbm) {
  static const uint16_t kDec[4] = {1, 2, 4, 6};
  uint8_t d = (uint8_t)constrainI((int)dbm, 0, 40);
  uint32_t mw = kDec[d % 10 < 3 ? 0 : (d % 10 < 5 ? 1 : (d % 10 < 8 ? 2 : 3))];
  for (uint8_t b = 0; b < (d / 10); b++) mw *= 10;
  return (uint16_t)constrainL((long)mw, 1L, 10000L);
}

static inline uint8_t vtxEncodeSaDirectDbm(uint16_t mw) {
  return (uint8_t)(0x80 | (vtxMwToDbmInt(mw) & 0x7F));
}

static void vtxEnsureBaud(uint32_t baud) {
  if (!vtx_uart_ready || vtx_baud != baud) {
    if (vtx_uart_ready) { uart6.flush(); uart6.end(); }
    uart6.begin(baud);
    vtx_baud = baud; vtx_uart_ready = true; tramp_rx_idx = sa_rx_idx = 0;
    while (uart6.available() > 0) (void)uart6.read();
  }
}

static uint8_t smartAudioCrc8(const uint8_t *d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t trampCrc8(const uint8_t *f) {
  uint16_t s = 0; for (uint8_t i = 1; i <= 13; i++) s += f[i]; return (uint8_t)(s & 0xFF);
}

static void vtxMarkFail() { vtx_link_ok = false; setFcErrorRam(FC_ERR_SA_VTX_FAIL); }

static void vtxMarkOk(uint8_t proto, uint32_t now_ms) {
  if (proto == VTX_PROTO_SMARTAUDIO || proto == VTX_PROTO_TRAMP) vtx_detected_proto = proto;
  vtx_link_ok = true; vtx_last_probe_ms = now_ms; vtx_initial_probe_done = true;
}

static uint8_t vtxResolveProto() {
  if (currentConfig.vtx_prot == VTX_PROT_TRAMP) return VTX_PROTO_TRAMP;
  if (currentConfig.vtx_prot == VTX_PROT_SMARTAUDIO) return VTX_PROTO_SMARTAUDIO;
  return vtx_detected_proto;
}

static bool pollSmartAudioRx() {
  bool seen = false;
  while (uart6.available() > 0) {
    uint8_t b = (uint8_t)uart6.read();
    if (sa_rx_idx == 0) { if (b == 0xAA) sa_rx_buf[sa_rx_idx++] = b; }
    else if (sa_rx_idx == 1) { if (b == 0x55) sa_rx_buf[sa_rx_idx++] = b; else sa_rx_idx = (b == 0xAA) ? 1 : 0; }
    else {
      sa_rx_buf[sa_rx_idx++] = b;
      if (sa_rx_idx >= 4) {
        uint8_t plen = sa_rx_buf[3];
        if (plen > 10) { sa_rx_idx = 0; continue; }
        uint8_t tlen = (uint8_t)(4 + plen + 1);
        if (sa_rx_idx == tlen) {
          if (smartAudioCrc8(sa_rx_buf, (uint8_t)(tlen - 1)) == sa_rx_buf[tlen - 1]) {
            if (plen >= 5) {
              uint8_t dbm = sa_rx_buf[8] & 0x7F;
              if (dbm <= 40) { vtx_actual_dbm = (int8_t)dbm; vtx_actual_mw = vtxDbmToMwInt(dbm); }
            }
            seen = true;
          }
          sa_rx_idx = 0;
        }
      }
    }
  }
  return seen;
}

static bool pollTrampRx() {
  bool seen = false;
  while (uart6.available() > 0) {
    uint8_t b = (uint8_t)uart6.read();
    if (tramp_rx_idx == 0) { if (b == '$') tramp_rx_buf[tramp_rx_idx++] = b; }
    else {
      tramp_rx_buf[tramp_rx_idx++] = b;
      if (tramp_rx_idx == 16) {
        if (tramp_rx_buf[15] == 0x00 && tramp_rx_buf[14] == trampCrc8(tramp_rx_buf)) {
          uint16_t v16 = (uint16_t)tramp_rx_buf[2] | ((uint16_t)tramp_rx_buf[3] << 8);
          if ((tramp_rx_buf[1] == 'r' || tramp_rx_buf[1] == 'v') && v16 > 0 && v16 <= 10000) {
            vtx_actual_mw = v16; vtx_actual_dbm = (int8_t)vtxMwToDbmInt(v16);
          }
          seen = true;
        }
        tramp_rx_idx = 0;
      }
    }
  }
  return seen;
}

static bool sendSmartAudioFrame(uint8_t cmd, const uint8_t *pl, uint8_t plen) {
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return false; }
  vtxEnsureBaud(SMARTAUDIO_BAUD_RATE);
  sa_rx_idx = 0; while (uart6.available() > 0) (void)uart6.read();
  uint8_t f[12], idx = 0;
  f[idx++] = 0xAA; f[idx++] = 0x55; f[idx++] = cmd; f[idx++] = plen;
  for (uint8_t i = 0; i < plen && idx < 11; i++) f[idx++] = pl[i];
  f[idx] = smartAudioCrc8(f, idx); idx++;
  if (uart6.write(f, idx) != idx) { vtxMarkFail(); return false; }
  return true;
}

static bool sendTrampFrame(char cmd, uint16_t param) {
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return false; }
  vtxEnsureBaud(TRAMP_BAUD_RATE);
  tramp_rx_idx = 0; while (uart6.available() > 0) (void)uart6.read();
  uint8_t f[16]; memset(f, 0, 16);
  f[0] = '$'; f[1] = (uint8_t)cmd; f[2] = (uint8_t)(param & 0xFF); f[3] = (uint8_t)((param >> 8) & 0xFF);
  f[14] = trampCrc8(f);
  if (uart6.write(f, 16) != 16) { vtxMarkFail(); return false; }
  return true;
}

static bool vtxSendFreqCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('F', currentConfig.vtx_mhz);
  uint8_t fb[2] = {(uint8_t)(currentConfig.vtx_mhz >> 8), (uint8_t)(currentConfig.vtx_mhz & 0xFF)};
  return sendSmartAudioFrame(0x09, fb, 2);
}

static bool vtxSendPowerCmd(uint8_t proto) {
  if (proto == VTX_PROTO_TRAMP) return sendTrampFrame('P', currentConfig.vtx_mw);
  uint8_t pb = vtxEncodeSaDirectDbm(currentConfig.vtx_mw);
  return sendSmartAudioFrame(0x05, &pb, 1);
}

static void vtxArmCmdAck(uint32_t now_ms) {
  vtx_last_tx_ms = vtx_step_start_ms = now_ms;
  vtx_probe_step = VTX_STEP_WAIT_CMD_ACK;
}

void vtxSetFrequencyMhz(uint16_t mhz, bool send_now, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrainI((int)mhz, 5000, 6000);
  vtx_send_freq_req = true;
  if (send_now && currentConfig.vtx_en) {
    uint8_t proto = vtxResolveProto();
    if ((proto == VTX_PROTO_TRAMP || proto == VTX_PROTO_SMARTAUDIO) && vtxSendFreqCmd(proto)) {
      vtx_send_freq_req = false; vtxArmCmdAck(millis());
    }
  }
  if (save_eeprom) saveSettings();
}

void vtxSetPowerMw(uint16_t mw, bool send_now, bool save_eeprom) {
  currentConfig.vtx_mw = vtxClampMw(mw);
  vtx_send_power_req = true;
  if (send_now && currentConfig.vtx_en) {
    uint8_t proto = vtxResolveProto();
    if ((proto == VTX_PROTO_TRAMP || proto == VTX_PROTO_SMARTAUDIO) && vtxSendPowerCmd(proto)) {
      vtx_send_power_req = false; vtxArmCmdAck(millis());
    }
  }
  if (save_eeprom) saveSettings();
}

void vtxSetFrequencyAndPower(uint16_t mhz, uint16_t power_mw, bool save_eeprom) {
  currentConfig.vtx_mhz = (uint16_t)constrainI((int)mhz, 5000, 6000);
  currentConfig.vtx_mw = vtxClampMw(power_mw);
  vtx_send_freq_req = vtx_send_power_req = true;
  if (save_eeprom) saveSettings();
}

void vtxInit() {
  gpioConfigPin(Reg::GPIOC, 6, GPIO_AF, GPIO_PULL_UP, 8, false);
  gpioConfigPin(Reg::GPIOC, 7, GPIO_AF, GPIO_PULL_UP, 8, false);

  vtx_hw_unavailable = vtx_initial_probe_done = false;
  vtx_probe_step = VTX_STEP_IDLE;
  vtx_step_start_ms = vtx_baud = 0;
  vtx_link_ok = false; vtx_detected_proto = VTX_PROTO_NONE;
  vtx_actual_dbm = 0; vtx_actual_mw = 0; tramp_rx_idx = sa_rx_idx = 0;

  if (!currentConfig.vtx_en) {
    vtx_uart_ready = vtx_send_freq_req = vtx_send_power_req = false;
    return;
  }
  uint8_t proto = vtxResolveProto();
  vtxEnsureBaud((proto == VTX_PROTO_TRAMP) ? TRAMP_BAUD_RATE : SMARTAUDIO_BAUD_RATE);
  vtx_send_freq_req = vtx_send_power_req = true;
  vtx_last_probe_ms = millis();
}

static void vtxFinishProbeTimeout(uint32_t now_ms) {
  vtx_initial_probe_done = true; vtx_last_probe_ms = now_ms; vtxMarkFail(); vtx_probe_step = VTX_STEP_IDLE;
}

void vtxStepNonBlocking(uint32_t now_ms) {
  if (!currentConfig.vtx_en) { vtx_link_ok = false; vtx_probe_step = VTX_STEP_IDLE; return; }
  if (vtx_hw_unavailable) { setFcErrorRam(FC_ERR_SA_VTX_FAIL); return; }
  if (!vtx_uart_ready) { vtxInit(); if (!vtx_uart_ready) return; }

  uint8_t known = vtxResolveProto();
  bool auto_fb = (currentConfig.vtx_prot == VTX_PROT_AUTO) && (known == VTX_PROTO_NONE);

  if (vtx_probe_step == VTX_STEP_WAIT_SA) {
    if (pollSmartAudioRx()) { vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms); vtx_probe_step = VTX_STEP_IDLE; return; }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) {
      if (auto_fb && sendTrampFrame('v', 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_TRAMP; }
      else vtxFinishProbeTimeout(now_ms);
    }
    return;
  }
  if (vtx_probe_step == VTX_STEP_WAIT_TRAMP) {
    if (pollTrampRx()) { vtxMarkOk(VTX_PROTO_TRAMP, now_ms); vtx_probe_step = VTX_STEP_IDLE; return; }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }
  if (vtx_probe_step == VTX_STEP_WAIT_CMD_ACK) {
    bool ack = (known == VTX_PROTO_TRAMP) ? pollTrampRx() : pollSmartAudioRx();
    if (ack) {
      uint8_t ok_proto = (known != VTX_PROTO_NONE) ? known : (uint8_t)VTX_PROTO_SMARTAUDIO;
      vtxMarkOk(ok_proto, now_ms);
      vtx_probe_step = VTX_STEP_IDLE;
      return;
    }
    if ((now_ms - vtx_step_start_ms) >= VTX_PROBE_TIMEOUT_MS) vtxFinishProbeTimeout(now_ms);
    return;
  }

  if (known == VTX_PROTO_TRAMP && pollTrampRx()) vtxMarkOk(VTX_PROTO_TRAMP, now_ms);
  else if (known == VTX_PROTO_SMARTAUDIO && pollSmartAudioRx()) vtxMarkOk(VTX_PROTO_SMARTAUDIO, now_ms);

  if ((now_ms - vtx_last_tx_ms) < VTX_CMD_INTERVAL_MS) return;

  if (vtx_send_freq_req || vtx_send_power_req) {
    uint8_t p = (known != VTX_PROTO_NONE) ? known : (uint8_t)VTX_PROTO_SMARTAUDIO;
    if (vtx_send_freq_req && vtxSendFreqCmd(p)) { vtx_send_freq_req = false; vtxArmCmdAck(now_ms); return; }
    if (vtx_send_power_req && vtxSendPowerCmd(p)) { vtx_send_power_req = false; vtxArmCmdAck(now_ms); return; }
  }

  if (!vtx_initial_probe_done || (now_ms - vtx_last_probe_ms) >= VTX_PROBE_INTERVAL_MS) {
    if (known == VTX_PROTO_TRAMP) {
      if (sendTrampFrame('v', 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_TRAMP; }
    } else {
      if (sendSmartAudioFrame(0x03, nullptr, 0)) { vtx_last_tx_ms = vtx_step_start_ms = now_ms; vtx_probe_step = VTX_STEP_WAIT_SA; }
    }
  }
}

namespace SensorDriver {
  enum IMUType : uint8_t { IMU_UNKNOWN = 0, IMU_MPU6050 = 1, IMU_MPU6500 = 2, IMU_MPU9250 = 3, IMU_ICM20602 = 4, IMU_BMI160 = 5, IMU_QMI8658 = 6 };
  enum BaroType : uint8_t { BARO_UNKNOWN = 0, BARO_BMP280 = 1, BARO_BME280 = 2, BARO_MS5611 = 3, BARO_BMP180 = 4, BARO_BMP388 = 5, BARO_BMP390 = 6, BARO_DPS310 = 7 };
  enum MagType : uint8_t { MAG_UNKNOWN = 0, MAG_HMC5883L = 1, MAG_QMC5883L = 2, MAG_AK8963 = 3, MAG_IST8310 = 4, MAG_HSCDTD008A = 5, MAG_AK09911 = 6 };
  enum LidarType : uint8_t { LIDAR_NONE = 0, LIDAR_VL53L0X = 1, LIDAR_VL53L1X = 2, LIDAR_VL53L3CX = 3, LIDAR_VL53L4CX = 4 };
  enum Ms5611Phase : uint8_t { MS_IDLE = 0, MS_WAIT_D1, MS_WAIT_D2 };
  enum Bmp180Phase : uint8_t { B180_IDLE = 0, B180_WAIT_TEMP, B180_WAIT_PRESS };

  struct BaroCalib {
    uint16_t dig_T1 = 0; int16_t dig_T2 = 0, dig_T3 = 0;
    uint16_t dig_P1 = 0; int16_t dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;
    uint16_t ms_c[7] = {0};
    int16_t ac1 = 0, ac2 = 0, ac3 = 0; uint16_t ac4 = 0, ac5 = 0, ac6 = 0;
    int16_t b1 = 0, b2 = 0, mc = 0, md = 0;
    float b3_t1 = 0.0f, b3_t2 = 0.0f, b3_t3 = 0.0f;
    float b3_p1 = 0.0f, b3_p2 = 0.0f, b3_p3 = 0.0f, b3_p4 = 0.0f, b3_p5 = 0.0f;
    float b3_p6 = 0.0f, b3_p7 = 0.0f, b3_p8 = 0.0f, b3_p9 = 0.0f, b3_p10 = 0.0f, b3_p11 = 0.0f;
    int32_t dps_c00 = 0, dps_c10 = 0;
    int16_t dps_c01 = 0, dps_c11 = 0, dps_c20 = 0, dps_c21 = 0, dps_c30 = 0;
    float dps_kp = 524288.0f, dps_kt = 524288.0f;
    bool valid = false;
  } baroCal;

  struct MagCalib { float asa_x = 1.0f, asa_y = 1.0f, asa_z = 1.0f, scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f; } magCal;

  struct SensorStatus {
    IMUType imu_type = IMU_UNKNOWN; BaroType baro_type = BARO_UNKNOWN;
    MagType mag_type = MAG_UNKNOWN; LidarType lidar_type = LIDAR_NONE;
    uint8_t imu_addr = 0x68, baro_addr = 0x76, mag_addr = 0x0C, lidar_addr = 0x29;
    float gyro_scale = 131.0f, acc_scale = 16384.0f, temp_scale = 340.0f, temp_offset = 25.0f, lidar_max_m = 4.0f;
    bool imu_found = false, baro_found = false, mag_found = false, lidar_found = false;
  } activeSensors;

  Ms5611Phase ms5611_phase = MS_IDLE; uint32_t ms5611_conv_start_ms = 0, ms5611_d1_raw = 0, ms5611_d2_raw = 0;
  Bmp180Phase bmp180_phase = B180_IDLE; uint32_t bmp180_conv_start_ms = 0; int32_t bmp180_ut_raw = 0;

  void detectIMU(); void detectBaro(); void detectMag(); void detectLidar(); void initLidarHardware();
  bool calibrateGyro(bool silent = false); bool calibrateAcc(bool silent = false);
  void calibrateIMU(); void calibrateBaro(); void calibrateLidar();
  void calibrateBatVoltage(float actual_voltage); void calibrateCurrent(bool has_value, float actual_current_a);

  float getEffectiveLidarMaxM() { return activeSensors.lidar_max_m * 0.90f; }
  inline float getBatScaleV() { return constrainF(currentConfig.bat_scale_v, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX); }
  inline float getBatScaleA() { return constrainF(currentConfig.bat_scale_a, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX); }

  inline float computeCurrentAmpsFromAdc(uint16_t raw_adc) {
    float pin_v = ((float)raw_adc / 4095.0f) * 3.3f;
    float off_a = constrainF(currentConfig.bat_offset_a, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
    float a = ((pin_v / SHUNT_RESISTANCE_OHM) - off_a) * getBatScaleA();
    return (a > 0.0f) ? a : 0.0f;
  }

  inline float wrapDeg0To360(float deg) {
    if (!isfinite(deg)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return 0.0f; }
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return (w >= 360.0f || w < 0.0f) ? 0.0f : w;
  }

  inline void rotateXYByYawAlign(float &x, float &y, uint8_t align) {
    float ox = x, oy = y;
    switch (align & 0x03) {
      case 1: x = oy; y = -ox; break;
      case 2: x = -ox; y = -oy; break;
      case 3: x = -oy; y = ox; break;
      default: break;
    }
  }

  inline float extractTempFromImuBuf(const uint8_t *buf) {
    return ((float)((int16_t)(buf[6] << 8 | buf[7])) / activeSensors.temp_scale) + activeSensors.temp_offset;
  }

  inline void packNormalizedImuToBuffer(uint8_t *out14, float ax_g, float ay_g, float az_g, float temp_c, float gx_dps, float gy_dps, float gz_dps) {
    int16_t iax = (int16_t)constrainF(ax_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iay = (int16_t)constrainF(ay_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t iaz = (int16_t)constrainF(az_g * activeSensors.acc_scale, -32767.0f, 32767.0f);
    int16_t itm = (int16_t)constrainF((temp_c - activeSensors.temp_offset) * activeSensors.temp_scale, -32767.0f, 32767.0f);
    int16_t igx = (int16_t)constrainF(gx_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igy = (int16_t)constrainF(gy_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    int16_t igz = (int16_t)constrainF(gz_dps * activeSensors.gyro_scale, -32767.0f, 32767.0f);
    out14[0] = (uint8_t)((uint16_t)iax >> 8); out14[1] = (uint8_t)((uint16_t)iax & 0xFF);
    out14[2] = (uint8_t)((uint16_t)iay >> 8); out14[3] = (uint8_t)((uint16_t)iay & 0xFF);
    out14[4] = (uint8_t)((uint16_t)iaz >> 8); out14[5] = (uint8_t)((uint16_t)iaz & 0xFF);
    out14[6] = (uint8_t)((uint16_t)itm >> 8); out14[7] = (uint8_t)((uint16_t)itm & 0xFF);
    out14[8] = (uint8_t)((uint16_t)igx >> 8); out14[9] = (uint8_t)((uint16_t)igx & 0xFF);
    out14[10] = (uint8_t)((uint16_t)igy >> 8); out14[11] = (uint8_t)((uint16_t)igy & 0xFF);
    out14[12] = (uint8_t)((uint16_t)igz >> 8); out14[13] = (uint8_t)((uint16_t)igz & 0xFF);
  }

  void checkBusAndRecover() {
    if (i2c_error_counter > I2C_MAX_ERRORS) {
      setFcErrorRam(FC_ERR_I2C_BUS);
      initI2c1();
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; i2c_error_counter = 0;
    }
  }

  static bool recordI2cError() { i2c_error_counter++; setFcErrorRam(FC_ERR_I2C_BUS); return false; }

  bool readReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    if (i2c1WriteRead(addr, &reg, 1, buf, len)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool readReg16Addr(uint8_t addr, uint16_t reg16, uint8_t *buf, uint8_t len) {
    checkBusAndRecover();
    uint8_t r[2] = {(uint8_t)(reg16 >> 8), (uint8_t)(reg16 & 0xFF)};
    if (i2c1WriteRead(addr, r, 2, buf, len)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    checkBusAndRecover();
    uint8_t t[2] = {reg, val};
    if (i2c1WriteRead(addr, t, 2, nullptr, 0)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  bool writeReg16Addr8Val(uint8_t addr, uint16_t reg16, uint8_t val) {
    checkBusAndRecover();
    uint8_t t[3] = {(uint8_t)(reg16 >> 8), (uint8_t)(reg16 & 0xFF), val};
    if (i2c1WriteRead(addr, t, 3, nullptr, 0)) { i2c_error_counter = 0; return true; }
    return recordI2cError();
  }

  void enableImuI2cBypass() {
    if (activeSensors.imu_found && (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500)) {
      writeReg(activeSensors.imu_addr, 0x6A, 0x00); delay(2);
      writeReg(activeSensors.imu_addr, 0x37, 0x02); delay(5);
    }
  }

  static void initSpecificImu(uint8_t addr, IMUType type) {
    activeSensors.imu_addr = addr; activeSensors.imu_type = type;
    if (type == IMU_BMI160) {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 512.0f; activeSensors.temp_offset = 23.0f;
      writeReg(addr, 0x7E, 0x11); delay(5); writeReg(addr, 0x7E, 0x15); delay(10);
      writeReg(addr, 0x40, 0x16); writeReg(addr, 0x41, 0x03); writeReg(addr, 0x42, 0x17); writeReg(addr, 0x43, 0x03);
    } else if (type == IMU_QMI8658) {
      activeSensors.acc_scale = 8192.0f; activeSensors.gyro_scale = 128.0f; activeSensors.temp_scale = 256.0f; activeSensors.temp_offset = 0.0f;
      writeReg(addr, 0x02, 0x60); writeReg(addr, 0x03, 0x15); writeReg(addr, 0x04, 0x45); writeReg(addr, 0x05, 0x11); writeReg(addr, 0x07, 0x03);
    } else if (type == IMU_ICM20602) {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 326.8f; activeSensors.temp_offset = 25.0f;
      writeReg(addr, 0x6B, 0x01); delay(10); writeReg(addr, 0x1A, 0x03); writeReg(addr, 0x1B, 0x00); writeReg(addr, 0x1C, 0x00); writeReg(addr, 0x1D, 0x03);
    } else {
      activeSensors.acc_scale = 16384.0f; activeSensors.gyro_scale = 131.0f; activeSensors.temp_scale = 340.0f; activeSensors.temp_offset = 25.0f;
      writeReg(addr, 0x6B, 0x00); delay(10); writeReg(addr, 0x1A, 0x03); writeReg(addr, 0x1B, 0x00); writeReg(addr, 0x1C, 0x00);
      if (type == IMU_MPU6500 || type == IMU_MPU9250) writeReg(addr, 0x1D, 0x03);
    }
    activeSensors.imu_found = true; err_imu_fail = false;
    if ((type == IMU_MPU9250 || type == IMU_MPU6500) && currentConfig.com_en) enableImuI2cBypass();
  }

  static bool probeImuOnce() {
    uint8_t id = 0, req = currentConfig.imu_id;
    const uint8_t addrs689[2] = {0x68, 0x69};
    const uint8_t addrsQmi[4] = {0x6A, 0x6B, 0x68, 0x69};

    if (req == IMU_BMI160 || req == IMU_UNKNOWN) {
      for (uint8_t i = 0; i < 2; i++) if (readReg(addrs689[i], 0x00, &id, 1) && id == 0xD1) { initSpecificImu(addrs689[i], IMU_BMI160); return true; }
      if (req == IMU_BMI160) return false;
    }
    if (req == IMU_QMI8658 || req == IMU_UNKNOWN) {
      for (uint8_t i = 0; i < 4; i++) if (readReg(addrsQmi[i], 0x00, &id, 1) && id == 0x05) { initSpecificImu(addrsQmi[i], IMU_QMI8658); return true; }
      if (req == IMU_QMI8658) return false;
    }
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t a = addrs689[i];
      if (!readReg(a, 0x75, &id, 1)) continue;
      if ((req == IMU_MPU6050 || req == IMU_UNKNOWN) && id == 0x68) { initSpecificImu(a, IMU_MPU6050); return true; }
      if ((req == IMU_MPU6500 || req == IMU_MPU9250 || req == IMU_UNKNOWN) && id == 0x70) {
        if (req == IMU_MPU6500) { initSpecificImu(a, IMU_MPU6500); return true; }
        uint8_t ak = 0; writeReg(a, 0x6A, 0x00); delay(2); writeReg(a, 0x37, 0x02); delay(5);
        initSpecificImu(a, (readReg(0x0C, 0x00, &ak, 1) && ak == 0x48) ? IMU_MPU9250 : IMU_MPU6500);
        return true;
      }
      if ((req == IMU_MPU9250 || req == IMU_UNKNOWN) && (id == 0x71 || id == 0x73)) { initSpecificImu(a, IMU_MPU9250); return true; }
      if ((req == IMU_ICM20602 || req == IMU_UNKNOWN) && id == 0x12) { initSpecificImu(a, IMU_ICM20602); return true; }
    }
    return false;
  }

  void detectIMU() {
    activeSensors.imu_found = false; activeSensors.imu_type = IMU_UNKNOWN; activeSensors.temp_offset = 25.0f;
    if (probeImuOnce()) return;
    err_imu_fail = true; setFcErrorRam(FC_ERR_IMU_FAIL);
  }

  bool readMPU6050(uint8_t *buffer) {
    if (!activeSensors.imu_found) return false;
    if (activeSensors.imu_type == IMU_BMI160) {
      uint8_t raw[14]; if (!readReg(activeSensors.imu_addr, 0x0C, raw, 14)) return false;
      packNormalizedImuToBuffer(buffer,
        (float)((int16_t)(raw[7] << 8 | raw[6])) / activeSensors.acc_scale,
        (float)((int16_t)(raw[9] << 8 | raw[8])) / activeSensors.acc_scale,
        (float)((int16_t)(raw[11] << 8 | raw[10])) / activeSensors.acc_scale,
        ((float)((int16_t)(raw[13] << 8 | raw[12])) / activeSensors.temp_scale) + activeSensors.temp_offset,
        (float)((int16_t)(raw[1] << 8 | raw[0])) / activeSensors.gyro_scale,
        (float)((int16_t)(raw[3] << 8 | raw[2])) / activeSensors.gyro_scale,
        (float)((int16_t)(raw[5] << 8 | raw[4])) / activeSensors.gyro_scale);
      return true;
    }
    if (activeSensors.imu_type == IMU_QMI8658) {
      uint8_t tb[2], xb[12];
      if (!readReg(activeSensors.imu_addr, 0x33, tb, 2) || !readReg(activeSensors.imu_addr, 0x35, xb, 12)) return false;
      packNormalizedImuToBuffer(buffer,
        (float)((int16_t)(xb[1] << 8 | xb[0])) / activeSensors.acc_scale,
        (float)((int16_t)(xb[3] << 8 | xb[2])) / activeSensors.acc_scale,
        (float)((int16_t)(xb[5] << 8 | xb[4])) / activeSensors.acc_scale,
        ((float)((int16_t)(tb[1] << 8 | tb[0])) / activeSensors.temp_scale) + activeSensors.temp_offset,
        (float)((int16_t)(xb[7] << 8 | xb[6])) / activeSensors.gyro_scale,
        (float)((int16_t)(xb[9] << 8 | xb[8])) / activeSensors.gyro_scale,
        (float)((int16_t)(xb[11] << 8 | xb[10])) / activeSensors.gyro_scale);
      return true;
    }
    return readReg(activeSensors.imu_addr, 0x3B, buffer, 14);
  }

  inline float pressureToAltitudeMeters(float p_pa) { return 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1903f)); }
  inline bool isPressureValidPa(float p_pa) { return isfinite(p_pa) && p_pa >= 30000.0f && p_pa <= 125000.0f; }
  static void markBaroError() { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_BARO_FAIL); }

  bool readBme280PressurePa(float &p_pa) {
    uint8_t raw[6]; if (!readReg(activeSensors.baro_addr, 0xF7, raw, 6)) return false;
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t v1t = ((((adc_T >> 3) - ((int32_t)baroCal.dig_T1 << 1))) * (int32_t)baroCal.dig_T2) >> 11;
    int32_t v2t = (((((adc_T >> 4) - (int32_t)baroCal.dig_T1) * ((adc_T >> 4) - (int32_t)baroCal.dig_T1)) >> 12) * (int32_t)baroCal.dig_T3) >> 14;
    int64_t v1p = ((int64_t)(v1t + v2t)) - 128000;
    int64_t v2p = v1p * v1p * (int64_t)baroCal.dig_P6 + ((v1p * (int64_t)baroCal.dig_P5) << 17) + (((int64_t)baroCal.dig_P4) << 35);
    v1p = ((v1p * v1p * (int64_t)baroCal.dig_P3) >> 8) + ((v1p * (int64_t)baroCal.dig_P2) << 12);
    v1p = (((((int64_t)1) << 47) + v1p)) * (int64_t)baroCal.dig_P1 >> 33;
    if (v1p == 0) return false;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2p) * 3125) / v1p;
    v1p = (((int64_t)baroCal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2p = (((int64_t)baroCal.dig_P8) * p) >> 19;
    p = ((p + v1p + v2p) >> 8) + (((int64_t)baroCal.dig_P7) << 4);
    p_pa = (float)p / 256.0f;
    return isPressureValidPa(p_pa);
  }

  bool computeMs5611AltitudeFromRaw(uint32_t d1, uint32_t d2, float &alt_m) {
    if (d1 == 0 || d2 == 0) return false;
    int32_t dT = (int32_t)d2 - ((int32_t)baroCal.ms_c[5] << 8);
    int64_t temp = 2000LL + (((int64_t)dT * baroCal.ms_c[6]) >> 23);
    int64_t off = ((int64_t)baroCal.ms_c[2] << 16) + (((int64_t)baroCal.ms_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)baroCal.ms_c[1] << 15) + (((int64_t)baroCal.ms_c[3] * dT) >> 8);
    if (temp < 2000) {
      int64_t t2 = ((int64_t)dT * dT) >> 31, dt = temp - 2000LL;
      int64_t o2 = (5LL * dt * dt) >> 1, s2 = (5LL * dt * dt) >> 2;
      if (temp < -1500) { int64_t d15 = temp + 1500LL; o2 += 7LL * d15 * d15; s2 += (11LL * d15 * d15) >> 1; }
      temp -= t2; off -= o2; sens -= s2;
    }
    float p_pa = (float)(((((int64_t)d1 * sens) >> 21) - off) >> 15) * 10.0f;
    if (!isPressureValidPa(p_pa)) { markBaroError(); return false; }
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  float computeBmp180PressurePa(int32_t ut, int32_t up, uint8_t oss = 1) {
    int32_t x1 = ((ut - (int32_t)baroCal.ac6) * (int32_t)baroCal.ac5) >> 15;
    int32_t den = x1 + (int32_t)baroCal.md; if (den == 0) den = 1;
    int32_t x2 = ((int32_t)baroCal.mc << 11) / den;
    int32_t b5 = x1 + x2, b6 = b5 - 4000;
    x1 = ((int32_t)baroCal.b2 * ((b6 * b6) >> 12)) >> 11; x2 = ((int32_t)baroCal.ac2 * b6) >> 11;
    int32_t b3 = (((((int32_t)baroCal.ac1) * 4 + (x1 + x2)) << oss) + 2) >> 2;
    x1 = ((int32_t)baroCal.ac3 * b6) >> 13; x2 = ((int32_t)baroCal.b1 * ((b6 * b6) >> 12)) >> 16;
    uint32_t b4 = ((uint32_t)baroCal.ac4 * (uint32_t)((((x1 + x2) + 2) >> 2) + 32768)) >> 15; if (b4 == 0) b4 = 1;
    uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (50000UL >> oss);
    int32_t p = (b7 < 0x80000000UL) ? (int32_t)((b7 * 2UL) / b4) : (int32_t)((b7 / b4) * 2UL);
    x1 = (p >> 8) * (p >> 8); x1 = (x1 * 3038) >> 16; x2 = (-7357 * p) >> 16;
    return (float)(p + ((x1 + x2 + 3791) >> 4));
  }

  bool stepBmp180Phased(uint32_t now_ms, float &alt_m) {
    constexpr uint8_t oss = 1;
    if (bmp180_phase == B180_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) { markBaroError(); return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_TEMP; return false;
    }
    if (bmp180_phase == B180_WAIT_TEMP) {
      if ((now_ms - bmp180_conv_start_ms) < BMP180_TEMP_DELAY_MS) return false;
      uint8_t tb[2]; if (!readReg(activeSensors.baro_addr, 0xF6, tb, 2)) { bmp180_phase = B180_IDLE; return false; }
      bmp180_ut_raw = (int32_t)((uint16_t)tb[0] << 8 | tb[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) { bmp180_phase = B180_IDLE; return false; }
      bmp180_conv_start_ms = now_ms; bmp180_phase = B180_WAIT_PRESS; return false;
    }
    if ((now_ms - bmp180_conv_start_ms) < BMP180_PRESS_DELAY_MS) return false;
    uint8_t pb[3]; if (!readReg(activeSensors.baro_addr, 0xF6, pb, 3)) { bmp180_phase = B180_IDLE; return false; }
    int32_t up = (int32_t)(((((uint32_t)pb[0] << 16) | ((uint32_t)pb[1] << 8) | pb[2])) >> (8 - oss));
    bmp180_phase = B180_IDLE;
    float p_pa = computeBmp180PressurePa(bmp180_ut_raw, up, oss);
    if (!isPressureValidPa(p_pa)) { markBaroError(); return false; }
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  bool readBmp388PressurePa(float &p_pa) {
    uint8_t d[6]; if (!readReg(activeSensors.baro_addr, 0x04, d, 6)) return false;
    uint32_t up = ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0];
    uint32_t ut = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | d[3];
    float pd1 = (float)ut - baroCal.b3_t1;
    float tl = pd1 * baroCal.b3_t2 + pd1 * pd1 * baroCal.b3_t3;
    float tl2 = tl * tl, tl3 = tl2 * tl, fp = (float)up, fp2 = fp * fp, fp3 = fp2 * fp;
    float o1 = baroCal.b3_p5 + baroCal.b3_p6 * tl + baroCal.b3_p7 * tl2 + baroCal.b3_p8 * tl3;
    float o2 = fp * (baroCal.b3_p1 + baroCal.b3_p2 * tl + baroCal.b3_p3 * tl2 + baroCal.b3_p4 * tl3);
    p_pa = o1 + o2 + fp2 * (baroCal.b3_p9 + baroCal.b3_p10 * tl) + fp3 * baroCal.b3_p11;
    return isPressureValidPa(p_pa);
  }

  bool readDps310PressurePa(float &p_pa) {
    uint8_t d[6]; if (!readReg(activeSensors.baro_addr, 0x00, d, 6)) return false;
    int32_t pr = (int32_t)(((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]); if (pr & 0x00800000) pr |= 0xFF000000;
    int32_t tr = (int32_t)(((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5]); if (tr & 0x00800000) tr |= 0xFF000000;
    float ts = (float)tr / baroCal.dps_kt, ps = (float)pr / baroCal.dps_kp;
    p_pa = (float)baroCal.dps_c00 + ps * ((float)baroCal.dps_c10 + ps * ((float)baroCal.dps_c20 + ps * (float)baroCal.dps_c30)) +
           ts * ((float)baroCal.dps_c01 + ps * ((float)baroCal.dps_c11 + ps * (float)baroCal.dps_c21));
    return isPressureValidPa(p_pa);
  }

  bool stepMs5611Phased(uint32_t now_ms, float &alt_m) {
    if (ms5611_phase == MS_IDLE) {
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) { markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D1; return false;
    }
    if (ms5611_phase == MS_WAIT_D1) {
      if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
      uint8_t ab[3]; if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) { ms5611_phase = MS_IDLE; return false; }
      ms5611_d1_raw = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) { ms5611_phase = MS_IDLE; markBaroError(); return false; }
      ms5611_conv_start_ms = now_ms; ms5611_phase = MS_WAIT_D2; return false;
    }
    if ((now_ms - ms5611_conv_start_ms) < MS5611_CONV_DELAY_MS) return false;
    uint8_t ab[3]; if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) { ms5611_phase = MS_IDLE; return false; }
    ms5611_d2_raw = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
    ms5611_phase = MS_IDLE;
    return computeMs5611AltitudeFromRaw(ms5611_d1_raw, ms5611_d2_raw, alt_m);
  }

  bool readRawBaroAltitudeBlocking(float &alt_m) {
    if (!activeSensors.baro_found || !baroCal.valid) return false;
    if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t ab[3];
      if (!writeReg(activeSensors.baro_addr, 0x48, 0x00)) return false;
      delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) return false;
      uint32_t d1 = ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2];
      if (!writeReg(activeSensors.baro_addr, 0x58, 0x00)) return false;
      delay(MS5611_CONV_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0x00, ab, 3)) return false;
      return computeMs5611AltitudeFromRaw(d1, ((uint32_t)ab[0] << 16) | ((uint32_t)ab[1] << 8) | ab[2], alt_m);
    }
    if (activeSensors.baro_type == BARO_BMP180) {
      constexpr uint8_t oss = 1; uint8_t tb[2], pb[3];
      if (!writeReg(activeSensors.baro_addr, 0xF4, 0x2E)) return false;
      delay(BMP180_TEMP_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, tb, 2)) return false;
      int32_t ut = (int32_t)((uint16_t)tb[0] << 8 | tb[1]);
      if (!writeReg(activeSensors.baro_addr, 0xF4, (uint8_t)(0x34 + (oss << 6)))) return false;
      delay(BMP180_PRESS_DELAY_MS);
      if (!readReg(activeSensors.baro_addr, 0xF6, pb, 3)) return false;
      float p_pa = computeBmp180PressurePa(ut, (int32_t)(((((uint32_t)pb[0] << 16) | ((uint32_t)pb[1] << 8) | pb[2])) >> (8 - oss)), oss);
      if (!isPressureValidPa(p_pa)) return false;
      alt_m = pressureToAltitudeMeters(p_pa); return true;
    }
    float p_pa = 0.0f;
    bool ok = (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) ? readBmp388PressurePa(p_pa) :
              (activeSensors.baro_type == BARO_DPS310) ? readDps310PressurePa(p_pa) : readBme280PressurePa(p_pa);
    if (!ok) return false;
    alt_m = pressureToAltitudeMeters(p_pa); return true;
  }

  void initBaroHardware() {
    if (!activeSensors.baro_found) return;
    baroCal.valid = false; ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE;
    if (activeSensors.baro_type == BARO_BMP280 || activeSensors.baro_type == BARO_BME280) {
      uint8_t c[24];
      if (readReg(activeSensors.baro_addr, 0x88, c, 24)) {
        baroCal.dig_T1 = (uint16_t)(c[1] << 8 | c[0]); baroCal.dig_T2 = (int16_t)(c[3] << 8 | c[2]); baroCal.dig_T3 = (int16_t)(c[5] << 8 | c[4]);
        baroCal.dig_P1 = (uint16_t)(c[7] << 8 | c[6]); baroCal.dig_P2 = (int16_t)(c[9] << 8 | c[8]); baroCal.dig_P3 = (int16_t)(c[11] << 8 | c[10]);
        baroCal.dig_P4 = (int16_t)(c[13] << 8 | c[12]); baroCal.dig_P5 = (int16_t)(c[15] << 8 | c[14]); baroCal.dig_P6 = (int16_t)(c[17] << 8 | c[16]);
        baroCal.dig_P7 = (int16_t)(c[19] << 8 | c[18]); baroCal.dig_P8 = (int16_t)(c[21] << 8 | c[20]); baroCal.dig_P9 = (int16_t)(c[23] << 8 | c[22]);
        baroCal.valid = (baroCal.dig_T1 != 0 && baroCal.dig_P1 != 0);
      }
      writeReg(activeSensors.baro_addr, 0xF4, 0x2F);
    } else if (activeSensors.baro_type == BARO_BMP180) {
      uint8_t c[22];
      if (readReg(activeSensors.baro_addr, 0xAA, c, 22)) {
        baroCal.ac1 = (int16_t)(c[0] << 8 | c[1]); baroCal.ac2 = (int16_t)(c[2] << 8 | c[3]); baroCal.ac3 = (int16_t)(c[4] << 8 | c[5]);
        baroCal.ac4 = (uint16_t)(c[6] << 8 | c[7]); baroCal.ac5 = (uint16_t)(c[8] << 8 | c[9]); baroCal.ac6 = (uint16_t)(c[10] << 8 | c[11]);
        baroCal.b1 = (int16_t)(c[12] << 8 | c[13]); baroCal.b2 = (int16_t)(c[14] << 8 | c[15]);
        baroCal.mc = (int16_t)(c[18] << 8 | c[19]); baroCal.md = (int16_t)(c[20] << 8 | c[21]);
        baroCal.valid = (baroCal.ac4 != 0 && baroCal.ac5 != 0 && baroCal.ac6 != 0);
      }
    } else if (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) {
      uint8_t n[21];
      if (readReg(activeSensors.baro_addr, 0x31, n, 21)) {
        uint16_t ut1 = (uint16_t)(n[1] << 8 | n[0]), ut2 = (uint16_t)(n[3] << 8 | n[2]);
        int8_t st3 = (int8_t)n[4]; int16_t sp1 = (int16_t)(n[6] << 8 | n[5]), sp2 = (int16_t)(n[8] << 8 | n[7]);
        int8_t sp3 = (int8_t)n[9], sp4 = (int8_t)n[10]; uint16_t up5 = (uint16_t)(n[12] << 8 | n[11]), up6 = (uint16_t)(n[14] << 8 | n[13]);
        int8_t sp7 = (int8_t)n[15], sp8 = (int8_t)n[16]; int16_t sp9 = (int16_t)(n[18] << 8 | n[17]);
        int8_t sp10 = (int8_t)n[19], sp11 = (int8_t)n[20];
        baroCal.b3_t1 = (float)ut1 * 256.0f; baroCal.b3_t2 = (float)ut2 / 1073741824.0f; baroCal.b3_t3 = (float)st3 / 281474976710656.0f;
        baroCal.b3_p1 = ((float)sp1 - 16384.0f) / 1048576.0f; baroCal.b3_p2 = ((float)sp2 - 16384.0f) / 536870912.0f;
        baroCal.b3_p3 = (float)sp3 / 4294967296.0f; baroCal.b3_p4 = (float)sp4 / 137438953472.0f;
        baroCal.b3_p5 = (float)up5 * 8.0f; baroCal.b3_p6 = (float)up6 / 64.0f;
        baroCal.b3_p7 = (float)sp7 / 256.0f; baroCal.b3_p8 = (float)sp8 / 32768.0f;
        baroCal.b3_p9 = (float)sp9 / 281474976710656.0f; baroCal.b3_p10 = (float)sp10 / 281474976710656.0f;
        baroCal.b3_p11 = (float)sp11 / (281474976710656.0f * 131072.0f);
        baroCal.valid = (ut1 != 0 && ut2 != 0);
      }
      writeReg(activeSensors.baro_addr, 0x1C, 0x03); writeReg(activeSensors.baro_addr, 0x1F, 0x04); writeReg(activeSensors.baro_addr, 0x1B, 0x33);
    } else if (activeSensors.baro_type == BARO_DPS310) {
      uint8_t c[18];
      if (readReg(activeSensors.baro_addr, 0x10, c, 18)) {
        int32_t c00 = (int32_t)(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4)); if (c00 & 0x00080000) c00 |= 0xFFF00000;
        int32_t c10 = (int32_t)(((uint32_t)(c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7]); if (c10 & 0x00080000) c10 |= 0xFFF00000;
        baroCal.dps_c00 = c00; baroCal.dps_c10 = c10;
        baroCal.dps_c01 = (int16_t)((uint16_t)c[8] << 8 | c[9]); baroCal.dps_c11 = (int16_t)((uint16_t)c[10] << 8 | c[11]);
        baroCal.dps_c20 = (int16_t)((uint16_t)c[12] << 8 | c[13]); baroCal.dps_c21 = (int16_t)((uint16_t)c[14] << 8 | c[15]); baroCal.dps_c30 = (int16_t)((uint16_t)c[16] << 8 | c[17]);
        baroCal.valid = (c00 != 0 || c10 != 0);
      }
      writeReg(activeSensors.baro_addr, 0x06, 0x54); writeReg(activeSensors.baro_addr, 0x07, 0x84); writeReg(activeSensors.baro_addr, 0x09, 0x0C);
      baroCal.dps_kp = baroCal.dps_kt = 253952.0f;
      writeReg(activeSensors.baro_addr, 0x08, 0x07);
    } else if (activeSensors.baro_type == BARO_MS5611) {
      uint8_t prom[2]; bool ok = true;
      for (uint8_t i = 1; i <= 6; i++) {
        if (readReg(activeSensors.baro_addr, (uint8_t)(0xA0 + (i * 2)), prom, 2)) { baroCal.ms_c[i] = ((uint16_t)prom[0] << 8) | prom[1]; if (baroCal.ms_c[i] == 0) ok = false; }
        else ok = false;
      }
      baroCal.valid = ok;
    }
    if (!baroCal.valid && currentConfig.baro_en) markBaroError();
  }

  static void bindAndInitBaro(uint8_t addr, BaroType t) { activeSensors.baro_addr = addr; activeSensors.baro_type = t; activeSensors.baro_found = true; initBaroHardware(); }

  void detectBaro() {
    uint8_t id = 0, req = currentConfig.bar_id;
    activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; baroCal.valid = false;
    const uint8_t addrs[2] = {0x76, 0x77};
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t a = addrs[i];
      if ((req == BARO_DPS310 || req == BARO_UNKNOWN) && readReg(a, 0x0D, &id, 1) && id == 0x10) { bindAndInitBaro(a, BARO_DPS310); return; }
      if (readReg(a, 0xD0, &id, 1)) {
        if ((req == BARO_BMP180 || req == BARO_UNKNOWN) && id == 0x55) { bindAndInitBaro(a, BARO_BMP180); return; }
        if ((req == BARO_BMP388 || req == BARO_UNKNOWN) && id == 0x50) { bindAndInitBaro(a, BARO_BMP388); return; }
        if ((req == BARO_BMP280 || req == BARO_UNKNOWN) && id == 0x58) { bindAndInitBaro(a, BARO_BMP280); return; }
        if (id == 0x60) {
          uint8_t cm = 0; bool has_cm = readReg(a, 0xF4, &cm, 1);
          BaroType det = (has_cm && cm != 0x00) ? BARO_BME280 : BARO_BMP390;
          if (req == BARO_UNKNOWN || req == (uint8_t)det) { bindAndInitBaro(a, det); return; }
        }
      }
      if (req == BARO_MS5611 || req == BARO_UNKNOWN) {
        uint8_t prom[2]; if (readReg(a, 0xA2, prom, 2) && (prom[0] != 0 || prom[1] != 0)) { bindAndInitBaro(a, BARO_MS5611); return; }
      }
    }
    if (currentConfig.baro_en) markBaroError();
  }

  bool stepBaroNonBlocking(uint32_t now_ms, float &alt_m) {
    if (!currentConfig.baro_en || !activeSensors.baro_found || !baroCal.valid) return false;
    float raw_alt = 0.0f;
    if (activeSensors.baro_type == BARO_MS5611) { if (!stepMs5611Phased(now_ms, raw_alt)) return false; }
    else if (activeSensors.baro_type == BARO_BMP180) { if (!stepBmp180Phased(now_ms, raw_alt)) return false; }
    else {
      float p_pa = 0.0f;
      bool ok = (activeSensors.baro_type == BARO_BMP388 || activeSensors.baro_type == BARO_BMP390) ? readBmp388PressurePa(p_pa) :
                (activeSensors.baro_type == BARO_DPS310) ? readDps310PressurePa(p_pa) : readBme280PressurePa(p_pa);
      if (!ok) { markBaroError(); return false; }
      raw_alt = pressureToAltitudeMeters(p_pa);
    }
    if (!isfinite(raw_alt)) return false;
    if (!baro_cal_ready) { baro_ground_alt_m = raw_alt; lpf_baro_alt.output = 0.0f; baro_cal_ready = true; }
    alt_m = lpf_baro_alt.update(raw_alt - baro_ground_alt_m);
    return true;
  }

  void calibrateBaro() {
    if (!activeSensors.baro_found || !baroCal.valid) { usb_cdc.println("ERR: baro"); return; }
    float sum = 0.0f; uint8_t ok_c = 0;
    for (uint8_t i = 0; i < 12; i++) { float ra = 0.0f; if (readRawBaroAltitudeBlocking(ra)) { sum += ra; ok_c++; } }
    if (ok_c > 0) {
      baro_ground_alt_m = sum / (float)ok_c;
      lpf_baro_alt.output = lpf_baro_vz.output = baro_alt_m = baro_vz_mps = 0.0f;
      ms5611_phase = MS_IDLE; bmp180_phase = B180_IDLE; baro_cal_ready = true;
      usb_cdc.println("OK: cal_bar");
    } else usb_cdc.println("ERR: cal_bar");
  }

  void initMagHardware() {
    if (!activeSensors.mag_found) return;
    magCal.asa_x = magCal.asa_y = magCal.asa_z = 1.0f;
    if (activeSensors.mag_type == MAG_AK8963) {
      writeReg(activeSensors.mag_addr, 0x0A, 0x00); delay(5); writeReg(activeSensors.mag_addr, 0x0A, 0x1F); delay(5);
      uint8_t asa[3] = {128, 128, 128};
      if (readReg(activeSensors.mag_addr, 0x10, asa, 3)) {
        magCal.asa_x = (((float)asa[0] - 128.0f) / 256.0f) + 1.0f;
        magCal.asa_y = (((float)asa[1] - 128.0f) / 256.0f) + 1.0f;
        magCal.asa_z = (((float)asa[2] - 128.0f) / 256.0f) + 1.0f;
      }
      writeReg(activeSensors.mag_addr, 0x0A, 0x00); delay(5); writeReg(activeSensors.mag_addr, 0x0A, 0x16); delay(5);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.15f;
    } else if (activeSensors.mag_type == MAG_AK09911) {
      writeReg(activeSensors.mag_addr, 0x31, 0x00); delay(3); writeReg(activeSensors.mag_addr, 0x31, 0x04); delay(3);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.6f;
    } else if (activeSensors.mag_type == MAG_HSCDTD008A) {
      writeReg(activeSensors.mag_addr, 0x1E, 0x10); delay(2); writeReg(activeSensors.mag_addr, 0x1B, 0x80); delay(3);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 0.15f;
    } else if (activeSensors.mag_type == MAG_QMC5883L) {
      writeReg(activeSensors.mag_addr, 0x0B, 0x01); delay(2); writeReg(activeSensors.mag_addr, 0x09, 0x0D); delay(2);
      magCal.scale_x = magCal.scale_y = magCal.scale_z = 100.0f / 12000.0f;
    } else if (activeSensors.mag_type == MAG_HMC5883L) {
      writeReg(activeSensors.mag_addr, 0x00, 0x78); writeReg(activeSensors.mag_addr, 0x01, 0x20); writeReg(activeSensors.mag_addr, 0x02, 0x00); delay(6);
      magCal.scale_x = magCal.scale_y = 100.0f / 1090.0f; magCal.scale_z = 100.0f / 980.0f;
    }
  }

  static void bindAndInitMag(uint8_t addr, MagType t) { activeSensors.mag_addr = addr; activeSensors.mag_type = t; activeSensors.mag_found = true; initMagHardware(); }

  bool readMagUnified(float &mx_u, float &my_u, float &mz_u) {
    if (!activeSensors.mag_found) return false;
    uint8_t buf[6]; float x = 0.0f, y = 0.0f, z = 0.0f;
    if (activeSensors.mag_type == MAG_AK8963) {
      uint8_t st1 = 0, st2 = 0;
      if (!readReg(activeSensors.mag_addr, 0x02, &st1, 1) || !(st1 & 0x01) || !readReg(activeSensors.mag_addr, 0x03, buf, 6)) return false;
      readReg(activeSensors.mag_addr, 0x09, &st2, 1); if (st2 & 0x08) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.asa_x * magCal.scale_x;
      y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.asa_y * magCal.scale_y;
      z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.asa_z * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_AK09911) {
      uint8_t st1 = 0, meas[8];
      if (!readReg(activeSensors.mag_addr, 0x10, &st1, 1) || !(st1 & 0x01) || !readReg(activeSensors.mag_addr, 0x11, meas, 8) || (meas[7] & 0x08)) return false;
      x = (float)((int16_t)(meas[1] << 8 | meas[0])) * magCal.scale_x; y = (float)((int16_t)(meas[3] << 8 | meas[2])) * magCal.scale_y; z = (float)((int16_t)(meas[5] << 8 | meas[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_HSCDTD008A) {
      if (!readReg(activeSensors.mag_addr, 0x10, buf, 6)) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.scale_x; y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.scale_y; z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_QMC5883L) {
      uint8_t st = 0; if (!readReg(activeSensors.mag_addr, 0x06, &st, 1) || !(st & 0x01) || !readReg(activeSensors.mag_addr, 0x00, buf, 6)) return false;
      x = (float)((int16_t)(buf[1] << 8 | buf[0])) * magCal.scale_x; y = (float)((int16_t)(buf[3] << 8 | buf[2])) * magCal.scale_y; z = (float)((int16_t)(buf[5] << 8 | buf[4])) * magCal.scale_z;
    } else if (activeSensors.mag_type == MAG_HMC5883L) {
      if (!readReg(activeSensors.mag_addr, 0x03, buf, 6)) return false;
      x = (float)((int16_t)(buf[0] << 8 | buf[1])) * magCal.scale_x; y = (float)((int16_t)(buf[4] << 8 | buf[5])) * magCal.scale_y; z = (float)((int16_t)(buf[2] << 8 | buf[3])) * magCal.scale_z;
    } else return false;
    rotateXYByYawAlign(x, y, currentConfig.com_align);
    mx_u = x; my_u = y; mz_u = z; return true;
  }

  bool readCompassHeadingDeg(float &heading_deg) {
    if (!currentConfig.com_en || !activeSensors.mag_found) return false;
    float mx_u, my_u, mz_u;
    if (!readMagUnified(mx_u, my_u, mz_u)) return false;
    float x = mx_u - currentConfig.mag_bias_x, y = my_u - currentConfig.mag_bias_y, z = mz_u - currentConfig.mag_bias_z;
    float xh = x, yh = y;
    if (activeSensors.imu_found && !err_imu_fail) {
      float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
      float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
      float pitch = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
      xh = x * cosf(pitch) + y * sinf(roll) * sinf(pitch) + z * cosf(roll) * sinf(pitch);
      yh = y * cosf(roll) - z * sinf(roll);
    }
    float h = atan2f(yh, xh) * FC_RAD_TO_DEG;
    if (!isfinite(h)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); return false; }
    heading_deg = wrapDeg0To360(h); return true;
  }

  void detectMag() {
    uint8_t id = 0, req = currentConfig.com_id;
    activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN;
    if (activeSensors.imu_found && (activeSensors.imu_type == IMU_MPU9250 || activeSensors.imu_type == IMU_MPU6500)) enableImuI2cBypass();
    if ((req == MAG_AK8963 || req == MAG_UNKNOWN) && readReg(0x0C, 0x00, &id, 1) && id == 0x48) { bindAndInitMag(0x0C, MAG_AK8963); return; }
    if (req == MAG_AK09911 || req == MAG_UNKNOWN) {
      const uint8_t a2[2] = {0x0C, 0x0E};
      for (uint8_t i = 0; i < 2; i++) { uint8_t wia[2] = {0, 0}; if (readReg(a2[i], 0x00, wia, 2) && wia[0] == 0x48 && wia[1] == 0x05) { bindAndInitMag(a2[i], MAG_AK09911); return; } }
    }
    if (req == MAG_HSCDTD008A || req == MAG_UNKNOWN) {
      const uint8_t a2[2] = {0x0C, 0x0F};
      for (uint8_t i = 0; i < 2; i++) if (readReg(a2[i], 0x0F, &id, 1) && id == 0x49) { bindAndInitMag(a2[i], MAG_HSCDTD008A); return; }
    }
    if ((req == MAG_QMC5883L || req == MAG_UNKNOWN) && readReg(0x0D, 0x0D, &id, 1) && id != 0x00) { bindAndInitMag(0x0D, MAG_QMC5883L); return; }
    if (req == MAG_HMC5883L || req == MAG_UNKNOWN) {
      uint8_t hid[3] = {0}; if (readReg(0x1E, 0x0A, hid, 3) && hid[0] == 'H') { bindAndInitMag(0x1E, MAG_HMC5883L); return; }
    }
    if (currentConfig.com_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_MAG_FAIL); }
  }

  void detectLidar() {
    uint8_t req = currentConfig.lid_id;
    activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE;
    checkBusAndRecover();
    if (!i2c1WriteRead(0x29, nullptr, 0, nullptr, 0)) {
      if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); }
      return;
    }
    uint8_t id0 = 0; bool is0x = (readReg(0x29, 0xC0, &id0, 1) && (id0 == 0xEE || id0 == 0xAA));
    uint8_t mid[2] = {0, 0}; bool has_m = readReg16Addr(0x29, 0x010F, mid, 2);
    if (req == LIDAR_VL53L0X && is0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L1X && has_m && mid[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L3CX && has_m && (mid[0] == 0xEB || mid[0] == 0xED) && mid[1] <= 0x40) { activeSensors.lidar_type = LIDAR_VL53L3CX; activeSensors.lidar_max_m = 3.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_VL53L4CX && has_m && (mid[0] == 0xEB || mid[0] == 0xED) && mid[1] > 0x40) { activeSensors.lidar_type = LIDAR_VL53L4CX; activeSensors.lidar_max_m = 6.0f; activeSensors.lidar_found = true; return; }
    if (req == LIDAR_NONE) {
      if (is0x) { activeSensors.lidar_type = LIDAR_VL53L0X; activeSensors.lidar_max_m = 2.0f; activeSensors.lidar_found = true; return; }
      if (has_m && mid[0] == 0xEA) { activeSensors.lidar_type = LIDAR_VL53L1X; activeSensors.lidar_max_m = 4.0f; activeSensors.lidar_found = true; return; }
      if (has_m && (mid[0] == 0xEB || mid[0] == 0xED)) {
        activeSensors.lidar_type = (mid[1] > 0x40) ? LIDAR_VL53L4CX : LIDAR_VL53L3CX;
        activeSensors.lidar_max_m = (activeSensors.lidar_type == LIDAR_VL53L4CX) ? 6.0f : 3.0f; activeSensors.lidar_found = true; return;
      }
    }
    if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); }
  }

  void initLidarHardware() {
    if (!activeSensors.lidar_found || !currentConfig.lidar_en) return;
    checkBusAndRecover();
    if (activeSensors.lidar_type == LIDAR_VL53L0X) { writeReg(0x29, 0x00, 0x01); delay(10); }
    else {
      writeReg16Addr8Val(0x29, 0x0000, 0x00); delay(10);
      writeReg16Addr8Val(0x29, 0x007F, 0x7F); writeReg16Addr8Val(0x29, 0x0080, 0x77);
      writeReg16Addr8Val(0x29, 0x0002, 0x00); writeReg16Addr8Val(0x29, 0x0086, 0x40);
    }
  }

  bool readRawLidarSlantMeters(float &slant_m) {
    if (!activeSensors.lidar_found) return false;
    uint16_t dist_mm = 0;
    if (activeSensors.lidar_type == LIDAR_VL53L0X) {
      uint8_t d[3] = {0};
      if (!readReg(0x29, 0x13, d, 3)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = (d[0] >> 3) & 0x0F; if (st != 0 && st != 11) return false;
      dist_mm = ((uint16_t)d[1] << 8) | d[2];
    } else {
      uint8_t blk[15] = {0};
      if (!readReg16Addr(0x29, 0x0089, blk, 15)) { if (currentConfig.lidar_en) { err_aux_i2c_fail = true; setFcErrorRam(FC_ERR_LIDAR_FAIL); } return false; }
      uint8_t st = blk[0] & 0x1F; if (st != 0 && st != 9) return false;
      dist_mm = ((uint16_t)blk[13] << 8) | blk[14];
    }
    if (dist_mm == 0 || dist_mm > (uint16_t)(getEffectiveLidarMaxM() * 1000.0f)) return false;
    slant_m = (float)dist_mm / 1000.0f; return true;
  }

  float computeCorrectedLidarMeters(float slant_m) {
    float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
    float vf = (2.0f * (q1 * q3 - q0 * q2) * SIN_45) + (((q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3)) * COS_45);
    if (vf < 0.15f) vf = 0.15f;
    float off_m = constrainF(currentConfig.lidar_offset_m, CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);
    float c = (slant_m * vf) + off_m;
    return (c < 0.0f) ? 0.0f : c;
  }

  bool readLidar(float &dist_m) {
    if (!currentConfig.lidar_en || !activeSensors.lidar_found) return false;
    float sm = 0.0f; if (!readRawLidarSlantMeters(sm)) return false;
    dbg_lidar_m = adaptive_lidar.update(computeCorrectedLidarMeters(sm), currentConfig.lidar_filter_alpha);
    dist_m = dbg_lidar_m; last_lidar_packet_ms = millis(); return true;
  }

  void calibrateLidar() {
    if (!activeSensors.lidar_found) { usb_cdc.println("ERR: lidar"); return; }
    float sum = 0.0f; uint8_t ok = 0;
    for (uint8_t i = 0; i < 20; i++) { float sm = 0.0f; if (readRawLidarSlantMeters(sm)) { sum += sm; ok++; } delay(15); }
    if (ok > 0) {
      currentConfig.lidar_offset_m = constrainF(-(sum / (float)ok), CFG_LIDAR_OFFSET_MIN, CFG_LIDAR_OFFSET_MAX);
      adaptive_lidar.reset(); saveSettings(); usb_cdc.println("OK: cal_lid");
    } else usb_cdc.println("ERR: cal_lid");
  }

  void calibrateBatVoltage(float actual_v) {
    if (!isfinite(actual_v) || actual_v < 1.0f || actual_v > 60.0f) { usb_cdc.println("ERR: cal_bat"); return; }
    uint32_t sum = 0; for (uint8_t i = 0; i < 32; i++) { sum += adc1ReadChannel(4); delay(2); }
    float pv = (((float)sum / 32.0f) / 4095.0f) * 3.3f;
    if (pv < 0.01f) { usb_cdc.println("ERR: adc"); return; }
    currentConfig.bat_scale_v = constrainF(actual_v / pv, CFG_BAT_SCALE_V_MIN, CFG_BAT_SCALE_V_MAX);
    v_batt = lpf_battery.output = actual_v; batt_valid = true; power_w = v_batt * current_a;
    saveSettings(); usb_cdc.println("OK: cal_bat");
  }

  void calibrateCurrent(bool has_val, float actual_a) {
    uint32_t sum = 0; for (uint8_t i = 0; i < 32; i++) { sum += adc1ReadChannel(5); delay(2); }
    float sa = ((((float)sum / 32.0f) / 4095.0f) * 3.3f) / SHUNT_RESISTANCE_OHM;
    if (!has_val || fabsf(actual_a) < 0.001f) {
      currentConfig.bat_offset_a = constrainF(sa, CFG_BAT_OFFSET_A_MIN, CFG_BAT_OFFSET_A_MAX);
      current_a = power_w = lpf_current.output = 0.0f;
      saveSettings(); usb_cdc.println("OK: cal_curr"); return;
    }
    if (!isfinite(actual_a) || actual_a < 0.0f || actual_a > 250.0f) { usb_cdc.println("ERR: cal_curr"); return; }
    float da = sa - currentConfig.bat_offset_a;
    if (fabsf(da) < 0.01f) { usb_cdc.println("ERR: cal_curr"); return; }
    currentConfig.bat_scale_a = constrainF(actual_a / da, CFG_BAT_SCALE_A_MIN, CFG_BAT_SCALE_A_MAX);
    current_a = lpf_current.output = actual_a; power_w = v_batt * current_a;
    saveSettings(); usb_cdc.println("OK: cal_curr");
  }

  bool calibrateGyro(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) usb_cdc.println("ERR: imu"); return false; }
    float sgx = 0.0f, sgy = 0.0f, sgz = 0.0f, st = 0.0f; uint16_t ok = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t b[14];
      if (readMPU6050(b)) {
        float gx = (float)((int16_t)(b[8] << 8 | b[9])) / activeSensors.gyro_scale;
        float gy = (float)((int16_t)(b[10] << 8 | b[11])) / activeSensors.gyro_scale;
        float gz = (float)((int16_t)(b[12] << 8 | b[13])) / activeSensors.gyro_scale;
        rotateXYByYawAlign(gx, gy, currentConfig.imu_align);
        sgx += gx; sgy += gy; sgz += gz; st += extractTempFromImuBuf(b); ok++;
      }
      delay(4);
    }
    if (ok == 0) { if (!silent) usb_cdc.println("ERR: cal_gyro"); return false; }
    currentConfig.gyro_bias_x = sgx / (float)ok; currentConfig.gyro_bias_y = sgy / (float)ok;
    currentConfig.gyro_bias_z = sgz / (float)ok; currentConfig.base_temperature = st / (float)ok;
    if (!silent) { saveSettings(); usb_cdc.println("OK: cal_gyro"); }
    return true;
  }

  bool calibrateAcc(bool silent) {
    if (!activeSensors.imu_found) { if (!silent) usb_cdc.println("ERR: imu"); return false; }
    float sax = 0.0f, say = 0.0f, saz = 0.0f; uint16_t ok = 0;
    for (uint16_t i = 0; i < 200; i++) {
      uint8_t b[14];
      if (readMPU6050(b)) {
        float ax = (float)((int16_t)(b[0] << 8 | b[1])) / activeSensors.acc_scale;
        float ay = (float)((int16_t)(b[2] << 8 | b[3])) / activeSensors.acc_scale;
        float az = (float)((int16_t)(b[4] << 8 | b[5])) / activeSensors.acc_scale;
        rotateXYByYawAlign(ax, ay, currentConfig.imu_align);
        sax += ax; say += ay; saz += az; ok++;
      }
      delay(4);
    }
    if (ok == 0) { if (!silent) usb_cdc.println("ERR: cal_acc"); return false; }
    currentConfig.acc_bias_x = sax / (float)ok; currentConfig.acc_bias_y = say / (float)ok; currentConfig.acc_bias_z = (saz / (float)ok) - 1.0f;
    if (!silent) { saveSettings(); usb_cdc.println("OK: cal_acc"); }
    return true;
  }

  void calibrateIMU() {
    if (!activeSensors.imu_found) { usb_cdc.println("ERR: imu"); return; }
    if (!calibrateGyro(true) || !calibrateAcc(true)) { usb_cdc.println("ERR: cal_imu"); return; }
    saveSettings(); usb_cdc.println("OK: cal_imu");
  }

  void autoDetectAll() {
    err_aux_i2c_fail = false; detectIMU();
    if (currentConfig.baro_en) detectBaro(); else { activeSensors.baro_found = false; activeSensors.baro_type = BARO_UNKNOWN; }
    if (currentConfig.com_en) detectMag(); else { activeSensors.mag_found = false; activeSensors.mag_type = MAG_UNKNOWN; }
    if (currentConfig.lidar_en) { detectLidar(); initLidarHardware(); } else { activeSensors.lidar_found = false; activeSensors.lidar_type = LIDAR_NONE; }
  }
}

void startMagCalibrationNonBlocking() {
  if (!SensorDriver::activeSensors.mag_found) { usb_cdc.println("ERR: mag"); return; }
  mag_cal_active = true; mag_cal_start_ms = millis(); mag_cal_last_sample_ms = 0;
  mag_cal_min_x = mag_cal_min_y = mag_cal_min_z = 1e9f;
  mag_cal_max_x = mag_cal_max_y = mag_cal_max_z = -1e9f;
  usb_cdc.println("OK: cal_mag");
}

void stepMagCalibrationNonBlocking(uint32_t now_ms) {
  if (!mag_cal_active) return;
  if ((now_ms - mag_cal_last_sample_ms) >= MAG_CAL_SAMPLE_MS) {
    mag_cal_last_sample_ms = now_ms;
    float mx, my, mz;
    if (SensorDriver::readMagUnified(mx, my, mz)) {
      if (mx < mag_cal_min_x) mag_cal_min_x = mx;
      if (mx > mag_cal_max_x) mag_cal_max_x = mx;
      if (my < mag_cal_min_y) mag_cal_min_y = my;
      if (my > mag_cal_max_y) mag_cal_max_y = my;
      if (mz < mag_cal_min_z) mag_cal_min_z = mz;
      if (mz > mag_cal_max_z) mag_cal_max_z = mz;
    }
  }
  if ((now_ms - mag_cal_start_ms) >= MAG_CAL_DURATION_MS) {
    mag_cal_active = false;
    if (mag_cal_max_x > mag_cal_min_x && mag_cal_max_y > mag_cal_min_y && mag_cal_max_z > mag_cal_min_z) {
      currentConfig.mag_bias_x = constrainF(0.5f * (mag_cal_min_x + mag_cal_max_x), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      currentConfig.mag_bias_y = constrainF(0.5f * (mag_cal_min_y + mag_cal_max_y), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      currentConfig.mag_bias_z = constrainF(0.5f * (mag_cal_min_z + mag_cal_max_z), CFG_MAG_BIAS_MIN, CFG_MAG_BIAS_MAX);
      saveSettings(); usb_cdc.println("DONE: cal_mag");
    } else usb_cdc.println("ERR: cal_mag");
  }
}

void stepMonitorStreamNonBlocking(uint32_t now_ms) {
  if (!mon_stream_active || (now_ms - mon_last_send_ms) < MON_STREAM_PERIOD_MS) return;
  mon_last_send_ms = now_ms;
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float r = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float p = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
  usb_cdc.print("MON: err="); usb_cdc.print((int)getActiveStatusErrorCode());
  usb_cdc.print(" arm="); usb_cdc.print(is_armed ? 1 : 0);
  usb_cdc.print(" fs="); usb_cdc.print(is_failsafe ? 1 : 0);
  usb_cdc.print(" cpu="); usb_cdc.print(cpu_load_pct, 1); usb_cdc.print("/"); usb_cdc.print(cpu_max_load_pct, 1);
  usb_cdc.print(" vib="); usb_cdc.print(vibration_index, 1);
  usb_cdc.print(" rp="); usb_cdc.print(r, 1); usb_cdc.print(","); usb_cdc.print(p, 1);
  usb_cdc.print(" alt="); usb_cdc.print(baro_alt_m, 2); usb_cdc.print(" lid="); usb_cdc.print(dbg_lidar_m, 2);
  usb_cdc.print(" v="); usb_cdc.print(v_batt, 2); usb_cdc.print(" i="); usb_cdc.print(current_a, 2);
  usb_cdc.print(" lq="); usb_cdc.println((int)current_link_quality);
}

void readBatteryAndCurrent() {
  float iv = (((float)adc1ReadChannel(4) / 4095.0f) * 3.3f) * SensorDriver::getBatScaleV();
  if (!isfinite(iv) || iv < 0.0f || iv > 60.0f) {
    batt_valid = err_batt_low = false; setFcErrorRam(FC_ERR_BAT_SENSOR_FAIL); return;
  }
  if (iv > 0.5f) { if (!batt_valid) lpf_battery.output = iv; batt_valid = true; }
  v_batt = lpf_battery.update(iv);
  current_a = lpf_current.update(SensorDriver::computeCurrentAmpsFromAdc(adc1ReadChannel(5)));
  power_w = v_batt * current_a;
  if (is_armed) consumed_wh += (power_w * DT) / 3600.0f;
  if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { err_batt_low = true; setFcErrorRam(FC_ERR_BAT_LOW); }
  else err_batt_low = false;
}

void updateAuxSensorsNonBlocking(uint32_t now_ms) {
  static uint32_t last_aux_ms = 0, last_bvz_ms = 0;
  float alt_m = 0.0f;
  if (SensorDriver::stepBaroNonBlocking(now_ms, alt_m)) {
    baro_alt_m = alt_m;
    if (last_bvz_ms > 0 && (now_ms - last_bvz_ms) >= 10) {
      baro_vz_mps = lpf_baro_vz.update(constrainF((baro_alt_m - lpf_baro_alt.output) / ((float)(now_ms - last_bvz_ms) * 0.001f), -4.0f, 4.0f));
    }
    last_bvz_ms = now_ms;
  }
  if ((now_ms - last_aux_ms) < AUX_SENSOR_PERIOD_MS) return;
  last_aux_ms = now_ms;
  float lm = 0.0f;
  if (SensorDriver::readLidar(lm)) current_vz = constrainF((lm - adaptive_lidar.output) / (AUX_SENSOR_PERIOD_MS * 0.001f), -4.0f, 4.0f);
  float hd = 0.0f;
  if (SensorDriver::readCompassHeadingDeg(hd)) { compass_heading_deg = hd; compass_valid = true; }
  else compass_valid = false;
  readBatteryAndCurrent();
}

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
  if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) || !isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
    setFcErrorRam(FC_ERR_NAN_INF_MATH); orientation.normalize(); return;
  }
  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float an = sqrtf(ax * ax + ay * ay + az * az);
  float beta = 0.08f;
  if (currentConfig.flt_amb_en) {
    float g_dev = fabsf(an - 1.0f);
    float a = constrainF(currentConfig.flt_amb, CFG_ALPHA_MIN, CFG_ALPHA_MAX);
    float a_min = constrainF(currentConfig.flt_amb_min, 0.0005f, a);
    beta = (g_dev > 0.15f) ? a_min : (a - ((g_dev / 0.15f) * (a - a_min)));
  }
  float q0Dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float q1Dot = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float q2Dot = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float q3Dot = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (an > 1e-4f && isfinite(an)) {
    float ia = 1.0f / an; ax *= ia; ay *= ia; az *= ia;
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2, _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 1e-9f && isfinite(sn)) { float is = 1.0f / sn; q0Dot -= beta * s0 * is; q1Dot -= beta * s1 * is; q2Dot -= beta * s2 * is; q3Dot -= beta * s3 * is; }
  }
  if (compass_valid && isfinite(mx) && isfinite(my) && isfinite(mz)) {
    float mn = sqrtf(mx * mx + my * my + mz * mz);
    if (mn > 1e-4f) {
      float im = 1.0f / mn; mx *= im; my *= im; mz *= im;
      float yaw_rad = compass_heading_deg * FC_DEG_TO_RAD;
      float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
      float q0_m = 0.5f * sqrtf(fmaxf(0.0f, 1.0f + cy)), q3_m = (fabsf(q0_m) > 1e-5f) ? (sy / (2.0f * q0_m)) : 0.0f;
      float err_q3 = (q0 * q3_m - q3 * q0_m);
      q0Dot += 0.02f * (-q3 * err_q3); q1Dot += 0.02f * (-q2 * err_q3); q2Dot += 0.02f * (q1 * err_q3); q3Dot += 0.02f * (q0 * err_q3);
    }
  }
  q0 += q0Dot * dt; q1 += q1Dot * dt; q2 += q2Dot * dt; q3 += q3Dot * dt;
  if (!isfinite(q0) || !isfinite(q1) || !isfinite(q2) || !isfinite(q3)) { setFcErrorRam(FC_ERR_NAN_INF_MATH); orientation = Quaternion{}; return; }
  orientation = {q0, q1, q2, q3}; orientation.normalize();
}

void setupHardwareTimers() {
  gpioConfigPin(Reg::GPIOA, 0, GPIO_AF, GPIO_NO_PULL, 1, false);
  gpioConfigPin(Reg::GPIOA, 1, GPIO_AF, GPIO_NO_PULL, 1, false);
  gpioConfigPin(Reg::GPIOA, 2, GPIO_AF, GPIO_NO_PULL, 1, false);
  gpioConfigPin(Reg::GPIOA, 3, GPIO_AF, GPIO_NO_PULL, 1, false);

  Reg::TIM2->CR1 = 0;
  Reg::TIM2->PSC = 83;
  Reg::TIM2->ARR = 999;
  Reg::TIM2->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
  Reg::TIM2->CCMR2 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);
  Reg::TIM2->CCER  = (1U << 0) | (1U << 4) | (1U << 8) | (1U << 12);
  Reg::TIM2->CCR1  = 1000;
  Reg::TIM2->CCR2  = 1000;
  Reg::TIM2->CCR3  = 1000;
  Reg::TIM2->CCR4  = 1000;
  Reg::TIM2->CR1  |= (1U << 7) | (1U << 0);
}

void resetMotorSlewState() { prev_m1_out = prev_m2_out = prev_m3_out = prev_m4_out = 0.0f; arm_ramp_factor = 0.0f; }

static inline float applyMotorSlew(float target, float &prev_val, float max_delta, float max_cap) {
  float t = constrainF(target, OUTPUT_MIN, max_cap);
  if (!isfinite(prev_val)) prev_val = 0.0f;
  float d = t - prev_val;
  if (d > max_delta) d = max_delta; else if (d < -max_delta) d = -max_delta;
  prev_val = constrainF(prev_val + d, OUTPUT_MIN, max_cap);
  return prev_val;
}

static inline uint16_t motorPowerToPwmPulse(float power_0_1000) {
  float p = constrainF(power_0_1000, OUTPUT_MIN, OUTPUT_MAX);
  return (currentConfig.esc_prot == 1) ? (uint16_t)lroundf(125.0f + (p / OUTPUT_MAX) * 125.0f) : (uint16_t)lroundf(1000.0f + (p / OUTPUT_MAX) * 1000.0f);
}

void setMotorsPower(float m1, float m2, float m3, float m4, float max_cap) {
  float cap = constrainF(max_cap, OUTPUT_MIN, OUTPUT_MAX);
  if (!is_armed && !CliAutoCal::isFlightActive() && bench_test_mode == BENCH_TEST_NONE) {
    m1 = m2 = m3 = m4 = 0.0f; prev_m1_out = prev_m2_out = prev_m3_out = prev_m4_out = 0.0f;
  } else {
    float md = getEffectiveEscDeltaLimit();
    m1 = applyMotorSlew(m1, prev_m1_out, md, cap);
    m2 = applyMotorSlew(m2, prev_m2_out, md, cap);
    m3 = applyMotorSlew(m3, prev_m3_out, md, cap);
    m4 = applyMotorSlew(m4, prev_m4_out, md, cap);
  }
  Reg::TIM2->CCR1 = motorPowerToPwmPulse(m1);
  Reg::TIM2->CCR2 = motorPowerToPwmPulse(m2);
  Reg::TIM2->CCR3 = motorPowerToPwmPulse(m3);
  Reg::TIM2->CCR4 = motorPowerToPwmPulse(m4);
}

void stopBenchTest() {
  if (bench_test_mode != BENCH_TEST_NONE) {
    bench_test_mode = BENCH_TEST_NONE; resetMotorSlewState();
    if (!is_armed && !CliAutoCal::isFlightActive()) setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void startBenchTest(BenchTestMode mode) {
  if (is_armed || CliAutoCal::isFlightActive()) { usb_cdc.println("ERR: armed"); return; }
  bench_test_mode = mode; bench_test_start_ms = millis(); resetMotorSlewState();
  usb_cdc.println((mode == BENCH_TEST_MOTORS) ? "OK: test_motors" : "OK: test_pid");
}

void stepBenchTestNonBlocking(uint32_t now_ms) {
  if (bench_test_mode == BENCH_TEST_NONE) return;
  if (is_armed || CliAutoCal::isFlightActive() || (now_ms - bench_test_start_ms) >= BENCH_TEST_DURATION_MS) {
    stopBenchTest(); usb_cdc.println("DONE: test"); return;
  }
  uint32_t el = now_ms - bench_test_start_ms;
  float base = (BENCH_TEST_THROTTLE / 100.0f) * OUTPUT_MAX * 0.25f;
  if (bench_test_mode == BENCH_TEST_MOTORS) {
    uint8_t ph = (uint8_t)((el / 2500) % 4);
    setMotorsPower((ph == 0) ? base : 0.0f, (ph == 1) ? base : 0.0f, (ph == 2) ? base : 0.0f, (ph == 3) ? base : 0.0f, OUTPUT_MAX);
  } else {
    float p = sinf((float)el * 0.005f) * 35.0f;
    setMotorsPower(constrainF(base + p, 0, OUTPUT_MAX), constrainF(base - p, 0, OUTPUT_MAX), constrainF(base + p, 0, OUTPUT_MAX), constrainF(base - p, 0, OUTPUT_MAX), OUTPUT_MAX);
  }
}

float calculateSuperExpoRate(float in, float r, float s, float e) {
  float n = constrainF(in, -1.0f, 1.0f), a = fabsf(n), sign = (n >= 0.0f) ? 1.0f : -1.0f;
  float exp_v = a * ((1.0f - e) + e * a * a * a);
  float rc = r * 20.0f + 0.01f;
  return sign * ((rc * exp_v) / fmaxf(0.05f, 1.0f - (a * constrainF(s, 0.0f, 0.95f)))) * (MAX_RATE_DPS_PER_UNIT / 100.0f);
}

static inline float applyDeadband(float v, float db) {
  if (db <= 0.0f) return v;
  if (fabsf(v) <= db) return 0.0f;
  return (v > 0.0f) ? (v - db) : (v + db);
}

namespace CliAutoCal {
  enum State : uint8_t { IDLE = 0, TAKING_OFF, HOVERING, LANDING, COMPLETED, ABORTED };
  struct Session {
    State state = IDLE; uint32_t start_ms = 0, state_enter_ms = 0, last_print_ms = 0, last_sample_ms = 0;
    float target_alt_m = 0.8f, hover_sum_thr = 0.0f, learned_hover_thr = 0.0f, peak_alt_m = 0.0f, landing_base_thr = 0.0f;
    float gyro_sum_x = 0.0f, gyro_sum_y = 0.0f, gyro_sum_z = 0.0f, temp_sum = 0.0f;
    uint16_t hover_samples = 0, gyro_samples = 0;
    bool has_hover_sample = false, has_gyro_sample = false, touchdown_latched = false, flipped_latched = false;
  };
  static Session s;
  static uint32_t btn_press_start_ms = 0;
  static bool btn_prev_pressed = false, btn_long_fired = false;
  static inline bool isPhysicalButtonPressed() { return !gpioRead(Reg::GPIOC, 13); }
  bool isFlightActive() { return s.state == TAKING_OFF || s.state == HOVERING || s.state == LANDING; }

  static void enterState(State ns, uint32_t now_ms) {
    s.state = ns; s.state_enter_ms = now_ms;
    if (ns == HOVERING) { s.hover_sum_thr = 0.0f; s.hover_samples = 0; }
    else if (ns == LANDING) { s.landing_base_thr = (s.has_hover_sample && s.learned_hover_thr > 50.0f) ? s.learned_hover_thr : constrainF(base_hover_throttle, 80.0f, 650.0f); s.touchdown_latched = false; }
  }

  void stop(uint8_t reason_code, uint8_t fc_err_code) {
    bool was_act = isFlightActive();
    s.state = (reason_code == 0) ? COMPLETED : ABORTED;
    is_armed = false; resetMotorSlewState(); setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
    if (fc_err_code != FC_ERR_NONE) setFcErrorRam(fc_err_code);
    if (was_act) {
      usb_cdc.print("CAL_AUTO_END: reason="); usb_cdc.print((int)reason_code);
      usb_cdc.print(" hover="); usb_cdc.print(s.learned_hover_thr, 1);
      usb_cdc.print(" peak_alt="); usb_cdc.println(s.peak_alt_m, 2);
    }
  }

  void init() { gpioConfigPin(Reg::GPIOC, 13, GPIO_INPUT, GPIO_PULL_UP); memset(&s, 0, sizeof(s)); s.target_alt_m = 0.8f; }

  void start() {
    if (isFlightActive()) { usb_cdc.println("ERR: cal_auto already running"); return; }
    stopBenchTest();
    if (!SensorDriver::activeSensors.imu_found || err_imu_fail) { setFcErrorRam(FC_ERR_IMU_FAIL); usb_cdc.println("ERR: cal_auto imu"); return; }
    if (currentConfig.lidar_en && !SensorDriver::activeSensors.lidar_found) { setFcErrorRam(FC_ERR_LIDAR_FAIL); usb_cdc.println("ERR: cal_auto lidar"); return; }
    if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { setFcErrorRam(FC_ERR_BAT_LOW); usb_cdc.println("ERR: cal_auto bat"); return; }
    uint32_t now = millis();
    memset(&s, 0, sizeof(s)); s.target_alt_m = 0.8f; s.start_ms = s.state_enter_ms = now;
    orientation = Quaternion{}; vzPID.reset(); rollRatePID.reset(); pitchRatePID.reset(); yawRatePID.reset();
    resetMotorSlewState(); is_failsafe = false; is_armed = true; prev_armed_state = true;
    fc_error_ram = fc_latched_error_ram = last_sent_202_err = FC_ERR_NONE;
    enterState(TAKING_OFF, now);
    usb_cdc.println("OK: cal_auto");
  }

  void onDisarmRequested() { if (isFlightActive()) stop(2, FC_ERR_NONE); }
  void onArmRequested() { if (isFlightActive()) stop(2, FC_ERR_NONE); }

  void onAttitudeAndMotion(float ax_g, float ay_g, float az_g, float roll_deg, float pitch_deg, float gx_dps, float gy_dps) {
    if (!isFlightActive()) return;
    if (fabsf(roll_deg) > 65.0f || fabsf(pitch_deg) > 65.0f) { s.flipped_latched = true; stop(3, FC_ERR_TILT_DISARM); return; }
    float am = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float rm = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps);
    if (s.state == LANDING && (millis() - s.state_enter_ms) > 400 && ((am > 0.72f && am < 1.32f && rm < 25.0f) || (dbg_lidar_m > 0.001f && dbg_lidar_m < 0.04f))) {
      s.touchdown_latched = true;
    }
  }

  bool stepFlight(uint32_t now_ms, float gx_dps, float gy_dps, float gz_dps, float gx_rad, float gy_rad, float gz_rad,
                  float roll_deg, float pitch_deg, float &m1, float &m2, float &m3, float &m4) {
    if (!isFlightActive()) return false;
    if ((now_ms - s.start_ms) > 15000) { stop(4, FC_ERR_NONE); return true; }
    if (currentConfig.fs_bat_en && batt_valid && v_batt < currentConfig.bat_fs_v) { stop(5, FC_ERR_BAT_LOW); return true; }
    if (s.flipped_latched) { stop(3, FC_ERR_TILT_DISARM); return true; }

    float alt = (currentConfig.lidar_en && SensorDriver::activeSensors.lidar_found && dbg_lidar_m > 0.005f) ? dbg_lidar_m : baro_alt_m;
    if (alt > s.peak_alt_m) s.peak_alt_m = alt;
    uint32_t se = now_ms - s.state_enter_ms;
    float tvz = 0.0f;

    if (s.state == TAKING_OFF) {
      tvz = (alt < (s.target_alt_m - 0.08f)) ? 0.45f : 0.15f;
      if (alt >= (s.target_alt_m - 0.05f) || se >= 3500) enterState(HOVERING, now_ms);
    } else if (s.state == HOVERING) {
      tvz = constrainF((s.target_alt_m - alt) * 0.9f, -0.35f, 0.35f);
      if (se >= 1000 && se <= 4500 && fabsf(alt - s.target_alt_m) < 0.25f && fabsf(roll_deg) < 12.0f && fabsf(pitch_deg) < 12.0f) {
        float est = constrainF(base_hover_throttle + vzPID.update(tvz, current_vz, DT, false), 60.0f, 700.0f);
        s.hover_sum_thr += est; s.hover_samples++; s.has_hover_sample = true; s.learned_hover_thr = s.hover_sum_thr / (float)s.hover_samples;
        if ((now_ms - s.last_sample_ms) >= 10) {
          s.last_sample_ms = now_ms; s.gyro_sum_x += gx_dps; s.gyro_sum_y += gy_dps; s.gyro_sum_z += gz_dps;
          s.temp_sum += SensorDriver::extractTempFromImuBuf(mpu_dma_buffer); s.gyro_samples++; s.has_gyro_sample = true;
        }
      }
      if (se >= 5000) enterState(LANDING, now_ms);
    } else if (s.state == LANDING) {
      tvz = (alt > 0.25f) ? -0.35f : -0.20f;
      if (s.touchdown_latched || alt <= 0.05f || se >= 5000) {
        if (s.has_hover_sample && s.learned_hover_thr > 50.0f) { base_hover_throttle = s.learned_hover_thr; currentConfig.flt_esc_limit = constrainF(s.learned_hover_thr * 0.35f, 15.0f, 120.0f); }
        if (s.has_gyro_sample && s.gyro_samples >= 20) {
          currentConfig.gyro_bias_x += s.gyro_sum_x / (float)s.gyro_samples; currentConfig.gyro_bias_y += s.gyro_sum_y / (float)s.gyro_samples;
          currentConfig.gyro_bias_z += s.gyro_sum_z / (float)s.gyro_samples; currentConfig.base_temperature = s.temp_sum / (float)s.gyro_samples;
        }
        saveSettings(); stop(0, FC_ERR_NONE); return true;
      }
    }

    float base_t = (s.state == LANDING) ? s.landing_base_thr : ((s.has_hover_sample && s.learned_hover_thr > 50.0f) ? s.learned_hover_thr : constrainF(base_hover_throttle, 80.0f, 650.0f));
    float thr = constrainF(base_t + vzPID.update(tvz, current_vz, DT, false), 40.0f, 750.0f) * getBatteryCompensationFactor();
    float ro = rollRatePID.update(constrainF(-roll_deg * 2.0f, -120.0f, 120.0f) - gx_dps, gx_dps, DT, false);
    float po = pitchRatePID.update(constrainF(-pitch_deg * 2.0f, -120.0f, 120.0f) - gy_dps, gy_dps, DT, false);
    float yo = yawRatePID.update(-gz_dps, gz_dps, DT, false);

    m1 = constrainF(thr + po + ro - yo, OUTPUT_MIN, OUTPUT_MAX); m2 = constrainF(thr + po - ro + yo, OUTPUT_MIN, OUTPUT_MAX);
    m3 = constrainF(thr - po - ro - yo, OUTPUT_MIN, OUTPUT_MAX); m4 = constrainF(thr - po + ro + yo, OUTPUT_MIN, OUTPUT_MAX);

    if ((now_ms - s.last_print_ms) >= 250) {
      s.last_print_ms = now_ms;
      usb_cdc.print("CAL_AUTO: st="); usb_cdc.print((int)s.state); usb_cdc.print(" alt="); usb_cdc.print(alt, 2);
      usb_cdc.print(" thr="); usb_cdc.print(thr, 0); usb_cdc.print(" h_est="); usb_cdc.println(s.learned_hover_thr, 1);
    }
    (void)gx_rad; (void)gy_rad; (void)gz_rad; return true;
  }

  void pollButton(uint32_t now_ms) {
    bool pr = isPhysicalButtonPressed();
    if (pr && !btn_prev_pressed) { btn_press_start_ms = now_ms; btn_long_fired = false; }
    else if (pr && btn_prev_pressed && !btn_long_fired && (now_ms - btn_press_start_ms) >= 1200) {
      btn_long_fired = true;
      if (isFlightActive()) stop(2, FC_ERR_NONE); else start();
    } else if (!pr && btn_prev_pressed && !btn_long_fired && (now_ms - btn_press_start_ms) >= 40) {
      if (isFlightActive()) stop(2, FC_ERR_NONE);
      else { if (SensorDriver::activeSensors.imu_found) SensorDriver::calibrateIMU(); if (SensorDriver::activeSensors.baro_found) SensorDriver::calibrateBaro(); }
    }
    btn_prev_pressed = pr;
  }
}

void flightControlTask() {
  uint32_t now_ms = millis();
  if (currentConfig.rtx_en && ((now_ms - last_rc_packet_ms) > FAILSAFE_TIMEOUT_MS)) {
    if (!is_failsafe) { is_failsafe = true; err_rc_loss = true; setFcErrorRam(FC_ERR_RC_LOSS); if (currentConfig.fs_rtx_en && !CliAutoCal::isFlightActive()) is_armed = false; }
  } else if (is_failsafe) { is_failsafe = false; err_rc_loss = false; }

  handleMotorArmStateAndEepromError();
  if (!SensorDriver::activeSensors.imu_found || !SensorDriver::readMPU6050(mpu_dma_buffer)) {
    err_imu_fail = true; setFcErrorRam(FC_ERR_IMU_FAIL);
    if (CliAutoCal::isFlightActive()) CliAutoCal::stop(6, FC_ERR_IMU_FAIL);
    is_armed = false; setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f); return;
  }
  err_imu_fail = false;

  float tc = SensorDriver::extractTempFromImuBuf(mpu_dma_buffer);
  if (isfinite(tc)) current_chip_temp = lpf_temp.update(tc);
  float dt = fmaxf(0.0f, current_chip_temp - currentConfig.base_temperature);
  float gx_raw = (float)((int16_t)(mpu_dma_buffer[8] << 8 | mpu_dma_buffer[9])) / SensorDriver::activeSensors.gyro_scale;
  float gy_raw = (float)((int16_t)(mpu_dma_buffer[10] << 8 | mpu_dma_buffer[11])) / SensorDriver::activeSensors.gyro_scale;
  float gz_raw = (float)((int16_t)(mpu_dma_buffer[12] << 8 | mpu_dma_buffer[13])) / SensorDriver::activeSensors.gyro_scale;
  float ax_raw = (float)((int16_t)(mpu_dma_buffer[0] << 8 | mpu_dma_buffer[1])) / SensorDriver::activeSensors.acc_scale;
  float ay_raw = (float)((int16_t)(mpu_dma_buffer[2] << 8 | mpu_dma_buffer[3])) / SensorDriver::activeSensors.acc_scale;
  float az_raw = (float)((int16_t)(mpu_dma_buffer[4] << 8 | mpu_dma_buffer[5])) / SensorDriver::activeSensors.acc_scale;

  SensorDriver::rotateXYByYawAlign(gx_raw, gy_raw, currentConfig.imu_align);
  SensorDriver::rotateXYByYawAlign(ax_raw, ay_raw, currentConfig.imu_align);

  float gx_dps = gx_raw - (currentConfig.gyro_bias_x + currentConfig.temp_coeff_x * dt);
  float gy_dps = gy_raw - (currentConfig.gyro_bias_y + currentConfig.temp_coeff_y * dt);
  float gz_dps = gz_raw - (currentConfig.gyro_bias_z + currentConfig.temp_coeff_z * dt);
  float ax_g = ax_raw - currentConfig.acc_bias_x, ay_g = ay_raw - currentConfig.acc_bias_y, az_g = az_raw - currentConfig.acc_bias_z;

  if (!isfinite(gx_dps) || !isfinite(gy_dps) || !isfinite(gz_dps) || !isfinite(ax_g) || !isfinite(ay_g) || !isfinite(az_g)) {
    setFcErrorRam(FC_ERR_NAN_INF_MATH); is_armed = false; setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f); return;
  }

  vibration_index = 0.90f * vibration_index + 0.10f * sqrtf(lpf_xy_hp_x.update(ax_g) * lpf_xy_hp_x.update(ax_g) + lpf_xy_hp_y.update(ay_g) * lpf_xy_hp_y.update(ay_g)) * 100.0f;

  if (currentConfig.flt_lpf_en) { gx_dps = lpf_gx.update(gx_dps); gy_dps = lpf_gy.update(gy_dps); gz_dps = lpf_gz.update(gz_dps); ax_g = lpf_ax.update(ax_g); ay_g = lpf_ay.update(ay_g); az_g = lpf_az.update(az_g); }
  if (currentConfig.flt_not_en) { gx_dps = notch_gx.update(gx_dps); gy_dps = notch_gy.update(gy_dps); gz_dps = notch_gz.update(gz_dps); }

  float gx_r = gx_dps * FC_DEG_TO_RAD, gy_r = gy_dps * FC_DEG_TO_RAD, gz_r = gz_dps * FC_DEG_TO_RAD;
  float mx_c = 0.0f, my_c = 0.0f, mz_c = 0.0f;
  bool has_m = (compass_valid && SensorDriver::readMagUnified(mx_c, my_c, mz_c));
  if (has_m) { mx_c -= currentConfig.mag_bias_x; my_c -= currentConfig.mag_bias_y; mz_c -= currentConfig.mag_bias_z; }
  madgwickUpdate(gx_r, gy_r, gz_r, ax_g, ay_g, az_g, mx_c, my_c, mz_c, DT);

  float q0 = orientation.q0, q1 = orientation.q1, q2 = orientation.q2, q3 = orientation.q3;
  float roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * FC_RAD_TO_DEG;
  float pitch_deg = asinf(constrainF(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * FC_RAD_TO_DEG;
  CliAutoCal::onAttitudeAndMotion(ax_g, ay_g, az_g, roll_deg, pitch_deg, gx_dps, gy_dps);

  float m1 = 0.0f, m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;
  if (CliAutoCal::stepFlight(now_ms, gx_dps, gy_dps, gz_dps, gx_r, gy_r, gz_r, roll_deg, pitch_deg, m1, m2, m3, m4)) {
    setMotorsPower(m1, m2, m3, m4, OUTPUT_MAX); return;
  }
  if (!is_armed) {
    if (bench_test_mode == BENCH_TEST_NONE) setMotorsPower(0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }
  if (arm_ramp_factor < 1.0f) arm_ramp_factor = fminf(1.0f, arm_ramp_factor + (DT / 0.35f));

  float r_st = applyDeadband(smooth_roll - STICK_CENTER, currentConfig.rc_deadband) / (STICK_CENTER - currentConfig.rc_deadband);
  float p_st = applyDeadband(smooth_pitch - STICK_CENTER, currentConfig.rc_deadband) / (STICK_CENTER - currentConfig.rc_deadband);
  float y_st = applyDeadband(smooth_yaw - STICK_CENTER, currentConfig.rc_deadband) / (STICK_CENTER - currentConfig.rc_deadband);
  float t_st = constrainF(smooth_throttle / 1000.0f, 0.0f, 1.0f);

  float tr = calculateSuperExpoRate(r_st, currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo);
  float tp = calculateSuperExpoRate(p_st, currentConfig.rc_rate, currentConfig.rc_super_expo, currentConfig.rc_stick_expo);
  float ty = calculateSuperExpoRate(y_st, currentConfig.rc_yaw_rate, currentConfig.rc_yaw_super_expo, currentConfig.rc_yaw_expo);

  if (currentConfig.fm_mode == 1 || currentConfig.fm_mode == 2) {
    float w = (currentConfig.fm_mode == 1) ? 1.0f : (1.0f - constrainF((fmaxf(fabsf(r_st), fabsf(p_st)) - currentConfig.rc_horizon_transition) / (1.0f - currentConfig.rc_horizon_transition), 0.0f, 1.0f));
    tr = w * constrainF((constrainF(r_st, -1.0f, 1.0f) * currentConfig.fm_roll_max - roll_deg) * 2.5f, -300.0f, 300.0f) + (1.0f - w) * tr;
    tp = w * constrainF((constrainF(p_st, -1.0f, 1.0f) * currentConfig.fm_pitch_max - pitch_deg) * 2.5f, -300.0f, 300.0f) + (1.0f - w) * tp;
  }

  float thr = t_st * OUTPUT_MAX;
  if (currentConfig.fm_mode == 3 && currentConfig.lidar_en && SensorDriver::activeSensors.lidar_found) {
    float vz_t = ((t_st - 0.5f) * 2.0f) * currentConfig.fm_vz_speed_max;
    thr = constrainF(base_hover_throttle + vzPID.update(vz_t, current_vz, DT, false), 50.0f, 850.0f);
  }
  thr = constrainF(thr * getBatteryCompensationFactor() * arm_ramp_factor, OUTPUT_MIN, OUTPUT_MAX);

  float ro = rollRatePID.update(tr - gx_dps, gx_dps, DT, false) * arm_ramp_factor;
  float po = pitchRatePID.update(tp - gy_dps, gy_dps, DT, false) * arm_ramp_factor;
  float yo = yawRatePID.update(ty - gz_dps, gz_dps, DT, false) * arm_ramp_factor;

  m1 = constrainF(thr + po + ro - yo, OUTPUT_MIN, OUTPUT_MAX); m2 = constrainF(thr + po - ro + yo, OUTPUT_MIN, OUTPUT_MAX);
  m3 = constrainF(thr - po - ro - yo, OUTPUT_MIN, OUTPUT_MAX); m4 = constrainF(thr - po + ro + yo, OUTPUT_MIN, OUTPUT_MAX);
  setMotorsPower(m1, m2, m3, m4, OUTPUT_MAX);
}

static bool matchKeyFloat(const char *line, const char *key, float &out_val) {
  size_t klen = strlen(key);
  if (strncmp(line, key, klen) != 0 || line[klen] != '=') return false;
  char *endp = nullptr;
  float v = strtof(line + klen + 1, &endp);
  if (endp == line + klen + 1 || !isfinite(v)) return false;
  out_val = v; return true;
}

static bool matchKeyInt(const char *line, const char *key, int &out_val) {
  size_t klen = strlen(key);
  if (strncmp(line, key, klen) != 0 || line[klen] != '=') return false;
  char *endp = nullptr;
  long v = strtol(line + klen + 1, &endp, 0);
  if (endp == line + klen + 1) return false;
  out_val = (int)v; return true;
}

void executeCliCommand(char *cmd_line) {
  while (*cmd_line == ' ' || *cmd_line == '\t') cmd_line++;
  size_t len = strlen(cmd_line);
  while (len > 0 && (cmd_line[len - 1] == ' ' || cmd_line[len - 1] == '\t' || cmd_line[len - 1] == '\r' || cmd_line[len - 1] == '\n')) cmd_line[--len] = '\0';
  if (len == 0) return;

  if (strcmp(cmd_line, "info") == 0 || strcmp(cmd_line, "status") == 0) {
    usb_cdc.print("INFO: ver=4.52-F401-CDC err="); usb_cdc.print((int)getActiveStatusErrorCode());
    usb_cdc.print(" imu="); usb_cdc.print((int)SensorDriver::activeSensors.imu_type);
    usb_cdc.print(" bar="); usb_cdc.print((int)SensorDriver::activeSensors.baro_type);
    usb_cdc.print(" mag="); usb_cdc.print((int)SensorDriver::activeSensors.mag_type);
    usb_cdc.print(" lid="); usb_cdc.print((int)SensorDriver::activeSensors.lidar_type);
    usb_cdc.print(" rtx="); usb_cdc.print((int)rtx_active_chip_id);
    usb_cdc.print(" vtx_p="); usb_cdc.print((int)vtx_detected_proto);
    usb_cdc.print(" v="); usb_cdc.print(v_batt, 2); usb_cdc.print(" i="); usb_cdc.println(current_a, 2);
    return;
  }
  if (strcmp(cmd_line, "save") == 0) { saveSettings(); usb_cdc.println("OK: save"); return; }
  if (strcmp(cmd_line, "default") == 0 || strcmp(cmd_line, "reset") == 0) { resetToFactoryDefaults(); usb_cdc.println("OK: default"); return; }
  if (strcmp(cmd_line, "reboot") == 0) { usb_cdc.println("OK: reboot"); usb_cdc.flush(); Reg::SCB->AIRCR = (0x5FAUL << 16) | (1U << 2); while (1) {} }
  if (strcmp(cmd_line, "cal_gyro") == 0) { SensorDriver::calibrateGyro(false); return; }
  if (strcmp(cmd_line, "cal_acc") == 0) { SensorDriver::calibrateAcc(false); return; }
  if (strcmp(cmd_line, "cal_imu") == 0) { SensorDriver::calibrateIMU(); return; }
  if (strcmp(cmd_line, "cal_mag") == 0) { startMagCalibrationNonBlocking(); return; }
  if (strcmp(cmd_line, "cal_bar") == 0) { SensorDriver::calibrateBaro(); return; }
  if (strcmp(cmd_line, "cal_lid") == 0) { SensorDriver::calibrateLidar(); return; }
  if (strncmp(cmd_line, "cal_bat ", 8) == 0) { SensorDriver::calibrateBatVoltage(strtof(cmd_line + 8, nullptr)); return; }
  if (strcmp(cmd_line, "cal_curr") == 0) { SensorDriver::calibrateCurrent(false, 0.0f); return; }
  if (strncmp(cmd_line, "cal_curr ", 9) == 0) { SensorDriver::calibrateCurrent(true, strtof(cmd_line + 9, nullptr)); return; }
  if (strcmp(cmd_line, "cal_auto") == 0) { CliAutoCal::start(); return; }
  if (strcmp(cmd_line, "cal_auto_stop") == 0) { CliAutoCal::stop(2, FC_ERR_NONE); usb_cdc.println("OK: cal_auto_stop"); return; }
  if (strcmp(cmd_line, "mon_start") == 0) { mon_stream_active = true; mon_last_send_ms = 0; usb_cdc.println("OK: mon_start"); return; }
  if (strcmp(cmd_line, "mon_stop") == 0) { mon_stream_active = false; usb_cdc.println("OK: mon_stop"); return; }
  if (strcmp(cmd_line, "test_motors") == 0) { startBenchTest(BENCH_TEST_MOTORS); return; }
  if (strcmp(cmd_line, "test_pid") == 0) { startBenchTest(BENCH_TEST_PID); return; }
  if (strcmp(cmd_line, "test_stop") == 0) { stopBenchTest(); usb_cdc.println("OK: test_stop"); return; }
  if (strcmp(cmd_line, "scan_rtx") == 0) { startChannelScanNonBlocking(); usb_cdc.println("OK: scan_rtx"); return; }
  if (strncmp(cmd_line, "vtx ", 4) == 0) {
    unsigned int mhz = 0, mw = 0;
    if (sscanf(cmd_line + 4, "%u %u", &mhz, &mw) == 2) {
      vtxSetFrequencyAndPower((uint16_t)mhz, (uint16_t)mw, true); usb_cdc.println("OK: vtx");
    } else usb_cdc.println("ERR: vtx");
    return;
  }

  float fv = 0.0f; int iv = 0; bool chg = false, flt = false, pid = false, rtx = false, vtx = false;
  if (matchKeyFloat(cmd_line, "fm_roll_max", fv)) { currentConfig.fm_roll_max = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "fm_pitch_max", fv)) { currentConfig.fm_pitch_max = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "fm_vz_speed_max", fv)) { currentConfig.fm_vz_speed_max = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_rate", fv)) { currentConfig.rc_rate = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_super_expo", fv)) { currentConfig.rc_super_expo = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_stick_expo", fv)) { currentConfig.rc_stick_expo = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_yaw_rate", fv)) { currentConfig.rc_yaw_rate = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_yaw_super_expo", fv)) { currentConfig.rc_yaw_super_expo = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_yaw_expo", fv)) { currentConfig.rc_yaw_expo = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_deadband", fv)) { currentConfig.rc_deadband = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_horizon_transition", fv)) { currentConfig.rc_horizon_transition = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "rc_filter_smoothing", fv)) { currentConfig.rc_filter_smoothing = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "flt_esc_limit", fv)) { currentConfig.flt_esc_limit = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "gyro_filter_alpha", fv)) { currentConfig.gyro_filter_alpha = fv; chg = flt = true; }
  else if (matchKeyFloat(cmd_line, "acc_filter_alpha", fv)) { currentConfig.acc_filter_alpha = fv; chg = flt = true; }
  else if (matchKeyFloat(cmd_line, "filter_notch_freq", fv)) { currentConfig.filter_notch_freq = fv; chg = flt = true; }
  else if (matchKeyFloat(cmd_line, "filter_notch_bw", fv)) { currentConfig.filter_notch_bw = fv; chg = flt = true; }
  else if (matchKeyFloat(cmd_line, "flt_amb", fv)) { currentConfig.flt_amb = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "flt_amb_min", fv)) { currentConfig.flt_amb_min = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "bat_scale_v", fv)) { currentConfig.bat_scale_v = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "bat_v_max", fv)) { currentConfig.bat_v_max = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "bat_filter_alpha", fv)) { currentConfig.bat_filter_alpha = fv; chg = flt = true; }
  else if (matchKeyFloat(cmd_line, "bat_offset_a", fv)) { currentConfig.bat_offset_a = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "bat_scale_a", fv)) { currentConfig.bat_scale_a = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "bat_fs_v", fv)) { currentConfig.bat_fs_v = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "fs_vz_speed", fv)) { currentConfig.fs_vz_speed = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "lidar_filter_alpha", fv)) { currentConfig.lidar_filter_alpha = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "lidar_offset_m", fv)) { currentConfig.lidar_offset_m = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "mag_bias_x", fv)) { currentConfig.mag_bias_x = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "mag_bias_y", fv)) { currentConfig.mag_bias_y = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "mag_bias_z", fv)) { currentConfig.mag_bias_z = fv; chg = true; }
  else if (matchKeyFloat(cmd_line, "roll_kp", fv)) { currentConfig.roll_pid.kp = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "roll_ki", fv)) { currentConfig.roll_pid.ki = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "roll_kd", fv)) { currentConfig.roll_pid.kd = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "pitch_kp", fv)) { currentConfig.pitch_pid.kp = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "pitch_ki", fv)) { currentConfig.pitch_pid.ki = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "pitch_kd", fv)) { currentConfig.pitch_pid.kd = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "yaw_kp", fv)) { currentConfig.yaw_pid.kp = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "yaw_ki", fv)) { currentConfig.yaw_pid.ki = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "yaw_kd", fv)) { currentConfig.yaw_pid.kd = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "vz_kp", fv)) { currentConfig.vz_pid.kp = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "vz_ki", fv)) { currentConfig.vz_pid.ki = fv; chg = pid = true; }
  else if (matchKeyFloat(cmd_line, "vz_kd", fv)) { currentConfig.vz_pid.kd = fv; chg = pid = true; }
  else if (matchKeyInt(cmd_line, "fm_mode", iv)) { currentConfig.fm_mode = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "esc_prot", iv)) { currentConfig.esc_prot = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "flt_lpf_en", iv)) { currentConfig.flt_lpf_en = (uint8_t)iv; chg = flt = true; }
  else if (matchKeyInt(cmd_line, "flt_not_en", iv)) { currentConfig.flt_not_en = (uint8_t)iv; chg = flt = true; }
  else if (matchKeyInt(cmd_line, "flt_amb_en", iv)) { currentConfig.flt_amb_en = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "bat_comp_en", iv)) { currentConfig.bat_comp_en = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "fs_bat_en", iv)) { currentConfig.fs_bat_en = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "fs_rtx_en", iv)) { currentConfig.fs_rtx_en = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "lidar_en", iv)) { currentConfig.lidar_en = (uint8_t)iv; chg = true; if (currentConfig.lidar_en && !SensorDriver::activeSensors.lidar_found) { SensorDriver::detectLidar(); SensorDriver::initLidarHardware(); } }
  else if (matchKeyInt(cmd_line, "lid_id", iv)) { currentConfig.lid_id = (uint8_t)iv; chg = true; if (currentConfig.lidar_en) { SensorDriver::detectLidar(); SensorDriver::initLidarHardware(); } }
  else if (matchKeyInt(cmd_line, "baro_en", iv)) { currentConfig.baro_en = (uint8_t)iv; chg = true; if (currentConfig.baro_en && !SensorDriver::activeSensors.baro_found) SensorDriver::detectBaro(); }
  else if (matchKeyInt(cmd_line, "bar_id", iv)) { currentConfig.bar_id = (uint8_t)iv; chg = true; if (currentConfig.baro_en) SensorDriver::detectBaro(); }
  else if (matchKeyInt(cmd_line, "com_en", iv)) { currentConfig.com_en = (uint8_t)iv; chg = true; if (currentConfig.com_en && !SensorDriver::activeSensors.mag_found) SensorDriver::detectMag(); }
  else if (matchKeyInt(cmd_line, "com_id", iv)) { currentConfig.com_id = (uint8_t)iv; chg = true; if (currentConfig.com_en) SensorDriver::detectMag(); }
  else if (matchKeyInt(cmd_line, "com_align", iv)) { currentConfig.com_align = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "imu_id", iv)) { currentConfig.imu_id = (uint8_t)iv; chg = true; SensorDriver::detectIMU(); }
  else if (matchKeyInt(cmd_line, "imu_align", iv)) { currentConfig.imu_align = (uint8_t)iv; chg = true; }
  else if (matchKeyInt(cmd_line, "vtx_en", iv)) { currentConfig.vtx_en = (uint8_t)iv; chg = vtx = true; }
  else if (matchKeyInt(cmd_line, "vtx_prot", iv)) { currentConfig.vtx_prot = (uint8_t)iv; chg = vtx = true; }
  else if (matchKeyInt(cmd_line, "vtx_mhz", iv)) { currentConfig.vtx_mhz = (uint16_t)iv; chg = vtx = true; vtx_send_freq_req = true; }
  else if (matchKeyInt(cmd_line, "vtx_mw", iv)) { currentConfig.vtx_mw = (uint16_t)iv; chg = vtx = true; vtx_send_power_req = true; }
  else if (matchKeyInt(cmd_line, "rtx_en", iv)) { currentConfig.rtx_en = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_id", iv)) { currentConfig.rtx_id = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_band", iv)) { currentConfig.rtx_band = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_pri_chan", iv)) { currentConfig.rtx_pri_chan = (uint8_t)iv; main_rtx_chan = active_rtx_chan = (uint8_t)constrainI(iv, 0, 99); is_on_backup = false; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_pri_sf", iv)) { currentConfig.rtx_pri_sf = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_pri_bw", iv)) { currentConfig.rtx_pri_bw = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_pri_cr", iv)) { currentConfig.rtx_pri_cr = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_pri_power", iv)) { currentConfig.rtx_pri_power = (int8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_bak_chan", iv)) { currentConfig.rtx_bak_chan = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_bak_sf", iv)) { currentConfig.rtx_bak_sf = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_bak_bw", iv)) { currentConfig.rtx_bak_bw = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_bak_cr", iv)) { currentConfig.rtx_bak_cr = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_bak_power", iv)) { currentConfig.rtx_bak_power = (int8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_addr", iv)) { currentConfig.rtx_addr = (uint16_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_addr_en", iv)) { currentConfig.rtx_addr_en = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_hw_crc_en", iv)) { currentConfig.rtx_hw_crc_en = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_iq_inv", iv)) { currentConfig.rtx_iq_inv = (uint8_t)iv; chg = rtx = true; }
  else if (matchKeyInt(cmd_line, "rtx_sync_word", iv)) { currentConfig.rtx_sync_word = (uint16_t)iv; chg = rtx = true; }
  else { usb_cdc.print("ERR: unknown cmd: "); usb_cdc.println(cmd_line); return; }

  if (chg) {
    sanitizeConfig();
    if (flt) applyFilterConfig();
    if (pid) updatePidTunings();
    if (rtx) applyRtxConfiguration(is_on_backup);
    if (vtx) vtxInit();
    usb_cdc.println("OK: set");
  }
}

void parseCli() {
  static char line_buf[128];
  static size_t line_pos = 0;
  while (usb_cdc.available() > 0) {
    int ch = usb_cdc.read();
    if (ch < 0) break;
    char c = (char)ch;
    if (c == '\r' || c == '\n') {
      line_buf[line_pos] = '\0';
      if (line_pos > 0) executeCliCommand(line_buf);
      line_pos = 0;
    } else if (line_pos < (sizeof(line_buf) - 1)) line_buf[line_pos++] = c;
    else line_pos = 0;
  }
}

int main() {
  SystemInit();

  Reg::RCC->AHB1ENR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 21) | (1U << 22);
  Reg::RCC->APB1ENR |= (1U << 0) | (1U << 14) | (1U << 21);
  Reg::RCC->APB2ENR |= (1U << 4) | (1U << 5) | (1U << 8) | (1U << 11);

  usb_cdc.begin(SERIAL_BAUD_RATE);
  delay(50);

  loadSettings();
  setupHardwareTimers();
  initAdc1Dma();
  initI2c1();
  SensorDriver::autoDetectAll();

  main_rtx_chan = active_rtx_chan = (uint8_t)constrainI(currentConfig.rtx_pri_chan, 0, 99);
  is_on_backup = false;
  rtx_init_hardware();
  applyRtxConfiguration(false);
  vtxInit();
  CliAutoCal::init();
  applyFilterConfig();
  last_rc_packet_ms = millis();

  uint32_t last_loop_us = micros();
  while (true) {
    UsbCdcDriver::poll();
    uint32_t now_us = micros();
    if ((now_us - last_loop_us) >= LOOP_INTERVAL_US) {
      last_loop_us = now_us;
      uint32_t now_ms = millis();

      parseCli();
      CliAutoCal::pollButton(now_ms);
      stepMagCalibrationNonBlocking(now_ms);
      stepBenchTestNonBlocking(now_ms);
      stepMonitorStreamNonBlocking(now_ms);
      serviceError202Event(now_ms);
      updateAuxSensorsNonBlocking(now_ms);

      flightControlTask();

      pollTelemetryTxState();
      if (currentConfig.rtx_en && !rtx_tx_in_flight && !rtx_recovering) {
        if (rtx_scan_active) stepChannelScanNonBlocking();
        else {
          readRx();
          if (!rtx_scan_active && !rtx_tx_in_flight && (send_telemetry_pending || hasQueuedRtxResponse())) startTelemetryNonBlocking();
        }
      }

      vtxStepNonBlocking(now_ms);
      UsbCdcDriver::poll();

      uint32_t el_us = micros() - now_us;
      if (el_us > (LOOP_INTERVAL_US + LOOP_OVERRUN_MARGIN_US)) setFcErrorRam(FC_ERR_LOOP_OVERRUN);
      float cur_load = ((float)el_us / (float)LOOP_INTERVAL_US) * 100.0f;
      if (isfinite(cur_load)) {
        cpu_load_pct = 0.85f * cpu_load_pct + 0.15f * cur_load;
        if (cur_load > cpu_max_load_pct) cpu_max_load_pct = cur_load;
      }
    }
  }
}
