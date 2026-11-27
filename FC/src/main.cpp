#include <stdint.h>

namespace {
constexpr uintptr_t kAhb1Base = 0x40020000UL;
constexpr uintptr_t kRccBase  = kAhb1Base + 0x3800UL;
constexpr uintptr_t kGpiocBase = kAhb1Base + 0x0800UL;

volatile uint32_t *const rcc_ahb1enr =
    reinterpret_cast<volatile uint32_t *>(kRccBase + 0x30UL);
volatile uint32_t *const gpioc_moder =
    reinterpret_cast<volatile uint32_t *>(kGpiocBase + 0x00UL);
volatile uint32_t *const gpioc_bsrr =
    reinterpret_cast<volatile uint32_t *>(kGpiocBase + 0x18UL);

void spinDelay(volatile uint32_t count) {
  while (count > 0U) {
    --count;
  }
}
}  // namespace

extern "C" void SystemInit(void) {
  // На старте ядро уже работает от внутреннего HSI 16 МГц.
}

int main(void) {
  *rcc_ahb1enr |= (1U << 2U);  // GPIOCEN

  *gpioc_moder &= ~(3U << (13U * 2U));
  *gpioc_moder |= (1U << (13U * 2U));  // PC13 output push-pull

  while (1) {
    *gpioc_bsrr = (1U << 13U);        // PC13 = 1
    spinDelay(1200000U);
    *gpioc_bsrr = (1U << (13U + 16U)); // PC13 = 0
    spinDelay(1200000U);
  }
}
