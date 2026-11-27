/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : USB CDC "Hello" every 2s + onboard LED blink 1s/1s (C++)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software component, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern "C" {
#include "usb_device.h"
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdint>
#include <cstring>

extern "C" {
#include "usbd_cdc_if.h"
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
namespace {

constexpr uint32_t kCdcSendIntervalMs  = 2000U;
constexpr uint32_t kCdcTxTimeoutMs     = 50U;
constexpr uint32_t kLedBlinkIntervalMs = 1000U;
constexpr bool     kLedActiveLow       = true;
constexpr char     kHelloMessage[]     = "Hello\r\n";

#ifndef LED_GPIO_Port
constexpr GPIO_TypeDef *kLedPort = GPIOC;
#else
constexpr GPIO_TypeDef *kLedPort = LED_GPIO_Port;
#endif

#ifndef LED_Pin
constexpr uint16_t kLedPin = GPIO_PIN_13;
#else
constexpr uint16_t kLedPin = LED_Pin;
#endif

}  // namespace
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
namespace {

class BlinkLed {
public:
  BlinkLed(GPIO_TypeDef *port, uint16_t pin, bool active_low, uint32_t interval_ms)
      : port_(port),
        pin_(pin),
        active_low_(active_low),
        interval_ms_(interval_ms),
        last_tick_ms_(0U),
        is_on_(false) {}

  void init(uint32_t now_ms) {
    last_tick_ms_ = now_ms;
    set(false);
  }

  void set(bool on) {
    is_on_ = on;
    const GPIO_PinState state = resolvePinState(on);
    HAL_GPIO_WritePin(port_, pin_, state);
  }

  void update(uint32_t now_ms) {
    if ((now_ms - last_tick_ms_) >= interval_ms_) {
      last_tick_ms_ = now_ms;
      set(!is_on_);
    }
  }

private:
  GPIO_PinState resolvePinState(bool on) const {
    if (active_low_) {
      return on ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }

  GPIO_TypeDef *port_;
  uint16_t pin_;
  bool active_low_;
  uint32_t interval_ms_;
  uint32_t last_tick_ms_;
  bool is_on_;
};

class UsbCdcPeriodicSender {
public:
  UsbCdcPeriodicSender(const char *message, uint32_t interval_ms, uint32_t timeout_ms)
      : message_(message),
        interval_ms_(interval_ms),
        timeout_ms_(timeout_ms),
        last_tick_ms_(0U) {}

  void init(uint32_t now_ms) {
    last_tick_ms_ = now_ms;
  }

  bool sendNow(const char *str) const {
    if (str == nullptr) {
      return false;
    }

    const std::size_t len = std::strlen(str);
    if ((len == 0U) || (len > UINT16_MAX)) {
      return false;
    }

    const uint32_t start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < timeout_ms_) {
      const uint8_t status = CDC_Transmit_FS(
          reinterpret_cast<uint8_t *>(const_cast<char *>(str)),
          static_cast<uint16_t>(len));

      if (status == USBD_OK) {
        return true;
      }
      if (status == USBD_FAIL) {
        return false;
      }
    }

    return false;
  }

  void update(uint32_t now_ms) {
    if ((now_ms - last_tick_ms_) >= interval_ms_) {
      last_tick_ms_ = now_ms;
      (void)sendNow(message_);
    }
  }

private:
  const char *message_;
  uint32_t interval_ms_;
  uint32_t timeout_ms_;
  uint32_t last_tick_ms_;
};

BlinkLed g_led(kLedPort, kLedPin, kLedActiveLow, kLedBlinkIntervalMs);
UsbCdcPeriodicSender g_cdc_sender(kHelloMessage, kCdcSendIntervalMs, kCdcTxTimeoutMs);

}  // namespace
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
}
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  const uint32_t init_tick = HAL_GetTick();
  g_led.init(init_tick);
  g_cdc_sender.init(init_tick);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    const uint32_t now_ms = HAL_GetTick();
    g_led.update(now_ms);
    g_cdc_sender.update(now_ms);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLL_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(kLedPort, kLedPin, kLedActiveLow ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = kLedPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(kLedPort, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
