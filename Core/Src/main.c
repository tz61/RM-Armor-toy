/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_MAX_LUMINANCE 256U
#define MODE4_LED_MAX_LUMINANCE 128U
#if (LED_MAX_LUMINANCE < 1U) || (LED_MAX_LUMINANCE > 256U)
#error "LED_MAX_LUMINANCE must be in the range 1..256"
#endif
#if (MODE4_LED_MAX_LUMINANCE < 1U) || (MODE4_LED_MAX_LUMINANCE > 256U)
#error "MODE4_LED_MAX_LUMINANCE must be in the range 1..256"
#endif
#define LED_SOLID_LEVEL ((uint8_t)(LED_MAX_LUMINANCE - 1U))
#define MODE4_LED_SOLID_LEVEL ((uint8_t)(MODE4_LED_MAX_LUMINANCE - 1U))

#define SK6812_LED_COUNT 16U

#define SK6812_SPI_HZ 4000000UL

#define SK6812_CODE_BITS 5U
#define SK6812_CODE_0 0x10U // 10000
#define SK6812_CODE_1 0x1EU // 11110

// SPI = 4 MHz，1 byte = 2 us
// 128 bytes = 256 us reset low，满足 >= 200 us
#define SK6812_RESET_BYTES 128U

// 8 LEDs * 24 bits * 5 SPI bits / 8 = 120 bytes
#define SK6812_DATA_BYTES                                                      \
  ((SK6812_LED_COUNT * 24U * SK6812_CODE_BITS + 7U) / 8U)

#define SK6812_TX_BYTES                                                        \
  (SK6812_RESET_BYTES + SK6812_DATA_BYTES + SK6812_RESET_BYTES)

#define APP_MODE_COUNT 4U
#define ADJUSTABLE_LUMINANCE_LEVEL_COUNT 9U
#define ADJUSTABLE_LUMINANCE_DEFAULT_INDEX 5U
#define KEY_DEBOUNCE_MS 180U
#define HIT_DEBOUNCE_MS 70U

#define BUZZER_BEEP_ON_MS 35U
#define BUZZER_BEEP_OFF_MS 35U
#define BUZZER_MAX_BEEPS 8U
#define BUZZER_BEEP_HZ 2000U

#define STARTUP_TONE_COUNT 3U
#define STARTUP_TONE_MS 140U
#define STARTUP_LAST_TONE_MS 180U
#define STARTUP_TONE_GAP_MS 28U

#define MODE3_TAP_EFFECT_MS 500U
#define MODE3_TAP_BLINK_MS 30U
#define FANCY_FRAME_MS 10U
#define STATIC_FRAME_MS 10U
#define SEDENTARY_FRAME_MS 10U

#define FANCY_ROWS 8U
#define FANCY_STAGE_FLOW_FRAMES 260UL
#define FANCY_STAGE_BULLET_FRAMES 230UL
#define FANCY_STAGE_PORTAL_FRAMES 240UL
#define FANCY_STAGE_HEART_FRAMES 220UL
#define FANCY_STAGE_GLITTER_FRAMES 220UL
#define FANCY_STAGE_SCANNER_FRAMES 210UL
#define FANCY_STAGE_RAIN_FRAMES 240UL
#define FANCY_STAGE_SPLIT_FRAMES 220UL
#define FANCY_STAGE_ORBIT_FRAMES 260UL
#define FANCY_STAGE_FIREWORK_FRAMES 240UL
#define FANCY_STAGE_LOVE_FRAMES 260UL
#define FANCY_STAGE_TOTAL_FRAMES                                             \
  (FANCY_STAGE_FLOW_FRAMES + FANCY_STAGE_BULLET_FRAMES +                     \
   FANCY_STAGE_PORTAL_FRAMES + FANCY_STAGE_HEART_FRAMES +                    \
   /*FANCY_STAGE_GLITTER_FRAMES + */FANCY_STAGE_SCANNER_FRAMES +                 \
   FANCY_STAGE_RAIN_FRAMES + FANCY_STAGE_SPLIT_FRAMES +                      \
   FANCY_STAGE_ORBIT_FRAMES + FANCY_STAGE_FIREWORK_FRAMES +                  \
   FANCY_STAGE_LOVE_FRAMES)

#define SEDENTARY_TOTAL_SECONDS (48UL * 60UL)
#define SEDENTARY_LED_SECONDS (SEDENTARY_TOTAL_SECONDS / SK6812_LED_COUNT)
#define SEDENTARY_PROGRESS_SHOW_MS 2000UL
#define SEDENTARY_DONE_BEEPS 5U
#define SEDENTARY_REMINDER_BEEP_INTERVAL_MS 5000UL
#define RTC_WAKEUP_COUNTER_1HZ 0U

typedef enum {
  APP_MODE_FANCY = 1U,
  APP_MODE_RED,
  APP_MODE_BLUE,
  APP_MODE_SEDENTARY,
} AppMode;

typedef struct {
  uint8_t beeps_left;
  uint8_t output_on;
  uint32_t next_change_ms;
} BuzzerState;

static uint8_t sk6812_rgb[SK6812_LED_COUNT][3]; // R, G, B
static uint8_t sk6812_tx_buf[SK6812_TX_BYTES];
static uint8_t active_led_solid_level = LED_SOLID_LEVEL;

static volatile uint8_t sk6812_dma_busy = 0;
static volatile uint8_t chg_mode_request = 0U;
static volatile uint8_t reset_cnt_request = 0U;
static volatile uint8_t hit_request = 0U;
static volatile uint32_t rtc_second_ticks = 0U;

static uint32_t last_chg_irq_ms = 0U;
static uint32_t last_reset_irq_ms = 0U;
static uint32_t last_hit_irq_ms = 0U;

static AppMode app_mode = APP_MODE_FANCY;
static BuzzerState buzzer = {0U, 0U, 0U};
static uint8_t adjustable_luminance_index = ADJUSTABLE_LUMINANCE_DEFAULT_INDEX;
static uint32_t next_led_update_ms = 0U;
static uint32_t mode3_phase = 0U;
static uint32_t tap_effect_started_ms = 0U;
static uint8_t tap_effect_active = 0U;
static uint32_t sedentary_seconds = 0U;
static uint8_t sedentary_reminder_active = 0U;
static uint32_t sedentary_progress_until_ms = 0U;
static uint32_t sedentary_next_beep_ms = 0U;

static uint8_t TimeReached(uint32_t now, uint32_t target) {
  return ((int32_t)(now - target) >= 0) ? 1U : 0U;
}

static uint8_t TimeElapsed(uint32_t now, uint32_t start, uint32_t period) {
  return ((uint32_t)(now - start) >= period) ? 1U : 0U;
}

static uint8_t ClampLed(uint8_t value) {
  return (value > active_led_solid_level) ? active_led_solid_level : value;
}

static uint8_t ClampAddLed(uint8_t value, uint8_t add) {
  uint16_t sum = (uint16_t)value + (uint16_t)add;
  return (sum > active_led_solid_level) ? active_led_solid_level : (uint8_t)sum;
}

static uint8_t LuminanceToSolidLevel(uint16_t luminance) {
  if (luminance >= 256U) {
    return 255U;
  }

  return (uint8_t)luminance;
}

static uint8_t AdjustableLuminanceSolidLevel(void) {
  return LuminanceToSolidLevel((uint16_t)(1U << adjustable_luminance_index));
}

static void App_UpdateActiveLedLimit(void) {
  if (app_mode == APP_MODE_SEDENTARY) {
    active_led_solid_level = MODE4_LED_SOLID_LEVEL;
  } else if ((app_mode == APP_MODE_FANCY) || (app_mode == APP_MODE_RED) ||
             (app_mode == APP_MODE_BLUE)) {
    active_led_solid_level = AdjustableLuminanceSolidLevel();
  } else {
    active_led_solid_level = LED_SOLID_LEVEL;
  }
}

static void App_CycleAdjustableLuminance(void) {
  adjustable_luminance_index++;

  if (adjustable_luminance_index >= ADJUSTABLE_LUMINANCE_LEVEL_COUNT) {
    adjustable_luminance_index = 0U;
  }

  App_UpdateActiveLedLimit();
  next_led_update_ms = 0U;
}

static uint8_t TakeRequest(volatile uint8_t *request) {
  uint8_t value;

  __disable_irq();
  value = *request;
  *request = 0U;
  __enable_irq();

  return value;
}

static uint32_t TakeRtcSecondTicks(void) {
  uint32_t ticks;

  __disable_irq();
  ticks = rtc_second_ticks;
  rtc_second_ticks = 0U;
  __enable_irq();

  return ticks;
}

static void Buzzer_SetFrequency(uint16_t frequency_hz) {
  uint32_t counter_hz;
  uint32_t period_counts;

  if (frequency_hz == 0U) {
    return;
  }

  counter_hz = HAL_RCC_GetPCLK1Freq() / (htim2.Init.Prescaler + 1U);
  period_counts = (counter_hz + ((uint32_t)frequency_hz / 2U)) /
                  (uint32_t)frequency_hz;

  if (period_counts < 2U) {
    period_counts = 2U;
  }

  htim2.Init.Period = period_counts - 1U;
  __HAL_TIM_SET_AUTORELOAD(&htim2, htim2.Init.Period);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period_counts / 2U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
}

static void Buzzer_SetOutput(uint8_t on) {
  if (on) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                          (uint32_t)((htim2.Init.Period + 1U) / 2U));
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  } else {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  }
}

static void Buzzer_Cancel(void) {
  buzzer.beeps_left = 0U;
  buzzer.output_on = 0U;
  buzzer.next_change_ms = 0U;
  Buzzer_SetOutput(0U);
}

static void Buzzer_RequestBeeps(uint8_t count) {
  if (count == 0U) {
    return;
  }

  if (count > BUZZER_MAX_BEEPS) {
    count = BUZZER_MAX_BEEPS;
  }

  Buzzer_SetOutput(0U);
  Buzzer_SetFrequency(BUZZER_BEEP_HZ);
  buzzer.beeps_left = count;
  buzzer.output_on = 0U;
  buzzer.next_change_ms = HAL_GetTick();
}

static void Buzzer_Task(uint32_t now) {
  if (buzzer.output_on) {
    if (TimeReached(now, buzzer.next_change_ms)) {
      Buzzer_SetOutput(0U);
      buzzer.output_on = 0U;
      buzzer.next_change_ms = now + BUZZER_BEEP_OFF_MS;
    }

    return;
  }

  if ((buzzer.beeps_left > 0U) && TimeReached(now, buzzer.next_change_ms)) {
    Buzzer_SetOutput(1U);
    buzzer.output_on = 1U;
    buzzer.beeps_left--;
    buzzer.next_change_ms = now + BUZZER_BEEP_ON_MS;
  }
}

static uint8_t Buzzer_IsIdle(void) {
  return ((buzzer.beeps_left == 0U) && (buzzer.output_on == 0U)) ? 1U : 0U;
}

static void StartupSound_Play(void) {
  static const uint16_t tone_hz[STARTUP_TONE_COUNT] = {512U, 573U, 760U};

  Buzzer_Cancel();

  for (uint8_t i = 0U; i < STARTUP_TONE_COUNT; i++) {
    Buzzer_SetFrequency(tone_hz[i]);
    Buzzer_SetOutput(1U);
    HAL_Delay((i == (STARTUP_TONE_COUNT - 1U)) ? STARTUP_LAST_TONE_MS
                                               : STARTUP_TONE_MS);
    Buzzer_SetOutput(0U);

    if (i != (STARTUP_TONE_COUNT - 1U)) {
      HAL_Delay(STARTUP_TONE_GAP_MS);
    }
  }

  Buzzer_SetFrequency(BUZZER_BEEP_HZ);
  Buzzer_Cancel();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == CHG_MODE_Pin) {
    if ((last_chg_irq_ms == 0U) ||
        TimeElapsed(now, last_chg_irq_ms, KEY_DEBOUNCE_MS)) {
      last_chg_irq_ms = now;
      chg_mode_request = 1U;
    }
  } else if (GPIO_Pin == RST_COUNTER_Pin) {
    if ((last_reset_irq_ms == 0U) ||
        TimeElapsed(now, last_reset_irq_ms, KEY_DEBOUNCE_MS)) {
      last_reset_irq_ms = now;
      reset_cnt_request = 1U;
    }
  } else if (GPIO_Pin == HIT_TRIG_Pin) {
    if ((last_hit_irq_ms == 0U) ||
        TimeElapsed(now, last_hit_irq_ms, HIT_DEBOUNCE_MS)) {
      last_hit_irq_ms = now;
      hit_request = 1U;
    }
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
  if (hrtc->Instance == RTC) {
    rtc_second_ticks++;
  }
}
static void SK6812_WriteEncodedBit(uint16_t *bit_pos, uint8_t bit_is_one) {
  uint8_t code = bit_is_one ? SK6812_CODE_1 : SK6812_CODE_0;
  // 5-bit code, MSB first
  for (int k = 4; k >= 0; k--) {
    if (code & (1U << k)) {
      sk6812_tx_buf[*bit_pos >> 3] |= (uint8_t)(0x80U >> (*bit_pos & 0x07U));
    }

    (*bit_pos)++;
  }
}
static void SK6812_WriteEncodedByte(uint16_t *bit_pos, uint8_t value) {
  for (int b = 7; b >= 0; b--) {
    SK6812_WriteEncodedBit(bit_pos, (value & (1U << b)) != 0U);
  }
}
void SK6812_SetRGB(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  if ((i == 0U) || (i > SK6812_LED_COUNT)) {
    return;
  }
  i--; // User passes 1..16, array uses 0..15.
  sk6812_rgb[i][0] = ClampLed(r);
  sk6812_rgb[i][1] = ClampLed(g);
  sk6812_rgb[i][2] = ClampLed(b);
}
static void SK6812_SetAllRGB(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 1U; i <= SK6812_LED_COUNT; i++) {
    SK6812_SetRGB(i, r, g, b);
  }
}
HAL_StatusTypeDef SK6812_Show(void) {
  if (sk6812_dma_busy) {
    return HAL_BUSY;
  }
  memset(sk6812_tx_buf, 0x00, sizeof(sk6812_tx_buf));

  uint16_t bit_pos = SK6812_RESET_BYTES * 8U;

  for (uint8_t i = 0; i < SK6812_LED_COUNT; i++) {
    // SK6812-EC20 datasheet in GRB
    SK6812_WriteEncodedByte(&bit_pos, sk6812_rgb[i][1]); // G
    SK6812_WriteEncodedByte(&bit_pos, sk6812_rgb[i][0]); // R
    SK6812_WriteEncodedByte(&bit_pos, sk6812_rgb[i][2]); // B
  }

  sk6812_dma_busy = 1U;

  HAL_StatusTypeDef ret = HAL_SPI_Transmit_DMA(&hspi1, sk6812_tx_buf,
                                               (uint16_t)sizeof(sk6812_tx_buf));

  if (ret != HAL_OK) {
    sk6812_dma_busy = 0U;
  }

  return ret;
}
uint8_t SK6812_IsBusy(void) { return sk6812_dma_busy; }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    sk6812_dma_busy = 0U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    sk6812_dma_busy = 0U;
  }
}

static uint8_t Triangle8(uint8_t x) {
  return (x < 128U) ? (uint8_t)(x << 1U) : (uint8_t)((255U - x) << 1U);
}

static uint8_t Scale8(uint8_t value, uint8_t scale) {
  return (uint8_t)(((uint16_t)value * (uint16_t)scale) / 255U);
}

static uint8_t ClampScale(uint16_t scale) {
  return (scale > 255U) ? 255U : (uint8_t)scale;
}

static uint8_t LevelToScale(uint8_t level) {
  return ClampScale((uint16_t)level * 2U);
}

static uint8_t LevelToScaleMax(uint8_t level, uint8_t max_level) {
  if (max_level == 0U) {
    return 0U;
  }

  return (uint8_t)(((uint16_t)level * 255U) / max_level);
}

static void ColorWheelMax(uint8_t pos, uint8_t max_level, uint8_t *r,
                          uint8_t *g, uint8_t *b) {
  uint16_t max = max_level;

  pos = (uint8_t)(255U - pos);

  if (pos < 85U) {
    *r = (uint8_t)(max - (((uint16_t)pos * max) / 85U));
    *g = 0U;
    *b = (uint8_t)(((uint16_t)pos * max) / 85U);
  } else if (pos < 170U) {
    pos = (uint8_t)(pos - 85U);
    *r = 0U;
    *g = (uint8_t)(((uint16_t)pos * max) / 85U);
    *b = (uint8_t)(max - (((uint16_t)pos * max) / 85U));
  } else {
    pos = (uint8_t)(pos - 170U);
    *r = (uint8_t)(((uint16_t)pos * max) / 85U);
    *g = (uint8_t)(max - (((uint16_t)pos * max) / 85U));
    *b = 0U;
  }
}

static void ColorWheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b) {
  ColorWheelMax(pos, active_led_solid_level, r, g, b);
}

static uint8_t BreathLevel(uint32_t now, uint8_t max_level) {
  uint32_t phase = now % 2200U;

  if (phase < 1100U) {
    return (uint8_t)(1U + ((phase * max_level) / 1100U));
  }

  phase = 2199U - phase;
  return (uint8_t)(1U + ((phase * max_level) / 1100U));
}

static uint8_t FancyLedAt(uint8_t row, uint8_t right_side) {
  if (right_side) {
    return (uint8_t)(SK6812_LED_COUNT - row);
  }

  return (uint8_t)(row + 1U);
}

static void FancySetPixel(uint8_t row, uint8_t right_side, uint8_t r,
                          uint8_t g, uint8_t b) {
  if (row >= FANCY_ROWS) {
    return;
  }

  SK6812_SetRGB(FancyLedAt(row, right_side), r, g, b);
}

static void FancySetRow(uint8_t row, uint8_t r, uint8_t g, uint8_t b) {
  FancySetPixel(row, 0U, r, g, b);
  FancySetPixel(row, 1U, r, g, b);
}

static uint8_t AbsDiffU8(uint8_t a, uint8_t b) {
  return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

static uint8_t FancyNoise8(uint8_t seed, uint32_t tick) {
  uint32_t x = tick + ((uint32_t)seed * 0x45d9f3bUL) + 0x9e3779b9UL;

  x ^= x >> 16U;
  x *= 0x7feb352dUL;
  x ^= x >> 15U;

  return (uint8_t)(x >> 24U);
}

static void FancyRenderFlow(uint32_t t) {
  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t shimmer;

    ColorWheel((uint8_t)((t * 2U) + ((uint32_t)row * 24U)), &r, &g, &b);
    shimmer = (uint8_t)(170U + (Triangle8((uint8_t)((t * 5U) +
                                                    ((uint32_t)row * 28U))) >>
                                2U));
    FancySetPixel(row, 0U, Scale8(r, shimmer), Scale8(g, shimmer),
                  Scale8(b, shimmer));

    ColorWheel((uint8_t)((t * 2U) + ((uint32_t)(FANCY_ROWS - 1U - row) * 24U) +
                         80U),
               &r, &g, &b);
    shimmer = (uint8_t)(170U + (Triangle8((uint8_t)((t * 5U) +
                                                    ((uint32_t)row * 31U))) >>
                                2U));
    FancySetPixel(row, 1U, Scale8(r, shimmer), Scale8(g, shimmer),
                  Scale8(b, shimmer));
  }
}

static void FancyRenderBullets(uint32_t t) {
  uint8_t shot = (uint8_t)((t / 10U) % (FANCY_ROWS + 4U));

  SK6812_SetAllRGB(0U, 0U, 6U);

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t distance = AbsDiffU8(row, shot);

    if (distance < 4U) {
      uint8_t level = (uint8_t)(LED_SOLID_LEVEL - ((uint16_t)distance * 30U));
      FancySetPixel(row, 0U, level, (uint8_t)(level >> 2U), 0U);
    }

    if ((shot >= 1U) && (shot < (FANCY_ROWS + 1U))) {
      uint8_t hit_row = (uint8_t)(shot - 1U);
      uint8_t hit_distance = AbsDiffU8(row, hit_row);

      if (hit_distance < 3U) {
        uint8_t burst = (uint8_t)(LED_SOLID_LEVEL -
                                  ((uint16_t)hit_distance * 38U));
        FancySetPixel(row, 1U, burst, burst, (uint8_t)(burst >> 3U));
      }
    }
  }
}

static void FancyRenderPortal(uint32_t t) {
  uint8_t radius = (uint8_t)((t / 11U) % 9U);
  uint8_t hue = (uint8_t)(t * 3U);

  SK6812_SetAllRGB(0U, 0U, 0U);

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t d0 = AbsDiffU8(row, 3U);
    uint8_t d1 = AbsDiffU8(row, 4U);
    uint8_t distance = (d0 < d1) ? d0 : d1;

    if (distance <= radius) {
      uint8_t edge = AbsDiffU8(distance, radius);
      uint8_t level = (edge == 0U) ? LED_SOLID_LEVEL : 42U;

      ColorWheel((uint8_t)(hue + ((uint32_t)row * 18U)), &r, &g, &b);
      r = Scale8(r, LevelToScale(level));
      g = Scale8(g, LevelToScale(level));
      b = Scale8(b, LevelToScale(level));
      FancySetRow(row, r, g, b);
    }
  }
}

static void FancyRenderHeart(uint32_t t) {
  uint8_t beat = Triangle8((uint8_t)(t * 4U));

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t shape = (uint8_t)(LED_SOLID_LEVEL -
                              ((uint16_t)row * (LED_SOLID_LEVEL / 9U)));
    uint8_t glow = ClampAddLed(Scale8(shape, ClampScale(130U + (beat >> 1U))),
                               (uint8_t)(beat >> 3U));

    FancySetRow(row, glow, (uint8_t)(glow >> 4U), (uint8_t)(glow >> 2U));
  }
}

static void __attribute__((unused)) FancyRenderGlitter(uint32_t t) {
  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t base = (uint8_t)(18U + (Triangle8((uint8_t)(t * 2U + row * 21U)) >>
                                    4U));

    ColorWheel((uint8_t)(t + row * 29U), &r, &g, &b);
    FancySetRow(row, Scale8(r, base), Scale8(g, base), Scale8(b, base));

    for (uint8_t side = 0U; side < 2U; side++) {
      uint8_t noise = FancyNoise8((uint8_t)(row + (side * 13U)), t / 3U);

      if (noise > 232U) {
        FancySetPixel(row, side, LED_SOLID_LEVEL, LED_SOLID_LEVEL,
                      LED_SOLID_LEVEL);
      } else if (noise > 216U) {
        FancySetPixel(row, side, LED_SOLID_LEVEL, 0U, LED_SOLID_LEVEL);
      }
    }
  }
}

static void FancyRenderScanner(uint32_t t) {
  uint8_t pos = (uint8_t)((t / 6U) % ((FANCY_ROWS * 2U) - 2U));
  uint8_t center =
      (pos < FANCY_ROWS) ? pos : (uint8_t)(((FANCY_ROWS * 2U) - 2U) - pos);

  SK6812_SetAllRGB(0U, 0U, 10U);

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t distance = AbsDiffU8(row, center);

    if (distance < 4U) {
      uint8_t level = (uint8_t)(LED_SOLID_LEVEL -
                                ((uint16_t)distance * 34U));
      FancySetPixel(row, 0U, level, 0U, (uint8_t)(level >> 1U));
      FancySetPixel(row, 1U, 0U, (uint8_t)(level >> 1U), level);
    }
  }
}

static void FancyRenderRain(uint32_t t) {
  SK6812_SetAllRGB(0U, 0U, 0U);

  for (uint8_t side = 0U; side < 2U; side++) {
    for (uint8_t drop = 0U; drop < 4U; drop++) {
      uint8_t head = (uint8_t)((t / 7U + ((uint32_t)drop * 4U) +
                                ((uint32_t)side * 2U)) %
                               (FANCY_ROWS + 5U));
      uint8_t hue = (uint8_t)(150U + (drop * 18U) + (side * 50U));

      for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
        if (head >= row) {
          uint8_t r;
          uint8_t g;
          uint8_t b;
          uint8_t tail = (uint8_t)(head - row);

          if (tail < 4U) {
            uint8_t level = (uint8_t)(LED_SOLID_LEVEL -
                                      ((uint16_t)tail * 34U));

            ColorWheel((uint8_t)(hue + row * 9U), &r, &g, &b);
            FancySetPixel(row, side, Scale8(r, LevelToScale(level)),
                          Scale8(g, LevelToScale(level)),
                          Scale8(b, LevelToScale(level)));
          }
        }
      }
    }
  }
}

static void FancyRenderSplit(uint32_t t) {
  uint8_t radius = (uint8_t)((t / 8U) % FANCY_ROWS);
  uint8_t hue = (uint8_t)(t * 4U);

  SK6812_SetAllRGB(3U, 0U, 5U);

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t d0 = AbsDiffU8(row, 3U);
    uint8_t d1 = AbsDiffU8(row, 4U);
    uint8_t distance = (d0 < d1) ? d0 : d1;
    uint8_t error = AbsDiffU8(distance, radius);

    if (error < 3U) {
      uint8_t level = (uint8_t)(LED_SOLID_LEVEL -
                                ((uint16_t)error * 40U));

      ColorWheel((uint8_t)(hue + row * 20U), &r, &g, &b);
      FancySetRow(row, Scale8(r, LevelToScale(level)),
                  Scale8(g, LevelToScale(level)),
                  Scale8(b, LevelToScale(level)));
    }
  }
}

static void FancyRenderOrbit(uint32_t t) {
  uint8_t head = (uint8_t)((t / 5U) % SK6812_LED_COUNT);

  SK6812_SetAllRGB(0U, 4U, 6U);

  for (uint8_t trail = 0U; trail < 6U; trail++) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t step = (uint8_t)((head + SK6812_LED_COUNT - trail) %
                             SK6812_LED_COUNT);
    uint8_t row = (step < FANCY_ROWS) ? step : (uint8_t)(15U - step);
    uint8_t side = (step < FANCY_ROWS) ? 0U : 1U;
    uint8_t level = (uint8_t)(LED_SOLID_LEVEL -
                              ((uint16_t)trail * 19U));

    ColorWheel((uint8_t)(t * 3U + trail * 18U), &r, &g, &b);
    FancySetPixel(row, side, Scale8(r, LevelToScale(level)),
                  Scale8(g, LevelToScale(level)),
                  Scale8(b, LevelToScale(level)));
  }
}

static void FancyRenderFirework(uint32_t t) {
  uint32_t shot = t / 48U;
  uint8_t age = (uint8_t)(t % 48U);
  uint8_t center_row = (uint8_t)(FancyNoise8(61U, shot) % FANCY_ROWS);
  uint8_t center_side = (uint8_t)(FancyNoise8(97U, shot) & 1U);
  uint8_t radius = (uint8_t)(age / 8U);
  uint8_t hue = FancyNoise8(131U, shot);

  SK6812_SetAllRGB(0U, 0U, 0U);

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    for (uint8_t side = 0U; side < 2U; side++) {
      uint8_t r;
      uint8_t g;
      uint8_t b;
      uint8_t distance =
          (uint8_t)(AbsDiffU8(row, center_row) + ((side == center_side) ? 0U : 1U));
      uint8_t error = AbsDiffU8(distance, radius);

      if (error < 2U) {
        uint8_t level = (error == 0U) ? LED_SOLID_LEVEL : 54U;
        uint8_t fade = (uint8_t)(255U - ((uint16_t)age * 3U));

        ColorWheel((uint8_t)(hue + row * 13U + side * 35U), &r, &g, &b);
        level = Scale8(level, fade);
        FancySetPixel(row, side, Scale8(r, LevelToScale(level)),
                      Scale8(g, LevelToScale(level)),
                      Scale8(b, LevelToScale(level)));
      }
    }
  }
}

static void FancyRenderLove(uint32_t t) {
  static const uint8_t heart_shape[FANCY_ROWS] = {58U, 112U, 127U, 120U,
                                                  96U, 70U,  44U,  22U};
  uint8_t pulse = Triangle8((uint8_t)(t * 3U));

  for (uint8_t row = 0U; row < FANCY_ROWS; row++) {
    uint8_t glow =
        ClampAddLed(Scale8(heart_shape[row], ClampScale(160U + (pulse >> 2U))),
                    (uint8_t)(pulse >> 4U));

    FancySetRow(row, glow, (uint8_t)(glow >> 5U), (uint8_t)(glow >> 2U));
  }

  if (((t / 18U) & 1UL) == 0UL) {
    FancySetPixel(1U, 0U, LED_SOLID_LEVEL, LED_SOLID_LEVEL, LED_SOLID_LEVEL);
    FancySetPixel(1U, 1U, LED_SOLID_LEVEL, LED_SOLID_LEVEL, LED_SOLID_LEVEL);
  }

  if (((t / 31U) & 1UL) == 0UL) {
    FancySetPixel(5U, 0U, LED_SOLID_LEVEL, 20U, LED_SOLID_LEVEL);
    FancySetPixel(5U, 1U, LED_SOLID_LEVEL, 20U, LED_SOLID_LEVEL);
  }
}

static void RenderModeRed(uint32_t now) {
  if (tap_effect_active) {
    uint32_t age = now - tap_effect_started_ms;

    if (age >= MODE3_TAP_EFFECT_MS) {
      tap_effect_active = 0U;
    } else if (((age / MODE3_TAP_BLINK_MS) & 1UL) == 0UL) {
      SK6812_SetAllRGB(0U, 0U, 0U);
      return;
    }
  }
  SK6812_SetAllRGB(LED_SOLID_LEVEL, 0U, 0U);
}

static void RenderModeBlue(uint32_t now) {
  if (tap_effect_active) {
    uint32_t age = now - tap_effect_started_ms;

    if (age >= MODE3_TAP_EFFECT_MS) {
      tap_effect_active = 0U;
    } else if (((age / MODE3_TAP_BLINK_MS) & 1UL) == 0UL) {
      SK6812_SetAllRGB(0U, 0U, 0U);
      return;
    }
  }

  SK6812_SetAllRGB(0U, 0U, LED_SOLID_LEVEL);
}

static void RenderModeFancy(void) {
  uint32_t stage_time = mode3_phase % FANCY_STAGE_TOTAL_FRAMES;

  if (stage_time < FANCY_STAGE_FLOW_FRAMES) {
    FancyRenderFlow(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_FLOW_FRAMES;
  if (stage_time < FANCY_STAGE_BULLET_FRAMES) {
    FancyRenderBullets(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_BULLET_FRAMES;
  if (stage_time < FANCY_STAGE_PORTAL_FRAMES) {
    FancyRenderPortal(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_PORTAL_FRAMES;
  if (stage_time < FANCY_STAGE_HEART_FRAMES) {
    FancyRenderHeart(stage_time);
    mode3_phase++;
    return;
  }

  // stage_time -= FANCY_STAGE_HEART_FRAMES;
  // if (stage_time < FANCY_STAGE_GLITTER_FRAMES) {
  //   FancyRenderGlitter(stage_time);
  //   mode3_phase++;
  //   return;
  // }

  stage_time -= FANCY_STAGE_GLITTER_FRAMES;
  if (stage_time < FANCY_STAGE_SCANNER_FRAMES) {
    FancyRenderScanner(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_SCANNER_FRAMES;
  if (stage_time < FANCY_STAGE_RAIN_FRAMES) {
    FancyRenderRain(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_RAIN_FRAMES;
  if (stage_time < FANCY_STAGE_SPLIT_FRAMES) {
    FancyRenderSplit(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_SPLIT_FRAMES;
  if (stage_time < FANCY_STAGE_ORBIT_FRAMES) {
    FancyRenderOrbit(stage_time);
    mode3_phase++;
    return;
  }

  stage_time -= FANCY_STAGE_ORBIT_FRAMES;
  if (stage_time < FANCY_STAGE_FIREWORK_FRAMES) {
    FancyRenderFirework(stage_time);
  } else {
    stage_time -= FANCY_STAGE_FIREWORK_FRAMES;
    FancyRenderLove(stage_time);
  }

  mode3_phase++;
}

static void RenderModeSedentary(uint32_t now) {
  if (sedentary_reminder_active) {
    uint8_t white = BreathLevel(now, MODE4_LED_SOLID_LEVEL);
    SK6812_SetAllRGB(white, white, white);
    return;
  }

  if (TimeReached(now, sedentary_progress_until_ms)) {
    SK6812_SetAllRGB(0U, 0U, 0U);
    return;
  }

  for (uint8_t i = 0U; i < SK6812_LED_COUNT; i++) {
    uint32_t led_start = (uint32_t)i * SEDENTARY_LED_SECONDS;
    uint32_t led_end = led_start + SEDENTARY_LED_SECONDS;
    uint8_t level = 0U;
    uint8_t scale;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (sedentary_seconds >= led_end) {
      level = MODE4_LED_SOLID_LEVEL;
    } else if (sedentary_seconds >= led_start) {
      uint32_t elapsed_in_led = sedentary_seconds - led_start;
      level = (uint8_t)((elapsed_in_led * MODE4_LED_SOLID_LEVEL) /
                        SEDENTARY_LED_SECONDS);
    }

    ColorWheelMax((uint8_t)((uint32_t)i * 16U), MODE4_LED_SOLID_LEVEL, &r, &g,
                  &b);
    scale = LevelToScaleMax(level, MODE4_LED_SOLID_LEVEL);
    SK6812_SetRGB((uint8_t)(i + 1U), Scale8(r, scale), Scale8(g, scale),
                  Scale8(b, scale));
  }
}

static uint32_t CurrentFramePeriod(void) {
  if (app_mode == APP_MODE_FANCY) {
    return (tap_effect_active != 0U) ? MODE3_TAP_BLINK_MS : STATIC_FRAME_MS;
  }

  if (app_mode == APP_MODE_BLUE) {
    return (tap_effect_active != 0U) ? MODE3_TAP_BLINK_MS : STATIC_FRAME_MS;
  }

  if (app_mode == APP_MODE_SEDENTARY) {
    return SEDENTARY_FRAME_MS;
  }

  return STATIC_FRAME_MS;
}

static void Sedentary_Reset(void) {
  sedentary_seconds = 0U;
  sedentary_reminder_active = 0U;
  sedentary_progress_until_ms = 0U;
  sedentary_next_beep_ms = 0U;
  next_led_update_ms = 0U;
}

static void Sedentary_ReminderBeepTask(uint32_t now) {
  if ((app_mode != APP_MODE_SEDENTARY) || (sedentary_reminder_active == 0U)) {
    return;
  }

  if (TimeReached(now, sedentary_next_beep_ms) && Buzzer_IsIdle()) {
    Buzzer_RequestBeeps(SEDENTARY_DONE_BEEPS);
    sedentary_next_beep_ms = now + SEDENTARY_REMINDER_BEEP_INTERVAL_MS;
  }
}

static void App_SetMode(AppMode mode) {
  app_mode = mode;
  tap_effect_active = 0U;
  App_UpdateActiveLedLimit();
  next_led_update_ms = 0U;

  if (app_mode == APP_MODE_SEDENTARY) {
    Sedentary_Reset();
  }

  Buzzer_RequestBeeps((uint8_t)app_mode);
}

static void App_AdvanceMode(void) {
  uint8_t next = (uint8_t)app_mode + 1U;

  if (next > APP_MODE_COUNT) {
    next = 1U;
  }

  App_SetMode((AppMode)next);
}

static void App_HandleEvents(uint32_t now) {
  uint32_t ticks = TakeRtcSecondTicks();
  uint8_t counter_was_reset = 0U;

  if (TakeRequest(&chg_mode_request)) {
    App_AdvanceMode();
    counter_was_reset = (app_mode == APP_MODE_SEDENTARY) ? 1U : 0U;
  }

  if (TakeRequest(&reset_cnt_request)) {
    if (app_mode == APP_MODE_SEDENTARY) {
      uint8_t was_reminding = sedentary_reminder_active;
      Sedentary_Reset();
      counter_was_reset = 1U;
      if (was_reminding) {
        Buzzer_Cancel();
      }
    } else if ((app_mode == APP_MODE_FANCY) || (app_mode == APP_MODE_RED) ||
               (app_mode == APP_MODE_BLUE)) {
      App_CycleAdjustableLuminance();
    }
  }

  if (TakeRequest(&hit_request)) {
    if (app_mode == APP_MODE_BLUE || app_mode == APP_MODE_RED) {
      tap_effect_started_ms = now;
      tap_effect_active = 1U;
      next_led_update_ms = 0U;
    } else if ((app_mode == APP_MODE_SEDENTARY) &&
               (sedentary_reminder_active == 0U)) {
      sedentary_progress_until_ms = now + SEDENTARY_PROGRESS_SHOW_MS;
      next_led_update_ms = 0U;
    }
  }

  if ((counter_was_reset == 0U) && (ticks > 0U) &&
      (app_mode == APP_MODE_SEDENTARY) &&
      (sedentary_reminder_active == 0U)) {
    if (sedentary_seconds + ticks >= SEDENTARY_TOTAL_SECONDS) {
      sedentary_seconds = SEDENTARY_TOTAL_SECONDS;
      sedentary_reminder_active = 1U;
      sedentary_next_beep_ms = now;
      next_led_update_ms = 0U;
    } else {
      sedentary_seconds += ticks;
    }
  }

  Sedentary_ReminderBeepTask(now);
}

static void App_Render(uint32_t now) {
  if (!TimeReached(now, next_led_update_ms) || SK6812_IsBusy()) {
    return;
  }

  next_led_update_ms = now + CurrentFramePeriod();

  if (app_mode == APP_MODE_RED) {
    RenderModeRed(now);
  } else if (app_mode == APP_MODE_BLUE) {
    RenderModeBlue(now);
  } else if (app_mode == APP_MODE_FANCY) {
    RenderModeFancy();
  } else {
    RenderModeSedentary(now);
  }

  SK6812_Show();
}

static void App_Init(void) {
  Buzzer_Cancel();
  App_UpdateActiveLedLimit();
  RenderModeFancy();
  SK6812_Show();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  StartupSound_Play();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t now = HAL_GetTick();

    App_HandleEvents(now);
    Buzzer_Task(now);
    App_Render(now);
    HAL_Delay(1);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }

  /** Enable the WakeUp
   */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, RTC_WAKEUP_COUNTER_1HZ,
                                  RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : CHG_MODE_Pin RST_COUNTER_Pin */
  GPIO_InitStruct.Pin = CHG_MODE_Pin | RST_COUNTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HIT_TRIG_Pin */
  GPIO_InitStruct.Pin = HIT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HIT_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
