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
#include "dacs_j1939.h"
#include "motor_control.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t min_mv;  /* конструкционный минимум (мВ датчика) */
  uint16_t max_mv;  /* конструкционный максимум (мВ датчика) */
  uint16_t last_mv; /* мВ на момент последнего значимого сдвига */
  uint16_t
      last_target_mv; /* прошлая цель — для сброса fault по смене команды */
  uint32_t stall_ms;  /* мс последнего значимого сдвига */
  uint8_t is_moving;
  uint8_t fault;
} Flap_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Заслонки */
#define FLAP_DEADBAND_MV 80U        /* зона нечувствительности к цели, мВ */
#define FLAP_RAIL_MV 20U            /* обрыв/КЗ датчика (по АЦП) */
#define FLAP_MOTION_DELTA_MV 25U    /* значимый сдвиг, мВ */
#define FLAP_STALL_TIMEOUT_MS 1500U /* нет движения дольше — застревание */

/* Вентилятор */
#define FAN_PWM_THRESHOLD 50U       /* 5% от 999 */
#define FAN_ERROR_TIMEOUT_MS 15000U /* 15 с */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
uint16_t sensor_zasl[4];
/* sensor_zasl[0] - заслонка рециркуляции (Rank 1, PA0)
 * sensor_zasl[1] - заслонка грудь        (Rank 2, PA5)
 * sensor_zasl[2] - заслонка стекло       (Rank 3, PA6)
 * sensor_zasl[3] - заслонка ноги         (Rank 4, PC2) */

volatile uint32_t g_ms_ticks = 0; /* 1 мс от TIM6 */
volatile uint32_t tacho_pulses = 0;
volatile uint16_t fun_speed = 0;
uint32_t fan_error_timer = 0;
uint8_t fan_is_fault = 0;

uint16_t fun_pwm = 0; /* текущий ЗАПРОС ШИМ (0…999), не выход */
uint32_t last_tacho_time = 0;
volatile uint8_t can_bus_off_flag = 0;

/* Конструкционные границы — пересчитаны из проверенных АЦП старого
 * рабочего кода (raw * 3300/4095 * 5000/2960):
 *   recirc: 1240..1940 АЦП  -> 1688..2641 мВ
 *   chest :  935..2055 АЦП  -> 1273..2797 мВ
 *   legs  :  935..2055 АЦП  -> 1273..2797 мВ
 *   window:  935..1980 АЦП  -> 1273..2695 мВ
 * Если на конкретном железе пределы отличаются — поправьте эти числа. */
Flap_t flap_recirc = {.min_mv = 1120, .max_mv = 2700};
Flap_t flap_chest = {.min_mv = 1750, .max_mv = 3300};
Flap_t flap_window = {.min_mv = 1750, .max_mv = 3300};
Flap_t flap_legs = {.min_mv = 1750, .max_mv = 3300};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static inline uint32_t ms_now(void) { return g_ms_ticks; }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Read_All_4_Sensors(void) {
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    for (uint8_t i = 0; i < 4; i++) {
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint16_t raw_val = (uint16_t)HAL_ADC_GetValue(&hadc1);
        if (sensor_zasl[i] == 0)
          sensor_zasl[i] = raw_val;
        else
          sensor_zasl[i] = (sensor_zasl[i] * 3 + raw_val) / 4;
      }
    }
    HAL_ADC_Stop(&hadc1);
  }
}

/* Пересчёт АЦП -> мВ датчика (0…5000). */
static inline uint32_t adc_to_sensor_mv(uint16_t raw_adc) {
  uint32_t mv_adc = (raw_adc * 3300U) / 4095U;
  return (mv_adc * 5000U) / 2960U;
}

/* Логика заслонки:
 *  - цель = min_mv или max_mv;
 *  - детекция застревания = нет значимого сдвига дольше FLAP_STALL_TIMEOUT_MS;
 *  - без обучения границ, без стриков;
 *  - fault сбрасывается при смене целевой позиции. */
void Flap_Update(Flap_t *flap, uint16_t raw_adc, uint8_t req_mode,
                 void (*motor_open)(void), void (*motor_close)(void),
                 void (*motor_stop)(void), void (*set_dacs_status)(uint8_t)) {
  uint32_t now = ms_now();

  uint32_t mv_adc = (raw_adc * 3300U) / 4095U;
  uint32_t mv_sensor = adc_to_sensor_mv(raw_adc);

  /* 1. Цель по команде */
  uint16_t target_mv;
  if (req_mode == DACS_ST_ON)
    target_mv = flap->min_mv;
  else if (req_mode == DACS_ST_OFF)
    target_mv = flap->max_mv;
  else {
    /* Не ON и не OFF — стоим, статус пробрасываем как есть */
    motor_stop();
    set_dacs_status(req_mode);
    flap->is_moving = 0;
    flap->fault = 0;
    flap->last_target_mv = 0;
    return;
  }

  /* 2. Смена целевой позиции — сбрасываем fault и пробуем снова */
  if (abs((int)target_mv - (int)flap->last_target_mv) > (int)FLAP_DEADBAND_MV) {
    flap->fault = 0;
    flap->is_moving = 0;
  }
  flap->last_target_mv = target_mv;

  /* 3. Обрыв/КЗ датчика */
  if (mv_adc < FLAP_RAIL_MV || mv_adc > (2960U - FLAP_RAIL_MV)) {
    motor_stop();
    flap->is_moving = 0;
    flap->fault = 1;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  /* 4. Уже в fault — стоим, пока не придёт другая цель */
  if (flap->fault) {
    motor_stop();
    flap->is_moving = 0;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  /* 5. Доехали до цели */
  int diff = (int)mv_sensor - (int)target_mv;
  if (abs(diff) <= (int)FLAP_DEADBAND_MV) {
    motor_stop();
    flap->is_moving = 0;
    set_dacs_status(req_mode);
    return;
  }

  /* 6. Старт движения */
  if (!flap->is_moving) {
    flap->is_moving = 1;
    flap->last_mv = (uint16_t)mv_sensor;
    flap->stall_ms = now;
  }

  /* 7. Крутим в нужную сторону */
  if (mv_sensor < target_mv)
    motor_open();
  else
    motor_close();

  /* 8. Детектор застревания: если нет значимого сдвига
        дольше FLAP_STALL_TIMEOUT_MS — стоп + fault. */
  int delta = abs((int)mv_sensor - (int)flap->last_mv);
  if (delta >= (int)FLAP_MOTION_DELTA_MV) {
    flap->last_mv = (uint16_t)mv_sensor;
    flap->stall_ms = now;
  } else if ((now - flap->stall_ms) > FLAP_STALL_TIMEOUT_MS) {
    motor_stop();
    flap->is_moving = 0;
    flap->fault = 1;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  set_dacs_status(DACS_ST_NA);
}

void Flap_Update_Percent(Flap_t *flap, uint16_t raw_adc, uint8_t percent,
                         void (*motor_open)(void), void (*motor_close)(void),
                         void (*motor_stop)(void),
                         void (*set_dacs_status)(uint8_t)) {
  if (percent > 100U)
    percent = 100U;
  uint32_t now = ms_now();

  uint32_t mv_adc = (raw_adc * 3300U) / 4095U;
  uint32_t mv_sensor = adc_to_sensor_mv(raw_adc);

  uint16_t target_mv =
      (uint16_t)((uint32_t)flap->min_mv +
                 ((uint32_t)(flap->max_mv - flap->min_mv) * percent) / 100U);

  /* Смена цели — сбрасываем fault */
  if (abs((int)target_mv - (int)flap->last_target_mv) > (int)FLAP_DEADBAND_MV) {
    flap->fault = 0;
    flap->is_moving = 0;
  }
  flap->last_target_mv = target_mv;

  /* Обрыв/КЗ датчика */
  if (mv_adc < FLAP_RAIL_MV || mv_adc > (2960U - FLAP_RAIL_MV)) {
    motor_stop();
    flap->is_moving = 0;
    flap->fault = 1;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  if (flap->fault) {
    motor_stop();
    flap->is_moving = 0;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  int diff = (int)mv_sensor - (int)target_mv;
  if (abs(diff) <= (int)FLAP_DEADBAND_MV) {
    motor_stop();
    flap->is_moving = 0;
    set_dacs_status(DACS_ST_ON);
    return;
  }

  if (!flap->is_moving) {
    flap->is_moving = 1;
    flap->last_mv = (uint16_t)mv_sensor;
    flap->stall_ms = now;
  }

  if (mv_sensor < target_mv)
    motor_open();
  else
    motor_close();

  int delta = abs((int)mv_sensor - (int)flap->last_mv);
  if (delta >= (int)FLAP_MOTION_DELTA_MV) {
    flap->last_mv = (uint16_t)mv_sensor;
    flap->stall_ms = now;
  } else if ((now - flap->stall_ms) > FLAP_STALL_TIMEOUT_MS) {
    motor_stop();
    flap->is_moving = 0;
    flap->fault = 1;
    set_dacs_status(DACS_ST_ERROR);
    return;
  }

  set_dacs_status(DACS_ST_NA);
}

void FanSpeed_Read(void) {
  uint32_t now = ms_now();
  if ((now - last_tacho_time) >= 1000U) {
    last_tacho_time = now;
    __disable_irq();
    uint32_t current_pulses = tacho_pulses;
    tacho_pulses = 0;
    __enable_irq();
    fun_speed = (uint16_t)(current_pulses * 15U); /* 4 имп/об */
  }
}

/* Логика вентилятора:
 *  - если подан ШИМ (>5%) и тахо = 0 дольше 15 с → fault;
 *  - в fault ШИМ уходит в 0, в CAN шлём 0xFE;
 *  - fault держится, пока CCU не снизит запрос ШИМ ≤ 5% (mode=OFF или
 * fanSpeed=0).
 *  - fun_pwm — это ЗАПРОС, при срабатывании fault его НЕ обнуляем. */
void Fan_Diagnostics(void) {
  uint32_t now = ms_now();

  if (fan_is_fault) {
    /* Снимаем ошибку только когда запрос ШИМ упал до нуля (CCU выключил) */
    if (fun_pwm <= FAN_PWM_THRESHOLD) {
      fan_is_fault = 0;
      fan_error_timer = now;
    }
    return;
  }

  if (fun_pwm > FAN_PWM_THRESHOLD) {
    if (fun_speed == 0) {
      if ((now - fan_error_timer) > FAN_ERROR_TIMEOUT_MS) {
        fan_is_fault = 1;
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0); /* аппаратно в 0 */
        DACS_SetFanSpeedRaw(0xFE);
        /* fun_pwm НЕ трогаем — это запрос */
      }
    } else {
      fan_error_timer = now;
    }
  } else {
    fan_error_timer = now;
  }
}

void Fan_Control(uint8_t mode, uint8_t funSpeed) {
  if (mode == DACS_MODE_VENTILATION || mode == DACS_MODE_HEATING ||
      mode == DACS_MODE_PTC_HEATING || mode == DACS_MODE_PTC) {
    DACS_SetAirflowMode(mode);

    if (mode == DACS_MODE_PTC_HEATING) {
      if (funSpeed < 75) {
        funSpeed = 75;
      }
    }

    if (fan_is_fault) {
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
      DACS_SetFanSpeedRaw(0xFE);
      return;
    }

    if (funSpeed != 0xFF && funSpeed != 0xFE) {

      fun_pwm = (uint16_t)(funSpeed * 4U);
      if (fun_pwm > 999U)
        fun_pwm = 999U;
      __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, fun_pwm);
      /* По протоколу J1939 (см. PDF, SPN 2.1.1) поле FanSpeed должно
       * передаваться как raw (0..250 при масштабе 0.4%/bit). Но CCU в этой
       * системе интерпретирует приходящий байт КАК ПРОЦЕНТ напрямую (0..100).
       */
      DACS_SetFanSpeedRaw(funSpeed * 0.4f);
    }
  } else {
    DACS_SetAirflowMode(DACS_MODE_OFF);
    DACS_SetFanSpeedRaw(0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    fun_pwm = 0; /* вот здесь fault сбросится в следующей итерации */
  }
}

void PWM_Init(void) {
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
}

void Power_5V(bool state) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Power_12V(bool state) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Contactor_ON(void) {
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void Contactor_OFF(void) {
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

/* Обёртки под Flap_Update. Индексы: 0=recirc, 1=chest, 2=window, 3=legs. */
void Recir_control(uint8_t mode) {
  Flap_Update(&flap_recirc, sensor_zasl[0], mode, Motor_open_recir,
              Motor_closed_recir, Motor_stop_recir, DACS_SetRecirculation);
}

void Chest_control(uint8_t mode) {
  Flap_Update(&flap_chest, sensor_zasl[1], mode, Motor_open_chest,
              Motor_closed_chest, Motor_stop_chest, DACS_SetChest);
}

void Window_control(uint8_t mode) {
  Flap_Update(&flap_window, sensor_zasl[2], mode, Motor_open_window,
              Motor_closed_window, Motor_stop_window, DACS_SetWindshield);
}

void Window_percent_control(uint8_t percent) {
  Flap_Update_Percent(&flap_window, sensor_zasl[2], percent, Motor_open_window,
                      Motor_closed_window, Motor_stop_window,
                      DACS_SetWindshield);
}

void Legs_control(uint8_t mode) {
  Flap_Update(&flap_legs, sensor_zasl[3], mode, Motor_open_legs,
              Motor_closed_legs, Motor_stop_legs, DACS_SetLegs);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  SCB->VTOR = 0x08008000U;
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  if (DACS_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }

  PWM_Init();
  Power_5V(true);
  Power_12V(true);

  /* Запуск TIM6 — 1 мс тик */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }

  last_tacho_time = ms_now();
  fan_error_timer = ms_now();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* 1. АЦП */
    Read_All_4_Sensors();

    /* 2. Восстановление после CAN Bus-Off */
    if (can_bus_off_flag) {
      can_bus_off_flag = 0;
      if (DACS_ReinitAfterBusOff() != HAL_OK) {
        can_bus_off_flag = 1; /* повторим на следующей итерации */
      }
    }

    /* 3. CAN J1939: RX-опрос + периодический TX */
    DACS_Task();

    static uint32_t s_last_can_mon = 0;
    if ((ms_now() - s_last_can_mon) >= 100U) {
      s_last_can_mon = ms_now();
      uint8_t tec = 0, rec = 0;
      if (DACS_GetErrorCounters(&tec, &rec)) {
        /* TEC/REC можно логировать или использовать для доп. диагностики.
           Выход в bus-off всё равно поймается через can_bus_off_flag. */
        (void)tec;
        (void)rec;
      }
    }

    /* 4. Обработка последнего запроса от CCU */
    const DACS_Request *req = DACS_GetLastRequest();
    if (req->received) {

      if (req->airflowMode == DACS_MODE_OFF ||
          req->airflowMode == DACS_MODE_ERROR) {
        /* Всё выключено — закрываем заслонки */
        Chest_control(DACS_ST_OFF);
        Window_control(DACS_ST_OFF);
        Legs_control(DACS_ST_OFF);

      } else if (req->airflowMode == DACS_MODE_HEATING ||
                 req->airflowMode == DACS_MODE_PTC_HEATING ||
                 req->airflowMode == DACS_MODE_VENTILATION || DACS_MODE_PTC) {

        if (req->chest == DACS_ST_OFF && req->legs == DACS_ST_OFF &&
            req->windshield == DACS_ST_ON) {
          Chest_control(DACS_ST_OFF);
          Legs_control(DACS_ST_OFF);
          Window_control(DACS_ST_ON);

        } else if (req->chest == DACS_ST_OFF && req->legs == DACS_ST_OFF &&
                   req->windshield == DACS_ST_OFF) {
          Chest_control(DACS_ST_OFF);
          Legs_control(DACS_ST_OFF);
          Window_control(DACS_ST_OFF);

        } else if (req->chest == DACS_ST_ON && req->legs == DACS_ST_OFF) {
          Chest_control(DACS_ST_ON);
          Legs_control(DACS_ST_OFF);
          Window_control(DACS_ST_OFF);

        } else if (req->chest == DACS_ST_OFF && req->legs == DACS_ST_ON) {
          Chest_control(DACS_ST_OFF);
          Legs_control(DACS_ST_ON);
          Window_percent_control(50);

        } else {
          Chest_control(DACS_ST_ON);
          Legs_control(DACS_ST_ON);
          Window_percent_control(50);
        }
      }

      Recir_control(req->recirc);

      /* Вентилятор */

      Fan_Control(req->airflowMode, req->fanSpeedRaw);

      /* Контактор PTC — включаем только при вращении вентилятора */
      if (req->airflowMode == DACS_MODE_PTC_HEATING ||
          req->airflowMode == DACS_MODE_PTC) {
        if (fun_speed > 0)
          Contactor_ON();
        else
          Contactor_OFF();
      } else {
        Contactor_OFF();
      }
    }

    /* 5. Диагностика вентилятора — по ms_now() */
    FanSpeed_Read();
    Fan_Diagnostics();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 */
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN1 Initialization Function
 */
static void MX_CAN1_Init(void) {
  /* Безопасная инициализация: деинициализация переводит пины в аналоговый
     режим (HAL_CAN_MspDeInit), разрывая связь с трансивером на время Init. */
  if (hcan1.State != HAL_CAN_STATE_RESET) {
    (void)HAL_CAN_DeInit(&hcan1);
  }
  HAL_Delay(1);

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ; /* 500 kbps при PCLK1=42 МГц */
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE; /* восстановление — программное */
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN2 Initialization Function
 */
static void MX_CAN2_Init(void) {
  if (hcan2.State != HAL_CAN_STATE_RESET) {
    (void)HAL_CAN_DeInit(&hcan2);
  }
  HAL_Delay(1);

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM6 Initialization Function
 */
static void MX_TIM6_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM9 Initialization Function
 */
static void MX_TIM9_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 167;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim9);
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOD,
                    GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 |
                        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(
      GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
      GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12 | GPIO_PIN_13 |
                        GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 |
                        GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 1 мс от TIM6 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    g_ms_ticks++;
  }
}

/* Таходатчик вентилятора */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_1) {
    tacho_pulses++;
  }
}

/* CAN Bus-Off */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN2) {
    if (HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_BOF) {
      can_bus_off_flag = 1;
    }
    /* Warning / Error Passive — можно логировать, действий не требуется */
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */