/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_DMA_BUFFER_LEN 2
#define ADC_AVERAGING_CNT 4

#define SENSOR_POLL_PERIOD_MS_DEFAULT 1000
#define SENSOR_POLL_PERIOD_MS_SHORT 100
#define SENSOR_POLL_PERIOD_MS_LONG 2000
#define SENSOR_POLL_PERIOD_MS_EXTRA_LONG 5000
#define SENSOR_POLL_PERIOD_MS SENSOR_POLL_PERIOD_MS_DEFAULT

#define Heartbeat_LED_Pin         GPIO_PIN_13
#define Heartbeat_LED_GPIO_Port   GPIOC
#define Buzzer_Pin                GPIO_PIN_15
#define Buzzer_GPIO_Port          GPIOA
#define ST7735_CS_Pin             GPIO_PIN_4
#define ST7735_CS_GPIO_Port       GPIOA
#define ST7735_DC_A0_Pin          GPIO_PIN_6
#define ST7735_DC_A0_GPIO_Port    GPIOA
#define ST7735_Reset_Pin          GPIO_PIN_0
#define ST7735_Reset_GPIO_Port    GPIOB
#define Flowmeter_input_Pin       GPIO_PIN_3
#define Flowmeter_input_GPIO_Port GPIOA
#define Flowmeter_input_EXTI_IRQn EXTI3_IRQn
#define Encoder_button_Pin        GPIO_PIN_6
#define Encoder_button_GPIO_Port  GPIOB
#define Encoder_button_EXTI_IRQn  EXTI9_5_IRQn

#define BUZZER_ON 0
#define BUZZER_OFF 1
#define BUZZER_STARTUP 0
#define BUZZER_DURATION 100

#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "cmsis_os.h"
#include "task.h"



#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_exti.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_flash.h"

// #include "ssd1306_conf.h"
// #include "ssd1306_fonts.h"
// #include "ssd1306.h"

#include "ST7735.h"
#include "fonts.h"
#include "encoder.h"
#include "daq_settings.h"
#include "daq_gui.h"

#include "OneWire.h"

void Error_Handler(void);
void split_float(float val_float, uint16_t *val_int, uint16_t *val_fract, uint8_t dop);

// Routines
void buzzer_on(void);
void buzzer_off(void);
void buzzer_startup(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
