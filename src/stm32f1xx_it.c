/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

#include "main.h"
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "OneWire.h"
#include "daq_settings.h"
#include "encoder.h"

extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_adc1;

extern ADC_HandleTypeDef hadc1;
extern int32_t adc_dma_buffer[ADC_DMA_BUFFER_LEN];
extern float  temperature_1;
extern float  temperature_2;
float sum1 = 0;
float sum2 = 0;
uint8_t avg_cnt = 0;
extern uint16_t pulse_counter;
extern daq_flash_settings_t settings;


void NMI_Handler(void)
{

  while (1)
  {
  }
}

void HardFault_Handler(void)
{

  while (1)
  {

  }
}

void MemManage_Handler(void)
{

  while (1)
  {

  }
}

void BusFault_Handler(void)
{

  while (1)
  {

  }
}

void UsageFault_Handler(void)
{

  while (1)
  {

  }
}

void DebugMon_Handler(void)
{

}

void SysTick_Handler(void)
{
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
}

void DMA1_Channel1_IRQHandler(void)
{
  if(__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_IT_TC) != RESET){
    if(avg_cnt < ADC_AVERAGING_CNT){
      avg_cnt++;
      sum1 += ((uint16_t)adc_dma_buffer[0])/settings.k_t1;
      sum2 += ((uint16_t)adc_dma_buffer[1])/settings.k_t2;
    }else{
      avg_cnt = 0;
      temperature_1 = sum1/ADC_AVERAGING_CNT;
      temperature_2 = sum2/ADC_AVERAGING_CNT;
      sum1 = 0;
      sum2 = 0;
    }
  }

  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void EXTI3_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_FLAG(Flowmeter_input_Pin) != RESET){
    pulse_counter++;
  }

  HAL_GPIO_EXTI_IRQHandler(Flowmeter_input_Pin);
}

void EXTI9_5_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_FLAG(Encoder_button_Pin) != RESET){
    encoder_sw_callback();
  }

  HAL_GPIO_EXTI_IRQHandler(Encoder_button_Pin);
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//   if(GPIO_Pin == Encoder_button_Pin){
//     encoder_sw_callback();
//   }
// }

void TIM4_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC4) != RESET){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_LEN);
  }
  
  HAL_TIM_IRQHandler(&htim4);
}

void USART2_IRQHandler(void)
{
  owReadHandler();
  HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}