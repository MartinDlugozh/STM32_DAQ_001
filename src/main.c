/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * This firmware is designed to run on STM32F103C8T6 (BluePill board).
  * 
  * Pinout:
  * 
  * PA0 - ADC0                - Thermocouple 1
  * PA1 - ADC1                - Thermocouple 2
  * PA2 - USART2              - DS18B20 OneWire Tx/Rx
  * PA3 - EXTI3               - Flowmeter pulse input (falling edge)
  * PA4 - SPI1 CS             - ST7735 - CS
  * PA5 - SPI1 SCK            - ST7735 - SCK
  * PA6 - GPIO                - ST7735 - DC/A0
  * PA7 - SPI1 MOSI           - ST7735 - MOSI
  * PB0 - GPIO                - ST7735 - RESET
  * 
  * PA11 - USB DM             - VCP
  * PA12 - USB DP             - VCP
  * 
  * PA13 - SWDIO              - debug
  * PA14 - SWCLK              - debug
  * 
  * PA15 - GPIO               - Active BUZZER - pnp driving pin
  * 
  * PB4 - TIM3 CH1            - Encoder - CLK
  * PB5 - TIM3 CH2            - Encoder - DT 
  * PB6 - GPIO                - Encoder - SW
  *
  ******************************************************************************
  */

#include "main.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;

// #define I2C_MAX_ADDRESS 127     // max address value for 12c scanner
// #define I2C_MAX_DEVICES 5       // max amount of registered i2c devices

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE]; // "extern" due to previous definition in "usbd_cdc_if.c"
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern view_t gui_current_view;
extern daq_flash_settings_t settings;

uint32_t            adc_dma_buffer[ADC_DMA_BUFFER_LEN]; // this buffer is used to store conversion results from both ADC channels

volatile uint32_t   timer_sec = 0;
uint8_t             buzzer_state = BUZZER_OFF;
volatile uint32_t   buzzer_timer = 0;
extern Temperature  t;
int                 onewire_status = 0xff;
volatile uint16_t   pulse_counter = 0;
uint16_t            pps = 0;        // pulses per second (for the flowmeter)

float               temperature_1 = 0;
float               temperature_2 = 0;
float               flow_rate = 0;

// Hardware handles
SPI_HandleTypeDef   hspi1;          // SPI1 - ST7735 display
DMA_HandleTypeDef   hdma_spi1_tx;   // SPI1 DMA
UART_HandleTypeDef  huart2;         // UART2 - DS18B20 OneWire
ADC_HandleTypeDef   hadc1;          // ADC for analog trmperature sensors
DMA_HandleTypeDef   hdma_adc1;      // ADC DMA
TIM_HandleTypeDef   htim3;          // Encoder timer
TIM_HandleTypeDef   htim4;          // Timer to triggrt ADC conversion 

// I2C_HandleTypeDef hi2c1;

// uint8_t i2c_address[I2C_MAX_DEVICES];
// uint8_t i2c_selected_device = 0;

// OS task attributes
osThreadId_t osTaskHeartbeat_handle;
uint32_t heartbeatTaskBuffer[128];
osStaticThreadDef_t heartbeatTaskControlBlock;
const osThreadAttr_t osTaskHeartbeat_attributes = {
  .name = "hb",
  .cb_mem = &heartbeatTaskControlBlock,
  .cb_size = sizeof(heartbeatTaskControlBlock),
  .stack_mem = &heartbeatTaskBuffer[0],
  .stack_size = sizeof(heartbeatTaskBuffer),
  .priority = (osPriority_t) (configMAX_PRIORITIES-2),
};

osThreadId_t osTaskSensorPoll_handle;
uint32_t sensorPollTaskBuffer[160];
osStaticThreadDef_t sensorPollTaskControlBlock;
const osThreadAttr_t osTaskSensorPoll_attributes = {
  .name = "sen",
  .cb_mem = &sensorPollTaskControlBlock,
  .cb_size = sizeof(sensorPollTaskControlBlock),
  .stack_mem = &sensorPollTaskBuffer[0],
  .stack_size = sizeof(sensorPollTaskBuffer),
  .priority = (osPriority_t) (configMAX_PRIORITIES-3),
};

osThreadId_t osTaskUSB_handle;
uint32_t usbTaskBuffer[160];
osStaticThreadDef_t usbTaskControlBlock;
const osThreadAttr_t osTaskUSB_attributes = {
  .name = "usb",
  .cb_mem = &usbTaskControlBlock,
  .cb_size = sizeof(usbTaskControlBlock),
  .stack_mem = &usbTaskBuffer[0],
  .stack_size = sizeof(usbTaskBuffer),
  .priority = (osPriority_t) (configMAX_PRIORITIES-1),
};

osThreadId_t osTaskDisplay_handle;
uint32_t displayTaskBuffer[160];
osStaticThreadDef_t displayTaskControlBlock;
const osThreadAttr_t osTaskDisplay_attributes = {
  .name = "disp",
  .cb_mem = &displayTaskControlBlock,
  .cb_size = sizeof(displayTaskControlBlock),
  .stack_mem = &displayTaskBuffer[0],
  .stack_size = sizeof(displayTaskBuffer),
  .priority = (osPriority_t) (configMAX_PRIORITIES-4),
};


// SYS config
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);
static void buzzer_GPIO_init(void);
// static void MX_I2C1_Init(void);

// Tasks
void osTaskHeartbeat(void *argument);
void osTaskSensorPoll(void *argument);
void osTaskUSB(void *argument);
void osTaskDisplay(void *argument);
void vApplicationIdleHook(void);

int main(void)
{
  // SYSTEM INIT
  HAL_Init();             // 1. Initialize core library
  SystemClock_Config();   // 2. Initialize system clock
  MX_DMA_Init();          // 3. DMA should be initialized before everything else that could use it

  MX_USB_DEVICE_Init();   // 4. USB also needs to be initialized asap 
  MX_NVIC_Init();         // 5. This was named by CubeMX as NVIC_Init but there is only initialization for USB rx interrupts

  MX_GPIO_Init();         // Initialize GPIO
  buzzer_GPIO_init();
  buzzer_startup();

  ST7735_Unselect();      // Initialize main display - unselect first
  MX_SPI1_Init();         // then initialize SPI module; everything else will be done inside the gui task
  MX_TIM3_Init();         // Initialize encoder timer (encoder is part of hmi, so it goes hre)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 

  // SENSORS INIT
  MX_USART2_UART_Init();  // Initialize usart2 - for OneWire DS18B20
  
  MX_ADC1_Init();         // Initialize adc1 ch1 and ch2 - for thermocouples
  MX_TIM4_Init();         // Initialize adc triggering timer
 
  // MISC
  // SSD1306 test
  // MX_I2C1_Init();         // I2C for secondary display
  // ssd1306_Init();
  // SSD1306 test

  // daq_settings_init();

  // OS INIT
  osKernelInitialize();
  osTaskHeartbeat_handle =    osThreadNew(osTaskHeartbeat,  NULL, &osTaskHeartbeat_attributes);
  osTaskSensorPoll_handle =   osThreadNew(osTaskSensorPoll, NULL, &osTaskSensorPoll_attributes);
  osTaskUSB_handle =          osThreadNew(osTaskUSB,        NULL, &osTaskUSB_attributes);
  osTaskDisplay_handle =      osThreadNew(osTaskDisplay,    NULL, &osTaskDisplay_attributes);
  osKernelStart();

  while (1)
  {

  }
}

void osTaskHeartbeat(void *argument)
{
  volatile uint32_t osTaskPreviousWoken = 0;
  uint8_t hb_counter = 0; // can do other low level stuf here

  for(;;)
  {
    osTaskPreviousWoken = osKernelGetTickCount();
    
    if(hb_counter<10){ // divide by 10
      hb_counter++;
      // do something with 100ms interval 
      buzzer_off();
    }else{
      hb_counter = 0;
      timer_sec = osTaskPreviousWoken/1000;
      HAL_GPIO_TogglePin(Heartbeat_LED_GPIO_Port, Heartbeat_LED_Pin);
    }    
    
    osDelay(100 - (osKernelGetTickCount() - osTaskPreviousWoken));
  }
}

void osTaskSensorPoll(void *argument){
  volatile uint32_t osTaskPreviousWoken = 0;

  daq_settings_init();

  onewire_status = get_ROMid();   // connect with DS18B20 (reference temperature sensor)
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_LEN); // start ADC for thermocouples 
  HAL_TIM_Base_Start_IT(&htim4);                                            // and it's timer
  
  for(;;){
    osTaskPreviousWoken = osKernelGetTickCount();

    if(onewire_status == 0){          // if previous attempt to connect to DS18B20 was successful
      get_Temperature();              // read the temperature
    }else{                            // if not
      onewire_status = get_ROMid();   // then try harder
    }

    // pps = pulse_counter/(SENSOR_POLL_PERIOD_MS/1000);
    pps = pulse_counter; // only if SENSOR_POLL_PERIOD_MS is 1 second
    pulse_counter = 0;
  
    flow_rate     = pps*settings.k_f;
    
    osDelayUntil(osTaskPreviousWoken + SENSOR_POLL_PERIOD_MS); // 1Hz (test) 
  }
}

void osTaskUSB(void *argument){
  volatile uint32_t osTaskPreviousWoken = 0;

  uint8_t cdc_tx_len = 0;
  // volatile UBaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // (void)uxHighWaterMark;

  for(;;){
    osTaskPreviousWoken = osKernelGetTickCount();
    
    memset(UserTxBufferFS, 0x00, APP_TX_DATA_SIZE);
    // S - time, [seconds * 10^(-2)]
    // C - reference temperature [degC]
    // A - ADC0 (thermocouple 1)
    // B - ADC1 (thermocouple 2)
    // P - flow [pulses per second]
    cdc_tx_len = sprintf((char*)UserTxBufferFS, "S%luC%d.%dA%dB%dP%d\r\n", 
                                                (uint32_t)(osKernelGetTickCount()/100), 
                                                t.inCelsus, t.frac, 
                                                (uint16_t)adc_dma_buffer[0], 
                                                (uint16_t)adc_dma_buffer[1],
                                                pps);
    CDC_Transmit_FS(UserTxBufferFS, cdc_tx_len);

    osDelayUntil(osTaskPreviousWoken + 100); // 10Hz (test) 

    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // (void)uxHighWaterMark;
  }
}

void osTaskDisplay(void *argument){
  // volatile UBaseType_t uxHighWaterMark;
  // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // (void)uxHighWaterMark;

  volatile uint32_t osTaskPreviousWoken = 0;
  
  uint8_t gui_redraw = 0; // don't need to redraw entire gui in every iteration, just numbers
  
  ST7735_Init();  // display initialization should be inside the task because it relies on 
  buzzer_off();   // taskYIELD() while waiting for the end of transmossion

  gui_current_view = VIEW_START;          // show boot screen
  daq_gui_draw_view(gui_current_view, 1);
  osDelay(1000);                          // boot screen delay
  gui_current_view = VIEW_MAIN;           // go to main screen
  daq_gui_draw_view(gui_current_view, 1);
  osDelay(500);

  // SSD1306 test
  // ssd1306_Fill(Black);
  // ssd1306_SetCursor(10, 5);
  // ssd1306_WriteString(" Hello,", Font_6x8, White);
  // ssd1306_SetCursor(10, 15);
  // ssd1306_WriteString("World =)", Font_6x8, White);
  // ssd1306_UpdateScreen();
  // SSD1306 test

  for(;;){
    osTaskPreviousWoken = osKernelGetTickCount();
    
    daq_gui_poll_encoder_sw();
    gui_redraw = daq_gui_poll_encoder_increment();
    
    daq_gui_update_cursor();
    daq_gui_draw_view(gui_current_view, gui_redraw);
    
    if(gui_redraw == 1){
      gui_redraw = 0;
      osDelay(500 - (osKernelGetTickCount() - osTaskPreviousWoken));
    }else{
      osDelay(200 - (osKernelGetTickCount() - osTaskPreviousWoken));
    }
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // (void)uxHighWaterMark;
  }
}

void vApplicationIdleHook(void){
  // encoder_update();
  // __DSB();  // sleep when in idle (non-tickless)
  // __WFI();
}

void buzzer_on(void){
  if(buzzer_state != BUZZER_ON){
    buzzer_timer = osKernelGetTickCount();
    buzzer_state = BUZZER_ON;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, buzzer_state);
  }
}

void buzzer_off(void){ // can be called from idle task
  if(buzzer_state != BUZZER_OFF){
    if((osKernelGetTickCount() - buzzer_timer) >= BUZZER_DURATION){
      buzzer_state = BUZZER_OFF;
      HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, buzzer_state);
    }
  }
}

void buzzer_startup(void){
  buzzer_state = BUZZER_STARTUP;
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, buzzer_state);
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_USB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 - Heartbeat LED*/
  GPIO_InitStruct.Pin = Heartbeat_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heartbeat_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Flowmeter_input_Pin */
  GPIO_InitStruct.Pin = Flowmeter_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Flowmeter_input_GPIO_Port, &GPIO_InitStruct);

  // USB
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Peripheral clock enable */
  
  /*Configure GPIO pins : ST7735_CS_Pin and ST7735_A0_Pin*/
  GPIO_InitStruct.Pin = ST7735_CS_Pin | ST7735_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7735_Reset_Pin  */
  GPIO_InitStruct.Pin = ST7735_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Encoder_button_Pin */
  GPIO_InitStruct.Pin = Encoder_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
  GPIO_InitStruct.Pull = GPIO_PULLUP; 
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(Encoder_button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init (flowmeter)*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* EXTI interrupt init (encoder sw)*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void buzzer_GPIO_init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, BUZZER_OFF);
  
  /*Configure GPIO pin : Active_Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel 0
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel 1
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4800-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // 24Mbits/s
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void split_float(float val_float, uint16_t *val_int, uint16_t *val_fract, uint8_t dop){
  uint16_t t_int = 0;
  uint16_t t_fract = 0;

  t_int = val_float;
  t_fract = (val_float - t_int)*10*dop; // dop can be 1, 10, 100, 1000 ....

  *val_int = t_int;
  *val_fract = t_fract;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

// static void MX_I2C1_Init(void)
// {
//   hi2c1.Instance = I2C1;
//   hi2c1.Init.ClockSpeed = 100000;   // 100kHz
//   hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//   hi2c1.Init.OwnAddress1 = 0;
//   hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//   hi2c1.Init.OwnAddress2 = 0;
//   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//   hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//   if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// SSD1306 test
// void scan_i2c_bus(void){
//   HAL_StatusTypeDef i2c_status;
//   uint8_t cdc_tx_len = 0;
//   for(uint8_t i = 0; i <= I2C_MAX_ADDRESS; i++){
//     i2c_status = HAL_I2C_IsDeviceReady(&hi2c1, (i<<1), 3, 100);

//     memset(UserTxBufferFS, 0x00, APP_TX_DATA_SIZE);

//     switch (i2c_status)
//     {
//     case HAL_OK:{
//       i2c_address[i2c_selected_device++] = i;
//       cdc_tx_len = sprintf((char*)UserTxBufferFS, "Device found at 0x%X\r\n", i);
//       // HAL_UART_Transmit(&huart1, uart_tx_buffer, uart_tx_len, (HMI_UPD_PERIOD_MS/10));
//       CDC_Transmit_FS(UserTxBufferFS, cdc_tx_len);
//       break;
//     }
//     case HAL_ERROR:{
//       cdc_tx_len = sprintf((char*)UserTxBufferFS, "0x%X - HAL error\r\n", i);
//       // HAL_UART_Transmit(&huart1, uart_tx_buffer, uart_tx_len, (HMI_UPD_PERIOD_MS/10));
//       CDC_Transmit_FS(UserTxBufferFS, cdc_tx_len);
//       break;
//     }
//     case HAL_BUSY:{
//       cdc_tx_len = sprintf((char*)UserTxBufferFS, "0x%X - HAL busy\r\n", i);
//       // HAL_UART_Transmit(&huart1, uart_tx_buffer, uart_tx_len, (HMI_UPD_PERIOD_MS/10));
//       CDC_Transmit_FS(UserTxBufferFS, cdc_tx_len);
//       // I2C1_ClearBusyFlagErratum(&hi2c1); // try to unlock
//       break;
//     }
//     case HAL_TIMEOUT:{
//       cdc_tx_len = sprintf((char*)UserTxBufferFS, "0x%X - HAL timeout\r\n", i);
//       // HAL_UART_Transmit(&huart1, uart_tx_buffer, uart_tx_len, (HMI_UPD_PERIOD_MS/10));
//       CDC_Transmit_FS(UserTxBufferFS, cdc_tx_len);
//       break;
//     }
//     osDelay(100);

//     default:
//       break;
//     }
//   }
// }
// SSD1306 test