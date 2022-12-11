# STM32 DAQ 001
Simple DAQ based on STM32F103.

## Features
* FreeRTOS
* USB VCP
* ST7735 SPI display
* Simple GUI
* OneWire digital thermometer (DS18B20)
* ADC (CH0, CH1, periodic (TIM4)) for reading two thermocouple amplifiers
* Thermocouple
* Flow meter input (pulse counter, IT)
* Settings (int. flash) - In progress, currently RAM only

### Humble notice
This is not a final version, some functions may not work properly.