#ifndef _DAQ_SETTINGS_H_
#define _DAQ_SETTINGS_H_

#include <stdint.h>
#include <string.h>
#include "main.h"

// #define USE_FLASH

#define SETTINGS_FLASH_BASE_ADDRESS 0x08010000 //0x08010000 0x0800fc00

#define SETTINGS_DEFAULT_K_F ((float)0.345)
#define SETTINGS_DEFAULT_K_T1 ((float)24.56)
#define SETTINGS_DEFAULT_K_T2 ((float)24.56)
#define SETTINGS_DEFAULT_DUMMY ((float)5.5)

#define SETTINGS_INCREMENT_K_F ((float)0.01)
#define SETTINGS_INCREMENT_K_T1 ((float)0.01)
#define SETTINGS_INCREMENT_K_T2 ((float)0.01)
#define SETTINGS_INCREMENT_DUMMY ((float)0.1)

#define NUM_SETTINGS 4
#define SETTINGS_WORDS (unsigned int)16U

typedef struct daq_settings{
  float k_t1;
  float k_t2;
  float k_f;
  float dummy;
}daq_flash_settings_t;

typedef enum param_status{
  STATUS_UNCHANGED = 0,
  STATUS_CHANGED
}param_status_t;

typedef struct daq_param_float{
  uint16_t value_int;   // integer part
  uint16_t value_frac;  // fractional part
  param_status_t status;
}daq_param_float_t;

typedef enum view_parameters{
  SET_SOL = 0,
  SET_K_T1,
  SET_K_T2,
  SET_K_F,
  SET_DUMMY,
  SET_SAVE,
  SET_BACK,
  SET_EOL
}view_parameters_t;

typedef enum param_id{
  ID_K_T1 = 0,    // use EXACTLY THE SAME parameter order as in view_parameters_t
  ID_K_T2,
  ID_K_F,
  ID_DUMMY
}param_id_t;

void daq_settings_init(void);
void daq_settings_set_default(void);

uint16_t daq_setings_get_param_int(param_id_t param_id);
uint16_t daq_setings_get_param_frac(param_id_t param_id);
param_status_t daq_setings_get_param_status(param_id_t param_id);
void daq_setings_set_param_status(param_id_t param_id, param_status_t status);
param_id_t daq_gui_param_view_to_id(view_parameters_t param_view);

void daq_setings_parameter_increment(view_parameters_t param_view, int16_t num_steps);
void daq_settings_save(void);

void daq_settings_read_flash(void);
void daq_settings_write_flash(void);
void daq_settings_status_reset(void);
void daq_settings_mem_to_front(void);
void daq_settings_flash_to_front(void);

#endif