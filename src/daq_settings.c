#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "daq_settings.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

daq_flash_settings_t settings;                             // actual parameter values; can be used for calculations or stored in flash memory
daq_param_float_t settings_parameter[NUM_SETTINGS];        // temporary values for gui

uint32_t PAGEError = 0;
FLASH_EraseInitTypeDef EraseInitStruct;
FLASH_ProcessTypeDef pFlash;

void daq_settings_flash_init(void){
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.NbPages = 1;
    EraseInitStruct.PageAddress = SETTINGS_FLASH_BASE_ADDRESS;
}

void daq_settings_flash_erase(void){
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
        //Erase error!
    }
}

void daq_settings_set_default(void){ 
    settings.k_f = SETTINGS_DEFAULT_K_F;        // reset settings to default (hardcoded) values
    settings.k_t1 = SETTINGS_DEFAULT_K_T1;
    settings.k_t2 = SETTINGS_DEFAULT_K_T2;
    settings.dummy = SETTINGS_DEFAULT_DUMMY;
}

void daq_settings_read_flash(void){
    uint32_t *source_addr = (uint32_t *)SETTINGS_FLASH_BASE_ADDRESS;
    uint32_t *dest_addr = (void *)&settings;
    for (uint16_t i=0; i < SETTINGS_WORDS; i++) {
		*dest_addr = *(__IO uint32_t*)source_addr;
		source_addr++;
		dest_addr++;
	}
}

void daq_settings_write_flash(void){
    taskENTER_CRITICAL();       // prevent this code from interruptions (write settings only when os kernel is already running!)
    HAL_FLASH_Unlock();
    daq_settings_flash_erase();
    uint32_t *source_addr = (void *)&settings;
    uint32_t *dest_addr = (uint32_t *) SETTINGS_FLASH_BASE_ADDRESS;
    for (uint16_t i=0; i < SETTINGS_WORDS; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr);
		source_addr++;
		dest_addr++;
	}
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
}

void daq_settings_status_reset(void){
    settings_parameter[ID_K_T1].status  = STATUS_UNCHANGED;         // reset status after reading from or writing into flash memory
    settings_parameter[ID_K_T2].status  = STATUS_UNCHANGED;
    settings_parameter[ID_K_F].status   = STATUS_UNCHANGED;
    settings_parameter[ID_DUMMY].status = STATUS_UNCHANGED;
}

void daq_settings_mem_to_front(void){
    split_float(settings.k_t1, &settings_parameter[ID_K_T1].value_int, &settings_parameter[ID_K_T1].value_frac, 10);       // split float parameter values 
    split_float(settings.k_t2, &settings_parameter[ID_K_T2].value_int, &settings_parameter[ID_K_T2].value_frac, 10);       // into int and frac decimals
    split_float(settings.k_f, &settings_parameter[ID_K_F].value_int, &settings_parameter[ID_K_F].value_frac, 10);
    split_float(settings.dummy, &settings_parameter[ID_DUMMY].value_int, &settings_parameter[ID_DUMMY].value_frac, 10);
}

void daq_settings_flash_to_front(void){
    daq_settings_read_flash();                // read settings from flash
    daq_settings_status_reset();        // reset status to UNCHANGED
    daq_settings_mem_to_front();    // prepare for display
}

void daq_settings_init(void){
#ifdef USE_FLASH
    daq_settings_flash_init();
    daq_settings_read_flash();
    if(settings.dummy == SETTINGS_DEFAULT_DUMMY){

    }else{
        daq_settings_set_default();
        daq_settings_write_flash();
    }
    daq_settings_mem_to_front();
#else
    daq_settings_set_default();
    daq_settings_mem_to_front();
#endif
}

uint16_t daq_setings_get_param_int(param_id_t param_id){
    return settings_parameter[param_id].value_int;
}

uint16_t daq_setings_get_param_frac(param_id_t param_id){
    return settings_parameter[param_id].value_frac;
}

param_status_t daq_setings_get_param_status(param_id_t param_id){
    return settings_parameter[param_id].status;
}

void daq_setings_set_param_status(param_id_t param_id, param_status_t status){
    settings_parameter[param_id].status = status;
}

param_id_t daq_gui_param_view_to_id(view_parameters_t param_view){
    return param_view-1;
}

void daq_setings_parameter_increment(view_parameters_t param_view, int16_t num_steps){
    param_id_t param_id = daq_gui_param_view_to_id(param_view);
    int16_t increment = 0;
    increment = abs(num_steps);
    if(increment > 1){
        increment = num_steps * increment;
    }else{
        increment = num_steps;
    }
    switch(param_id){
        case ID_K_T1:{
            settings.k_t1 += (increment * SETTINGS_INCREMENT_K_T1);     // increment actual value (in RAM)
            split_float(settings.k_t1, &settings_parameter[ID_K_T1].value_int, &settings_parameter[ID_K_T1].value_frac, 10); // prepare for display
            break;
        }
        case ID_K_T2:{
            settings.k_t2 += (increment * SETTINGS_INCREMENT_K_T2);
            split_float(settings.k_t2, &settings_parameter[ID_K_T2].value_int, &settings_parameter[ID_K_T2].value_frac, 10);
            break;
        }
        case ID_K_F:{
            settings.k_f += (increment * SETTINGS_INCREMENT_K_F);
            split_float(settings.k_f, &settings_parameter[ID_K_F].value_int, &settings_parameter[ID_K_F].value_frac, 10);
            break;
        }
        case ID_DUMMY:{
            settings.dummy += (increment * SETTINGS_INCREMENT_DUMMY);
            split_float(settings.dummy, &settings_parameter[ID_DUMMY].value_int, &settings_parameter[ID_DUMMY].value_frac, 10);
            break;
        }
        default:
            break;
    }
    daq_setings_set_param_status(param_id, STATUS_CHANGED); // indicate that value was changed and needs to be saved in flash
}

void daq_settings_save(void){
#ifdef USE_FLASH
    daq_settings_write_flash();
#endif
    daq_settings_status_reset();
}