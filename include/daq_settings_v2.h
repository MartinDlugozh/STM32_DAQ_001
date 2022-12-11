#ifndef _DAQ_SETTINGS_V2_H_
#define _DAQ_SETTINGS_V2_H_

/**
 * In order to add new parameters:
 * 1. Change SETV2_NUM_PARAMETERS according to the amount of parameters you need to add;
 * 2. Define new parameters as setV2param_t in daq_settings_v2.c;
 * 3. Add your paraneters to daq_setV2_table.
 * 4. Everything else will be done automagicly =)
 */

#include <stdint.h>
#include "main.h"
#include "daq_settings.h"

#define SETV2_NUM_PARAMETERS 4

typedef struct {
    const char *name;
    float value;
    uint16_t value_int;
    uint16_t value_frac;
    uint8_t status;
}setV2param_t;

void daq_setV2_init(void); // call this from the init section of your main() before gui initialization

const char* daq_setV2_get_param_name(uint8_t id); 
uint8_t daq_setV2_get_param_status(uint8_t id);
uint16_t daq_setV2_get_param_int(uint8_t id);
uint16_t daq_setV2_get_param_frac(uint8_t id);


#endif