#ifndef _DAQ_PARAM_H_
#define _DAQ_PARAM_H_

#include <stdint.h>

#define DAQ_NUM_PARAMETERS 5

typedef struct gui_parm{
    const char* name;
    float value;
    uint16_t value_int;
    uint16_t value_frac;
}daq_parm_t;

#endif