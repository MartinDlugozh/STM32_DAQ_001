#include "daq_param.h"
#include "main.h"
#include "daq_gui.h"

daq_parm_t gui_parm_sec = {
    .name = "SE",
};

daq_parm_t gui_parm_t_ref = {
    .name = "TR",
};

daq_parm_t gui_parm_t_a = {
    .name = "TA",
};

daq_parm_t gui_parm_t_b = {
    .name = "TB",
};

daq_parm_t gui_parm_flow = {
    .name = "FW",
};

daq_parm_t* gui_param_table[DAQ_NUM_PARAMETERS] = {
    &gui_parm_sec,
    &gui_parm_t_ref,
    &gui_parm_t_a,
    &gui_parm_t_b,
    &gui_parm_flow
};

enum{                       // make sure that the order is the same as in the table above
    PARAM_ID_SEC = 0,
    PARAM_ID_T_REF,
    PARAM_ID_T_A,
    PARAM_ID_T_B,
    PARAM_ID_FLOW
};

void daq_gui_param_init(void){

}

const char* daq_gui_param_get_name(uint8_t param_id){
    if(param_id < DAQ_NUM_PARAMETERS){
        return gui_param_table[param_id]->name;
    }
    return 0;
}

int16_t daq_gui_param_get_int(uint8_t param_id){
    if(param_id < DAQ_NUM_PARAMETERS){
        return gui_param_table[param_id]->value_int;
    }
    return 0;
}

uint16_t daq_gui_param_get_frac(uint8_t param_id){
    if(param_id < DAQ_NUM_PARAMETERS){
        return gui_param_table[param_id]->value_frac;
    }
    return 0;
}

void daq_gui_param_convert(uint8_t param_id){
    if(param_id < DAQ_NUM_PARAMETERS){
        split_float(gui_param_table[param_id]->value, &gui_param_table[param_id]->value_int, &gui_param_table[param_id]->value_frac, 10);
    }
    // return 0;
}

void daq_gui_param_convert_all(void){
    for (uint8_t i = 0; i < DAQ_NUM_PARAMETERS; i++){
        split_float(gui_param_table[i]->value, &gui_param_table[i]->value_int, &gui_param_table[i]->value_frac, 10);
    }
}

void daq_gui_param_set_value(uint8_t param_id, float val){
    if(param_id < DAQ_NUM_PARAMETERS){
        gui_param_table[param_id]->value = val;
    }
    // return 0;
}