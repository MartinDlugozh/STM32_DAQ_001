#include "daq_settings_v2.h"

setV2param_t param_k_t1 = {
    .name = "K_TA",
    .value = SETTINGS_DEFAULT_K_T1,
    .status = STATUS_UNCHANGED,
};

setV2param_t param_k_t2 = {
    .name = "K_TB",
    .value = SETTINGS_DEFAULT_K_T2,
    .status = STATUS_UNCHANGED,
};

setV2param_t param_k_f = {
    .name = "K_F",
    .value = SETTINGS_DEFAULT_K_F,
    .status = STATUS_UNCHANGED,
};

setV2param_t param_dummy = {
    .name = "K_D",
    .value = SETTINGS_DEFAULT_DUMMY,
    .status = STATUS_UNCHANGED,
};

setV2param_t* daq_setV2_table[SETV2_NUM_PARAMETERS] = {
    &param_k_t1,
    &param_k_t2,
    &param_k_f,
    &param_dummy
};

void daq_setV2_init(void){
    split_float(param_k_t1.value, &param_k_t1.value_int, &param_k_t1.value_frac, 10);
    split_float(param_k_t2.value, &param_k_t2.value_int, &param_k_t2.value_frac, 10);
    split_float(param_k_f.value, &param_k_f.value_int, &param_k_f.value_frac, 10);
    split_float(param_dummy.value, &param_dummy.value_int, &param_dummy.value_frac, 10);
}

const char* daq_setV2_get_param_name(uint8_t id){
    return daq_setV2_table[id]->name;
}

uint8_t daq_setV2_get_param_status(uint8_t id){
    return daq_setV2_table[id]->status;
}

uint16_t daq_setV2_get_param_int(uint8_t id){
    return daq_setV2_table[id]->value_int;
}

uint16_t daq_setV2_get_param_frac(uint8_t id){
    return daq_setV2_table[id]->value_frac;
}