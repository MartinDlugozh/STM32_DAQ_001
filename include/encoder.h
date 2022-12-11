#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdio.h>
#include <stdlib.h>

#define ENC_DIR_CW 1
#define ENC_DIR_CCW 0

typedef enum
{
    EM_SW_ViEW = 0,
    EM_SW_SETTING,
    EM_ADJ_SETTING
}encoder_mode_t;

uint16_t encoder_get_val(void);
uint8_t encoder_get_direction(void);
void encoder_set_direction(uint8_t direction);
void encoder_update(void);
int16_t encoder_get_inc(void);
uint8_t encoder_check_ink(void);
void encoder_force_reset_inc(void);
uint8_t encoder_sw_check_pressed(void);
void encoder_sw_continue(void);
void encoder_sw_callback(void);
encoder_mode_t encoedr_get_mode(void);
void encoder_set_mode(encoder_mode_t new_mode);

#endif