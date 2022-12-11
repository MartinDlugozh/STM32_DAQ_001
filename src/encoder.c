#include "main.h"
#include "encoder.h"

encoder_mode_t encoder_mode = EM_SW_ViEW;

uint8_t _encoder_direction = ENC_DIR_CW;
uint16_t encoder_val = 0;
uint16_t encoder_prev_val = 0;
int16_t encoder_inc = 0;
uint8_t sw_pressed = 0;

void encoder_init(){

}

uint16_t encoder_get_val(void){
    return ((TIM3->CNT)>>2);
}

uint8_t encoder_get_direction(void){
    return _encoder_direction;
}

void encoder_set_direction(uint8_t direction){
    if((direction == ENC_DIR_CW) || (direction == ENC_DIR_CCW)){
        _encoder_direction = direction;
    }
    
}

void encoder_update(void){
  encoder_val = encoder_get_val();
  encoder_inc = encoder_val - encoder_prev_val;
  encoder_prev_val = encoder_val;

  if(encoder_inc != 0){
    if(abs(encoder_inc) > 1000){
      encoder_inc = 1;
    }
    if(encoder_inc > 0){
      encoder_set_direction(ENC_DIR_CW);
    }else{
      encoder_set_direction(ENC_DIR_CCW);
    }
  }    
}

int16_t encoder_get_inc(void){
    return encoder_inc;
}

uint8_t encoder_check_ink(void){
    encoder_update();
    if(encoder_inc != 0){
        return 1;
    }
    return 0;
}

void encoder_force_reset_inc(void){
    encoder_inc = 0;
}

void encoder_sw_callback(void){   // this should be called from EXTI ISR handler
  sw_pressed = 1;
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

uint8_t encoder_sw_check_pressed(void){
  uint8_t res = sw_pressed;
  sw_pressed = 0;
  return res;
}

void encoder_sw_continue(void){
  __HAL_GPIO_EXTI_CLEAR_FLAG(Encoder_button_Pin);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

encoder_mode_t encoedr_get_mode(void){
  return encoder_mode;
}

void encoder_set_mode(encoder_mode_t new_mode){
  encoder_mode = new_mode;
}
