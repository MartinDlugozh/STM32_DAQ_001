#include "main.h"
#include "cmsis_os.h"
#include "daq_gui.h"

view_t gui_current_view = VIEW_MAIN;
view_parameters_t gui_current_parameter = SET_K_T1;

uint8_t spi_tx_buffer[SPI_TX_BUFFER_LEN];
uint8_t spi_tx_len_next[DISPLAY_NUM_LINES];
uint8_t spi_tx_len_previous[DISPLAY_NUM_LINES];

extern volatile uint32_t   timer_sec;
extern uint32_t            adc_dma_buffer[ADC_DMA_BUFFER_LEN];
extern uint16_t            pps;
extern Temperature  t;

extern float               temperature_1;
extern float               temperature_2;
extern float               flow_rate;

static const uint8_t space[SPI_TX_BUFFER_LEN] = "               ";

void daq_gui_cls(void){
  ST7735_FillScreen(ST7735_BLACK);
  memset(spi_tx_buffer, 0x00, SPI_TX_BUFFER_LEN);
  memset(spi_tx_len_next, 0x00, DISPLAY_NUM_LINES);
  memset(spi_tx_len_previous, 0x00, DISPLAY_NUM_LINES);
}

void daq_gui_set_current_view(view_t view){
  gui_current_view = view;
}

void daq_gui_set_view_next(void){
  if(gui_current_view < (VIEW_EOL-1)){
    gui_current_view++;
  }else{
    gui_current_view = VIEW_SOL+1;
  }
  // daq_gui_draw_view(gui_current_view, 1);
  // osDelay(500);
}

void daq_gui_set_view_previous(void){
  if(gui_current_view > (VIEW_SOL+1)){
    gui_current_view--;
  }else{
    gui_current_view = VIEW_EOL-1;
  }
  // daq_gui_draw_view(gui_current_view, 1);
  // osDelay(500);
}

view_t daq_gui_get_current_view(void){
  return gui_current_view;
}

void daq_gui_draw_curosr(uint8_t line){
  // here we assume, that parameter id is equal to it's line number (ex., ID_K_T1 = 0, so it will be placed on line 0, etc.)
  ST7735_DrawRectangle(DISP_CURSOR_X_OFFS, (DISP_LINE_0_Y + line * DISP_CURSOR_PX_Y), DISP_CURSOR_PX_X, ((DISP_LINE_0_Y + line * DISP_CURSOR_PX_Y)+DISP_CURSOR_PX_Y), ST7735_GREEN);
}

void daq_gui_clear_curosr(uint8_t line){
  ST7735_DrawRectangle(DISP_CURSOR_X_OFFS, (DISP_LINE_0_Y + line * DISP_CURSOR_PX_Y), DISP_CURSOR_PX_X, ((DISP_LINE_0_Y + line * DISP_CURSOR_PX_Y)+DISP_CURSOR_PX_Y), ST7735_BLACK);
}

void daq_gui_set_current_param(view_parameters_t parameter){
  gui_current_parameter = parameter;
  // somehow get previous parameter
  // redraw cursor
}

void daq_gui_update_cursor(void){
  if((encoedr_get_mode() == EM_ADJ_SETTING) || (encoedr_get_mode() == EM_SW_SETTING)){
    daq_gui_draw_curosr(daq_gui_param_view_to_id(gui_current_parameter));
  }else{
    daq_gui_clear_curosr(daq_gui_param_view_to_id(gui_current_parameter));
  }
}

void daq_gui_set_param_next(void){
  daq_gui_clear_curosr(daq_gui_param_view_to_id(gui_current_parameter));  // clear cursor
  if(gui_current_parameter < (SET_EOL-1)){
    gui_current_parameter++;
  }else{
    gui_current_parameter = SET_SOL+1;
  }
  daq_gui_draw_curosr(daq_gui_param_view_to_id(gui_current_parameter)); // redraw cursor
}

void daq_gui_set_param_previous(void){
  daq_gui_clear_curosr(daq_gui_param_view_to_id(gui_current_parameter));  // clear cursor
  if(gui_current_parameter > (SET_SOL+1)){
    gui_current_parameter--;
  }else{
    gui_current_parameter = SET_EOL-1;
  }
  daq_gui_draw_curosr(daq_gui_param_view_to_id(gui_current_parameter)); // redraw cursor
}

view_parameters_t daq_gui_get_current_param(void){
  return gui_current_parameter;
}

void daq_qui_write_line_normal(uint8_t line, uint16_t x_offset){
  ST7735_WriteString(x_offset, (DISP_LINE_0_Y + line * DISP_FONT_PX_Y), (char*)spi_tx_buffer, Font_11x18, ST7735_GREEN, ST7735_BLACK);
}

void daq_qui_draw_val_table(void){
  sprintf((char*)spi_tx_buffer, "SE");  // line 0
  daq_qui_write_line_normal(0, DISP_TABLE_X);
  sprintf((char*)spi_tx_buffer, "TR");  // line 1
  daq_qui_write_line_normal(1, DISP_TABLE_X);
  sprintf((char*)spi_tx_buffer, "TA");  // line 2
  daq_qui_write_line_normal(2, DISP_TABLE_X);
  sprintf((char*)spi_tx_buffer, "TB");  // line 3
  daq_qui_write_line_normal(3, DISP_TABLE_X);
  sprintf((char*)spi_tx_buffer, "FW");  // line 4
  daq_qui_write_line_normal(4, DISP_TABLE_X);
}

void daq_qui_draw_set_table(void){
  sprintf((char*)spi_tx_buffer, "K_TA");  // line 0
  daq_qui_write_line_normal(0, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "K_TB");  // line 1
  daq_qui_write_line_normal(1, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "K_F");  // line 2
  daq_qui_write_line_normal(2, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "K_D");  // line 3
  daq_qui_write_line_normal(3, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "SAVE");  // line 4
  daq_qui_write_line_normal(4, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "BACK");  // line 5
  daq_qui_write_line_normal(5, DISP_SET_TABLE_X);
}

void daq_gui_draw_set_table_v2(void){
  for (uint8_t i = 0; i < SETV2_NUM_PARAMETERS; i++){
    ST7735_WriteString(DISP_SET_TABLE_X, (DISP_LINE_0_Y + i * DISP_FONT_PX_Y), daq_setV2_get_param_name(i), Font_11x18, ST7735_GREEN, ST7735_BLACK);
  }

  sprintf((char*)spi_tx_buffer, "SAVE");
  daq_qui_write_line_normal(SETV2_NUM_PARAMETERS, DISP_SET_TABLE_X);
  sprintf((char*)spi_tx_buffer, "BACK");
  daq_qui_write_line_normal((SETV2_NUM_PARAMETERS+1), DISP_SET_TABLE_X);
}

void daq_qui_println_param_float(uint8_t line, uint16_t x_offset, param_status_t status, uint16_t val_int, uint16_t val_frac){
  uint16_t color = ST7735_GREEN;
  if(status == STATUS_CHANGED){
    color = ST7735_RED;
  }
  memset(spi_tx_buffer, 0x00, SPI_TX_BUFFER_LEN);
  spi_tx_len_previous[line] = spi_tx_len_next[line];
  spi_tx_len_next[line] = sprintf((char*)spi_tx_buffer, "%d.%02d", val_int, val_frac);
  if(spi_tx_len_next[line] < spi_tx_len_previous[line]){
    strncat((char*)spi_tx_buffer, (char*)space, (spi_tx_len_previous[line] - spi_tx_len_next[line]));
  }
  ST7735_WriteString(x_offset, (DISP_LINE_0_Y + line * DISP_FONT_PX_Y), (char*)spi_tx_buffer, Font_11x18, color, ST7735_BLACK);
}

void daq_qui_println_param_dec(uint8_t line, uint16_t x_offset, uint16_t val){
  memset(spi_tx_buffer, 0x00, SPI_TX_BUFFER_LEN);
  spi_tx_len_previous[line] = spi_tx_len_next[line];
  spi_tx_len_next[line] = sprintf((char*)spi_tx_buffer, "%d", val);
  if(spi_tx_len_next[line] < spi_tx_len_previous[line]){
    strncat((char*)spi_tx_buffer, (char*)space, (spi_tx_len_previous[line] - spi_tx_len_next[line]));
  }
  ST7735_WriteString(x_offset, (DISP_LINE_0_Y + line * DISP_FONT_PX_Y), (char*)spi_tx_buffer, Font_11x18, ST7735_GREEN, ST7735_BLACK);
}

void daq_qui_println_param_long(uint8_t line, uint16_t x_offset, uint32_t val){
  memset(spi_tx_buffer, 0x00, SPI_TX_BUFFER_LEN);
  spi_tx_len_previous[line] = spi_tx_len_next[line];
  spi_tx_len_next[line] = sprintf((char*)spi_tx_buffer, "%lu", val);
  if(spi_tx_len_next[line] < spi_tx_len_previous[line]){
    strncat((char*)spi_tx_buffer, (char*)space, (spi_tx_len_previous[line] - spi_tx_len_next[line]));
  }
  ST7735_WriteString(x_offset, (DISP_LINE_0_Y + line * DISP_FONT_PX_Y), (char*)spi_tx_buffer, Font_11x18, ST7735_GREEN, ST7735_BLACK);
}

void daq_gui_view_start(void){
  ST7735_WriteString(15, 40, "STM.DAQ v0.5", Font_11x18, ST7735_GREEN, ST7735_BLACK);
  ST7735_WriteString(10, 100, "December 2022", Font_11x18, ST7735_GREEN, ST7735_BLACK);
}

void daq_qui_view_main(uint8_t gui_change_view){
  uint16_t val_dec = 0;
  uint16_t val_frac = 0;
  
  if(gui_change_view == 1){
    daq_qui_draw_val_table();
  }

  // line 0 - SE
  daq_qui_println_param_long(0, DISP_VALUE_X, timer_sec);

  //line 1 - TR
  val_dec = t.inCelsus;
  val_frac = t.frac;
  daq_qui_println_param_float(1, DISP_VALUE_X, STATUS_UNCHANGED, val_dec, val_frac);

  //line 2 - TA
  split_float(temperature_1, &val_dec, &val_frac, 1);
  daq_qui_println_param_float(2, DISP_VALUE_X, STATUS_UNCHANGED, val_dec, val_frac);

  // line 3 - TB
  split_float(temperature_2, &val_dec, &val_frac, 1);
  daq_qui_println_param_float(3, DISP_VALUE_X, STATUS_UNCHANGED, val_dec, val_frac);

  // line 4 - FW
  split_float(flow_rate, &val_dec, &val_frac, 1);
  daq_qui_println_param_float(4, DISP_VALUE_X, STATUS_UNCHANGED, val_dec, val_frac);
}

void daq_gui_view_raw_data(uint8_t gui_change_view){
  uint16_t val_dec = 0;
  uint16_t val_frac = 0;

  if(gui_change_view == 1){
    daq_qui_draw_val_table();
  }

  // line 0 - SE
  daq_qui_println_param_long(0, DISP_VALUE_X, timer_sec);
  //line 1 - TR
  val_dec = t.inCelsus;
  val_frac = t.frac;
  daq_qui_println_param_float(1, DISP_VALUE_X, STATUS_UNCHANGED, val_dec, val_frac);
  //line 2 - TA
  daq_qui_println_param_dec(2, DISP_VALUE_X, (uint16_t)adc_dma_buffer[0]);
  // line 3 - TB
  daq_qui_println_param_dec(3, DISP_VALUE_X, (uint16_t)adc_dma_buffer[1]);
  // line 4 - FW
  daq_qui_println_param_dec(4, DISP_VALUE_X, pps);
}

void daq_gui_view_settings(uint8_t gui_change_view){
  if(gui_change_view == 1){
    // ST7735_WriteString(30, 50, "SETTINGS", Font_11x18, ST7735_GREEN, ST7735_BLACK);
    daq_qui_draw_set_table();
  }
  
  // line 0 - K_TA
  daq_qui_println_param_float(0, DISP_SET_VALUE_X, daq_setings_get_param_status(ID_K_T1), daq_setings_get_param_int(ID_K_T1), daq_setings_get_param_frac(ID_K_T1));
  //line 1 - K_TB
  daq_qui_println_param_float(1, DISP_SET_VALUE_X, daq_setings_get_param_status(ID_K_T2), daq_setings_get_param_int(ID_K_T2), daq_setings_get_param_frac(ID_K_T2));
  //line 2 - K_F
  daq_qui_println_param_float(2, DISP_SET_VALUE_X, daq_setings_get_param_status(ID_K_F), daq_setings_get_param_int(ID_K_F), daq_setings_get_param_frac(ID_K_F));
  // line 3 - K_D
  daq_qui_println_param_float(3, DISP_SET_VALUE_X, daq_setings_get_param_status(ID_DUMMY), daq_setings_get_param_int(ID_DUMMY), daq_setings_get_param_frac(ID_DUMMY));
}

void daq_gui_view_settings_v2(uint8_t gui_change_view){
  if(gui_change_view == 1){
    daq_qui_draw_set_table();
  }
  
  for (uint8_t i = 0; i < SETV2_NUM_PARAMETERS; i++){
    daq_qui_println_param_float(i, DISP_SET_VALUE_X, daq_setV2_get_param_status(i), daq_setV2_get_param_int(i), daq_setV2_get_param_frac(i));
  }
}

void daq_gui_draw_view(view_t gui_view, uint8_t gui_change_view){
  if(gui_change_view == 1){
    daq_gui_cls();
  }

  switch (gui_view)
  {
    case VIEW_START:
    {
      daq_gui_view_start();
      break;
    }
    case VIEW_MAIN:
    {
      daq_qui_view_main(gui_change_view);
      break;
    }
    case VIEW_RAW_DATA:
    {
      daq_gui_view_raw_data(gui_change_view);
      break;
    }
    case VIEW_SETTINGS:
    {
      daq_gui_view_settings(gui_change_view);
      break;
    } 
    default:
      break;
  }
}

uint8_t daq_gui_poll_encoder_increment(void){     // poll encoder rotation
  uint8_t gui_redraw = 0;
  uint8_t dir = ENC_DIR_CW;

  if(encoder_check_ink() == 1){
    dir = encoder_get_direction();  // get direction

    switch(encoedr_get_mode()){     // choose action depending on current encoder mode (VIEW, SETTING, ADJUSTMENT)
      case EM_SW_ViEW: {             // encoder switches views     
        if(dir == ENC_DIR_CW){
          daq_gui_set_view_next();
        }else{
          daq_gui_set_view_previous();
        }       
        gui_redraw = 1;
        break;
      }
      case EM_SW_SETTING: {
        if(dir == ENC_DIR_CW){
          daq_gui_set_param_next();
        }else{
          daq_gui_set_param_previous();
        }
        break;
      }
      case EM_ADJ_SETTING: {
        daq_setings_parameter_increment(daq_gui_get_current_param(), encoder_get_inc());
        break;
      }
      default:
        break;
    } // end encoder mode handler
  } // end encoder increment handler
  encoder_force_reset_inc();
  
  return gui_redraw;
}

void daq_gui_poll_encoder_sw(void){
  uint8_t sw_skip = 0;    // encoder pushbutton debounce feature
  if((encoder_sw_check_pressed() == 1) && (sw_skip == 0)){
      sw_skip = 1;
      switch (gui_current_view){
      case VIEW_MAIN: {
        // do nothing
        buzzer_on();
        break;
      }
      case VIEW_RAW_DATA: {
        // do nothing
        buzzer_on();
        break;
      }
      case VIEW_SETTINGS: {
        buzzer_on();

        switch(encoedr_get_mode()){
          case EM_SW_ViEW:{
            encoder_set_mode(EM_SW_SETTING); // go to settings mode
            break;
          }
          case EM_SW_SETTING:{
            if((daq_gui_get_current_param() != SET_SAVE) && (daq_gui_get_current_param() != SET_BACK)){ // parameter is selected
              encoder_set_mode(EM_ADJ_SETTING); // go to adjustment mode
            }else if(daq_gui_get_current_param() == SET_SAVE){  // save
              daq_settings_save();
              encoder_set_mode(EM_SW_ViEW);
            }else{  // back
              encoder_set_mode(EM_SW_ViEW);
            }         
            break;
          }
          case EM_ADJ_SETTING:{
            encoder_set_mode(EM_SW_SETTING); // go back to settings mode
            break;
          }
          default:
            break;
        }
        break;
      }
      default:
        break;
      }
    }else{
      sw_skip = 0;
      encoder_sw_continue();
    }
}