#ifndef _DAQ_GUI_H_
#define _DAQ_GUI_H_

#define SPI_TX_BUFFER_LEN 16      // SPI display TX buffer length
#define DISPLAY_NUM_LINES 6       // total number of lines per one view

#define DISP_FONT_PX_Y 18         // character pixels along Y
#define DISP_FONT_PX_X 11         // character pixels along X

#define DISP_CURSOR_X_OFFS 1
#define DISP_CURSOR_0_Y 8
#define DISP_CURSOR_PX_Y 18
#define DISP_CURSOR_PX_X (ST7735_WIDTH - 2*DISP_CURSOR_X_OFFS)


#define DISP_LINE_0_Y 8
#define DISP_LINE_1_Y (DISP_LINE_0_Y + DISP_FONT_PX_Y)
#define DISP_LINE_2_Y (DISP_LINE_1_Y + DISP_FONT_PX_Y)
#define DISP_LINE_3_Y (DISP_LINE_2_Y + DISP_FONT_PX_Y)
#define DISP_LINE_4_Y (DISP_LINE_3_Y + DISP_FONT_PX_Y)
#define DISP_LINE_5_Y (DISP_LINE_4_Y + DISP_FONT_PX_Y)

#define DISP_TABLE_X 2
#define DISP_VALUE_X 30

#define DISP_SET_TABLE_X 2
#define DISP_SET_VALUE_X 60

#include <stdio.h>
#include <string.h>
#include "main.h"

#include "st7735.h"
#include "fonts.h"

#include "OneWire.h"

#include "encoder.h"            // encoder is used here for switching between wiews and settings, so it should be included first
#include "daq_settings.h"       // gui also works with settings
#include "daq_settings_v2.h"    // currently abandonned version of settings library

typedef enum view{
    VIEW_START = 0,
    VIEW_SOL,        // SOL and EOL are needed for daq_gui_set_view_next() and daq_gui_set_view_previous()
    VIEW_MAIN,
    VIEW_RAW_DATA,
    VIEW_SETTINGS,
    VIEW_EOL
}view_t;

// public functions:
void daq_gui_poll_encoder_sw(void);                                 // 1. poll encoder button first
uint8_t daq_gui_poll_encoder_increment(void);                       // 2. then encoder increment - it will define which view will be displayed next
void daq_gui_draw_view(view_t gui_view, uint8_t gui_change_view);   // 4. redraw  screen

void daq_gui_cls(void);     // not implemented?

void daq_gui_set_current_view(view_t view);
view_t daq_gui_get_current_view(void);
void daq_gui_set_view_next(void);
void daq_gui_set_view_previous(void);

void daq_gui_set_current_param(view_parameters_t parameter);
view_parameters_t daq_gui_get_current_param(void);
void daq_gui_set_param_next(void);
void daq_gui_set_param_previous(void);

void daq_gui_draw_curosr(uint8_t line);
void daq_gui_clear_curosr(uint8_t line);
void daq_gui_update_cursor(void);                                   // 3. redraw cursor, if it is active

void daq_gui_view_start(void);
void daq_qui_view_main(uint8_t gui_change_view);
void daq_gui_view_raw_data(uint8_t gui_change_view);
void daq_gui_view_settings(uint8_t gui_change_view);

// private functions:
void daq_qui_write_line_normal(uint8_t line, uint16_t x_offset);
void daq_qui_draw_val_table(void);
void daq_qui_draw_set_table(void);
void daq_qui_println_param_float(uint8_t line, uint16_t x_offset, param_status_t status, uint16_t val_int, uint16_t val_frac, uint8_t dop);
void daq_qui_println_param_dec(uint8_t line, uint16_t x_offset, uint16_t val);
void daq_qui_println_param_long(uint8_t line, uint16_t x_offset, uint32_t val);

#endif