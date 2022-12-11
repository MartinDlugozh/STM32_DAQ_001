#include <stdint.h>

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include "ssd1306_conf.h"
#include "fonts.h"

// suppressed due to conflict with "fonts.h" supplied with ST7735

// typedef struct {
// 	const uint8_t width;    /*!< Font width in pixels */
// 	uint8_t height;   /*!< Font height in pixels */
// 	const uint16_t *data; /*!< Pointer to data font data array */
// } FontDef;

#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
#endif // __SSD1306_FONTS_H__
