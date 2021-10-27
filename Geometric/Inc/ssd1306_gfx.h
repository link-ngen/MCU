#ifndef SSD1306_GFX_H_
#define SSD1306_GFX_H_

#include "ssd1306.h"
#include "math.h"

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

typedef struct {
	int16_t	crrnt_x;
	int16_t crrnt_y;
	int16_t dx;
	int16_t dy;
} GFXTrans2DMatrix_t;

// private
static void ssd1306_GFX_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, SSD1306_COLOR color);
static void ssd1306_GFX_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, SSD1306_COLOR color);
static void ssd1306_GFX_FillCircleHelper(uint8_t x0, uint8_t y0, uint8_t r,
        							uint8_t corners, uint8_t delta, SSD1306_COLOR color);
// public
void ssd1306_GFX_Translate(uint8_t x, uint8_t y);
void ssd1306_GFX_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_GFX_FillCircle(uint8_t x, uint8_t y, uint8_t r, SSD1306_COLOR color);
void ssd1306_GFX_FillTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void ssd1306_GFX_DrawBitMap(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w, uint8_t h, SSD1306_COLOR color);

#endif /* SSD1306_GFX_H_ */
