#include "ssd1306_gfx.h"

static void ssd1306_GFX_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h,
		SSD1306_COLOR color)
{
	ssd1306_Line(x, y, x, y + h - 1, color);
}

static void ssd1306_GFX_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w,
		SSD1306_COLOR color)
{
	ssd1306_Line(x, y, x + w - 1, y, color);
}

static void ssd1306_GFX_FillCircleHelper(uint8_t x0, uint8_t y0, uint8_t r,
		uint8_t corners, uint8_t delta, SSD1306_COLOR color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	int16_t px = x;
	int16_t py = y;

	delta++; // Avoid some +1's in the loop

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		// These checks avoid double-drawing certain lines, important
		// for the SSD1306 library which has an INVERT drawing mode.
		if (x < (y + 1))
		{
			if (corners & 1)
				ssd1306_GFX_DrawFastVLine(x0 + x, y0 - y, 2 * y + delta, color);
			if (corners & 2)
				ssd1306_GFX_DrawFastVLine(x0 - x, y0 - y, 2 * y + delta, color);
		}
		if (y != py)
		{
			if (corners & 1)
				ssd1306_GFX_DrawFastVLine(x0 + py, y0 - px, 2 * px + delta, color);
			if (corners & 2)
				ssd1306_GFX_DrawFastVLine(x0 - py, y0 - px, 2 * px + delta, color);
			py = y;
		}
		px = x;
	}
}

void ssd1306_GFX_Translate(uint8_t x, uint8_t y)
{

}

void ssd1306_GFX_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		SSD1306_COLOR color)
{
	uint8_t w = x2 - x1;
	uint8_t h = y2 - y1;

	for (uint8_t i = x1; i < x1 + w; ++i)
	{
		ssd1306_GFX_DrawFastVLine(i, y1, h, color);
	}
}

void ssd1306_GFX_FillCircle(uint8_t x, uint8_t y, uint8_t r, SSD1306_COLOR color)
{
	ssd1306_GFX_DrawFastVLine(x, y - r, 2 * r + 1, color);
	ssd1306_GFX_FillCircleHelper(x, y, r, 3, 0, color);
}

void ssd1306_GFX_FillTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
		uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1)
	{
		_swap_int16_t(y0, y1);
		_swap_int16_t(x0, x1);
	}
	if (y1 > y2)
	{
		_swap_int16_t(y2, y1);
		_swap_int16_t(x2, x1);
	}
	if (y0 > y1)
	{
		_swap_int16_t(y0, y1);
		_swap_int16_t(x0, x1);
	}

	if (y0 == y2)
	{ // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)
			a = x1;
		else if (x1 > b)
			b = x1;
		if (x2 < a)
			a = x2;
		else if (x2 > b)
			b = x2;
		ssd1306_GFX_DrawFastHLine(a, y0, b - a + 1, color);

		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
			dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2)
		last = y1; // Include y1 scanline
	else
		last = y1 - 1; // Skip it

	for (y = y0; y <= last; y++)
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* longhand:
		 a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		 b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		 */
		if (a > b)
			_swap_int16_t(a, b);
		ssd1306_GFX_DrawFastHLine(a, y, b - a + 1, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = (int32_t) dx12 * (y - y1);
	sb = (int32_t) dx02 * (y - y0);
	for (; y <= y2; y++)
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* longhand:
		 a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		 b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		 */
		if (a > b)
			_swap_int16_t(a, b);
		ssd1306_GFX_DrawFastHLine(a, y, b - a + 1, color);
	}
}

void ssd1306_GFX_DrawBitMap(uint8_t x, uint8_t y, const uint8_t bitmap[], uint8_t w,
		uint8_t h, SSD1306_COLOR color)
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	for (int16_t j = 0; j < h; j++, y++)
	{
		for (int16_t i = 0; i < w; i++)
		{
			if (i & 7)
				byte <<= 1;
			else
				byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
			if (byte & 0x80)
				ssd1306_DrawPixel(x + i, y, color);
		}
	}
}
