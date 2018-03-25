#ifndef _FrameBuffer_H_
#define _FrameBuffer_H_

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "SSD1306.h"

#define column(x) 12*(x-1)
#define row(y) 16*(y-1)

void drawBitmap(const uint8_t *bitmap, uint8_t height, uint8_t width, uint8_t pos_x, uint8_t pos_y);
void drawBuffer(const uint8_t *buffer);
void drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status);
void drawPixel(uint8_t pos_x, uint8_t pos_y);
void drawVLine(uint8_t x, uint8_t y, uint8_t length);
void drawHLine(uint8_t pos_y, uint8_t start, uint8_t length);
void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill);
void clear();
void show();
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void drawString(int16_t x, int16_t y, const char *charArray);
void drawNumber(int16_t x, int16_t y, int number);
	
#endif