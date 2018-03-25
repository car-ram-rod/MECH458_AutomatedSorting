#include <stdlib.h>
#include <avr/io.h>
#include "SSD1306.h"

// Global Variables
volatile uint8_t buffer[1024];


void drawBitmap(const uint8_t *progmem_bitmap, uint8_t height, uint8_t width, uint8_t pos_x, uint8_t pos_y) 
{
    uint8_t current_byte;
    uint8_t byte_width = (width + 7)/8;

    for (uint8_t current_y = 0; current_y < height; current_y++) {
        for (uint8_t current_x = 0; current_x < width; current_x++) {
            current_byte = pgm_read_byte(progmem_bitmap + current_y*byte_width + current_x/8);
            if (current_byte & (128 >> (current_x&7))) {
                drawPixel(current_x+pos_x,current_y+pos_y,1);
            } else {
                drawPixel(current_x+pos_x,current_y+pos_y,0);
            }
        }
    }
}

void drawBuffer(const uint8_t *progmem_buffer) 
{
    uint8_t current_byte;

    for (uint8_t y_pos = 0; y_pos < 64; y_pos++) {
        for (uint8_t x_pos = 0; x_pos < 128; x_pos++) {
            current_byte = pgm_read_byte(progmem_buffer + y_pos*16 + x_pos/8);
            if (current_byte & (128 >> (x_pos&7))) {
                drawPixel(x_pos,y_pos,1);
            } else {
                drawPixel(x_pos,y_pos,0);
            }
        }
    }
}

void drawPixel(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status) 
{
	
	uint8_t buffer[1024];
	
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
        return;
    }

    if (pixel_status) {
        buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
    } else {
        buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] &= ~(1 << (pos_y&7));
    }
}

void drawPixel(uint8_t pos_x, uint8_t pos_y) 
{
	uint8_t buffer[1024];
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
        return;
    }

    buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
}

void drawVLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        drawPixel(x,i+y);
    }
}

void drawHLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        drawPixel(i+x,y);
    }
}

void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    uint8_t length = x2 - x1 + 1;
    uint8_t height = y2 - y1;

    drawHLine(x1,y1,length);
    drawHLine(x1,y2,length);
    drawVLine(x1,y1,height);
    drawVLine(x2,y1,height);
}

void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill) {
    if (!fill) {
        drawRectangle(x1,y1,x2,y2);
    } else {
        uint8_t length = x2 - x1 + 1;
        uint8_t height = y2 - y1;

        for (int x = 0; x < length; ++x) {
            for (int y = 0; y <= height; ++y) {
                drawPixel(x1+x,y+y1);
            }
        }
    }
}

void clear(void) {
    for (uint16_t buffer_location = 0; buffer_location < SSD1306_BUFFERSIZE; buffer_location++) {
        buffer[buffer_location] = 0x00;
    }
}

void show(void) {
    sendFramebuffer(buffer);
}

// Draw a character ***turn to uint8_t ?
//For size 2 the bottom right max is 117,50
//Size 2 fits 4 lines, at y= 0, 16, 32, 48
//fits 10 characters per line at multiples of 12
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {

//  if((x >= _width)            || // Clip right
//  (y >= _height)           || // Clip bottom
//  ((x + 6 * size - 1) < 0) || // Clip left
//  ((y + 8 * size - 1) < 0))   // Clip top
//  return;

    for (int8_t i=0; i<6; i++ ) {
        uint8_t line;
        if (i == 5)
        line = 0x0;
        else
        line = pgm_read_byte(font+(c*5)+i);
        for (int8_t j = 0; j<8; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                drawPixel(x+i, y+j); //removed color***
                else {  // big size
                    drawRectangle(x+(i*size), y+(j*size), size+x+(i*size), size+y+(j*size), 1);
                }
                } else if (bg != color) { //*** remove?
                if (size == 1) // default size
                drawPixel(x+i, y+j); //removed color
                else {  // big size
                    drawRectangle(x+i*size, y+j*size, size+x+(i*size), size+y+(j*size), 1);
                }
            }
            line >>= 1;
        }
    }
}

// Draw a string
void drawString(int16_t x, int16_t y, const char *string){

    for(char j=x; *string; string++,j+=12){
        //(x,y,char,color,bg,size)
        drawChar(j,y,*string,0,0,2);
    }
    
}

void drawNumber(int16_t x, int16_t y, int number){
	char i;
	if(number<10){
		i = 2;
	}
	else if(number < 100){
		i = 3;
	}
	else if(number <1000){
		i = 4;
	}
	else{
		i = 7;
	}
	char str[i];
	sprintf(str,"%i",number);
	drawString(x,y,str);
}
