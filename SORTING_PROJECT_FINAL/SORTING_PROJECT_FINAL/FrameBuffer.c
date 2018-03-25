#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "SSD1306.h"
#include "glcdfont.c"
#include "FrameBuffer.h"

// Global Variables
uint8_t buffer[1024] = "";


void drawBitmap(const uint8_t *progmem_bitmap, uint8_t height, uint8_t width, uint8_t pos_x, uint8_t pos_y) 
{
    uint8_t current_byte;
    uint8_t byte_width = (width + 7)/8;

    for (uint8_t current_y = 0; current_y < height; current_y++) {
        for (uint8_t current_x = 0; current_x < width; current_x++) {
            current_byte = pgm_read_byte(&progmem_bitmap + current_y*byte_width + current_x/8);
            if (current_byte & (128 >> (current_x&7))) {
                drawPixelP(current_x+pos_x, current_y+pos_y, 1);
            } else {
                drawPixelP(current_x+pos_x, current_y+pos_y, 0);
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
                drawPixelP(x_pos,y_pos,1);
            } else {
                drawPixelP(x_pos,y_pos,0);
            }
        }
    }
}

void drawPixelP(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status) 
{
	
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
    if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
        return;
    }

    buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
}


void drawVLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        drawPixel(x, y+i);
    }
}

void drawHLine(uint8_t x, uint8_t y, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) {
        drawPixel(x+i,y);
		drawPixel(x+i,y+1);
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

void drawRectangleFilled(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill) {
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
        line = pgm_read_byte(&(font[(c*5)+i]));
        for (int8_t j = 0; j<8; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                drawPixel(x+i, y+j); //removed color***
                else {  // big size
                    drawRectangleFilled(x+(i*size), y+(j*size), size+x+(i*size), size+y+(j*size), 1);
                }
                } else if (bg != color) { //*** remove?
                if (size == 1) // default size
                drawPixel(x+i, y+j); //removed color
                else {  // big size
                    drawRectangleFilled(x+i*size, y+j*size, size+x+(i*size), size+y+(j*size), 1);
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
        drawChar(j,y,*string, 0,0,2);
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
// Draws Running Screen
void drawRunning(void){
	clear();
	drawString(24,24, "Running");
	show();
	
}

// Draws Paused Screen
void drawPause(uint8_t objects){
	clear();
	uint8_t pos_x = 15; 
	uint8_t pos_y = 8; 

	// Pause String
	drawVLine(pos_x-3, pos_y -2, 18);
	drawVLine(pos_x-4, pos_y -2, 18);
	drawString(pos_x,pos_y,"P");
	drawString(pos_x+18,pos_y,"A");
	drawString(pos_x+36,pos_y,"U");
	drawString(pos_x+52,pos_y,"S");	
	drawString(pos_x+70,pos_y,"E");	
	drawString(pos_x+88,pos_y,"D");	
	drawVLine(pos_x+101, pos_y-2, 18);
	drawVLine(pos_x+102, pos_y-2, 18);
	
	// Object Count String
	drawString(0,pos_y + 20,"Objects");
	drawString(90,pos_y + 20,"bwt");
	drawString(0,pos_y + 36,"sensors:");
	drawNumber(100,pos_y + 36, objects);

	show();	
}

// Draws Object Count
void drawObjCount(uint8_t blk, uint8_t wht, uint8_t alm, uint8_t stl){
	
	drawVLine(9, 16, 32);
	drawString(12,16,"BL");	
	drawVLine(37, 16, 32);
	drawString(40,16,"WH");
	drawVLine(65, 16, 32);
	drawString(68,16,"AL");
	drawVLine(93, 16, 32);
	drawString(96,16,"ST");
	drawVLine(121, 16, 32);
	
	drawNumber(12,32, blk);
	drawNumber(40,32, wht);
	drawNumber(68,32, alm);
	drawNumber(96,32, stl);
}

// Draws ADC Calibration Screen
void drawCalibrateADC(void){
	
	drawString(12,0, "Calibrate");
	
	drawVLine(9, 20, 32);
	drawString(12,20,"BL");
	drawVLine(37, 20, 32);
	drawString(40,20,"WH");
	drawVLine(65, 20, 32);
	drawString(68,20,"AL");
	drawVLine(93, 20, 32);
	drawString(96,20,"ST");
	drawVLine(121, 20, 32);	
	
	
	//return blk, wht, alm, stl;
}

void drawSelectBar(uint8_t barState){


	// Selection Bar
	
	switch(barState){
		// Black Selection Bar
		case selBlack:
		
			drawHLine(12,38,23);
			drawHLine(12,40,23);
			drawHLine(12,42,23);
			break;
		// White Selection Bar
		case selWhite:
			drawHLine(40,38,23);
			drawHLine(40,40,23);
			drawHLine(40,42,23);
			break;
		// Alum Selection bar
		case selAlum:
			drawHLine(68,38,23);
			drawHLine(68,40,23);
			drawHLine(68,42,23);
			break;
		case selSteel:
		// Steel Selection Bar
			drawHLine(96,38,23);
			drawHLine(96,40,23);
			drawHLine(96,42,23);	
			break;
		
	}// end switch
}

void drawBlackCali(uint16_t blkCali){
	
	clear();
	drawString(5,32, "Black:");
	drawNumber(80,32, blkCali);
	show();
	
}


void drawWhiteCali(uint16_t whtCali){
		clear();
		drawString(5,32, "white:");
		drawNumber(80,32, whtCali);
		show();
}


void drawAluminumCali(uint16_t almCali){
	clear();
	drawString(5,32, "Alum:");
	drawNumber(80,32, almCali);
	show();	
}


void drawSteelCali(uint16_t stlCali){
	clear();
	drawString(5,32, "Steel:");
	drawNumber(80,32, stlCali);
	show();	
}