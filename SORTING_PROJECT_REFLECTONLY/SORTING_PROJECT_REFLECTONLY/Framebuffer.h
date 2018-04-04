#ifndef _FrameBuffer_H_
#define _FrameBuffer_H_

#include <stdlib.h>
#include <avr/io.h>
#include "SSD1306.h"

#define column(x) 12*(x-1)
#define row(y) 16*(y-1)

#define selBlack 1
#define selWhite 2
#define selAlum  3
#define selSteel 4
#define selAll	 5



void drawBitmap(const uint8_t *bitmap, uint8_t height, uint8_t width, uint8_t pos_x, uint8_t pos_y);
void drawBuffer(const uint8_t *buffer);
void drawPixelP(uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status);
void drawPixel(uint8_t pos_x, uint8_t pos_y);
void drawVLine(uint8_t x, uint8_t y, uint8_t length);
void drawHLine(uint8_t x, uint8_t y, uint8_t length);	//Doesn't work??
void drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void drawRectangleFilled(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t fill);
void clear(void);
void show(void);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void drawString(int16_t x, int16_t y, const char *charArray);
void drawNumber(int16_t x, int16_t y, int number);

/*************************************************************************
  Send Running string to oled
  
  Input:    None
  
*************************************************************************/
void drawInitializing(void);
/*************************************************************************
  Send Running string to oled
  
  Input:    None
  
*************************************************************************/
void drawRunning(void);

/*************************************************************************
  Send Pause string to oled
  
  Input:    objects - Number of objects between optical sensors
  
*************************************************************************/
void drawPause(uint8_t RLEX_Count, uint8_t BL_Count, uint8_t WH_Count, uint8_t AL_Count, uint8_t ST_Count);


/*************************************************************************
  Draws the Object Counter Screen
  
  Input:    blk - Black Objects
			wht - White Objects
			alm - Aluminum Objects
			stl - Steel Objects
			
*************************************************************************/
void drawObjCount(uint8_t blk, uint8_t wht, uint8_t alm, uint8_t stl);


/*************************************************************************
  Draws the ADC Calibration Screen
  
  Input:    blk - Black Value
			wht - White Value
			alm - Aluminum Value
			stl - Steel Value
			select - select which object to calibrate

*************************************************************************/
void drawCalibrateADC(void);	


/*************************************************************************
  Draws the Select Bar on Calibrate ADC screen
  
  Input:	select - select which object to calibrate

*************************************************************************/
void drawSelectBar(uint8_t barState);

/*************************************************************************
  Draws the PWM Duty Cycle calibration screen
  
  Input:	dutyCali - The duty cycle calibration value to be drawn

*************************************************************************/
void drawPWMCali(uint16_t dutyCali);
/*************************************************************************
  Draws the Black Object calibration screen
  
  Input:	blkCali - The black object calibration value to be drawn

*************************************************************************/
void drawBlackCali(uint16_t blkCali);

/*************************************************************************
  Draws the White Object calibration screen
  
  Input:	whtCali - The white object calibration value to be drawn

*************************************************************************/
void drawWhiteCali(uint16_t whtCali);

/*************************************************************************
  Draws the Aluminum Object calibration screen
  
  Input:	almCali - The aluminum object calibration value to be drawn

*************************************************************************/
void drawAluminumCali(uint16_t almCali);

/*************************************************************************
  Draws the Steel Object calibration screen
  
  Input:	stlCali - The steel object calibration value to be drawn

*************************************************************************/
void drawSteelCali(uint16_t stlCali);


/*************************************************************************
  Draws the Calibration Mode screen
  
  Input:	high - Highest value seen from ADC
			low - lowest value seen from ADC
			avg - the average of the values

*************************************************************************/
void drawCaliMode(uint16_t high, uint16_t low, uint16_t avg);



#endif