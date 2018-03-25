
#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "SSD1306.h"
#include "Framebuffer.h"
#include "I2C.h"
#include "glcdfont.c"

/*######## Function Declarations ########*/
void setupISR(void);
void setupPorts(void);
void initTimer0(void);
void initTimer1(void);
void mTimer(int count); // delay : function declaration

/*######## Global variables ########*/

// Menu States
uint8_t programPause = 0;
uint8_t menuState = 0;
uint8_t barState = 1;
uint8_t rampDownSet = 0; //Might not need
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

// Key Press State Variables
uint8_t right_key_press = 0; 
uint8_t left_key_press = 0;
uint8_t down_key_press = 0; 
uint8_t up_key_press = 0;

// Calibration Settings
volatile uint16_t blkCali = 100;
volatile uint16_t whtCali = 200;
volatile uint16_t almCali = 300;
volatile uint16_t stlCali = 400;


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	cli(); //disable all interrupts
	//initTimer0();
	//initTimer1();
	setupISR(); // Interrupt initializations (might have to add your ISR setup as well -Ben)
	i2cInit();	// I2C initialization
	SSD1306Init();	// OLED Initialization

	
	sei(); //enable interrupts
	

	
	clear();	// Clear display buffer
	show();		// Send data/command to OLED 
	
	while(1)
	{	//default Program
		if(menuState==0){
			drawRunning();
			programPause = 1;
						
		} // End menuState 0
		
		// Pause Screen
		if(menuState==1){
			if(rampDownSet==0){
			drawPause(34);
			programPause = 0;				
			}else if(rampDownSet==1){
				clear();
				drawString(24,24, "Complete");
				show();	
				programPause = 0;
			}
		} // End menuState 1
		
		// Ramp Down
		if(menuState==2){
			
			clear(); //to clear display buffer
			drawString(12,16, "Ramp Down");
			show();
			
			//When finished Ramp Down
			mTimer(10); // Remove, placeholder to simulated that sorting has finished
			menuState=1;
			
		} // End menuState 2
		
		// Calibrate Menu
		if(menuState==3){
			programPause = 1;
			clear();
			drawCalibrateADC();
			drawSelectBar(barState);
			show();
						
			if(right_key_press){
				right_key_press = 0; 
				barState=barState+1;
				if(barState > 4) {
					barState = 1;
				}
			}
			
			if(left_key_press){
				left_key_press = 0;
				barState = barState -1;
				if(barState < 1) {
					barState = 4;
				}
			}		
		} // End menuState 3
		
		// Object Colour Calibration
		if(menuState==4){
			
			switch(barState)
			{
				// Draw Black
				case selBlack:
					if(up_key_press==1){
						up_key_press=0;
						blkCali++;
					}else if(down_key_press==1){
						down_key_press=0;
						blkCali--;
					}
					drawBlackCali(blkCali);
					break;
					
				// Draw White
				case selWhite:
					if(up_key_press==1){
						up_key_press=0;
						whtCali++;
					}else if(down_key_press==1){
						down_key_press=0;
						whtCali--;
					}				
					drawWhiteCali(whtCali);
					break;
					
				// Draw Aluminum
				case selAlum:
					if(up_key_press==1){
						up_key_press=0;
						almCali++;
					}else if(down_key_press==1){
						down_key_press=0;
						almCali--;
					}			
					drawAluminumCali(almCali);
					break;
					
				// Draw Steel
				case selSteel:
					if(up_key_press==1){
						up_key_press=0;
						stlCali++;
					}else if(down_key_press==1){
						down_key_press=0;
						stlCali--;
					}			
					drawSteelCali(stlCali);
					break;
					
			}// end switch
		}//end menuState 4
		
	} // End While Loop
	
}

void initTimer0 (void){ //initialize Timer 0 for CTC (Clear Timer on Compare) (NOT USED)
	/*set Waveform Generation mode to Clear Timer*/
	/*set WGM bits to 0100*/
	/*note WGM is spread over two registers*/
	TCCR0A |= _BV(WGM01);
	TCCR0B |= _BV(CS01)|_BV(CS00); // clk/64
	TIMSK0 |= _BV(OCIE0A);
	/*set Output Compare Register for 255 cycles (10ms)*/
	OCR0A = 0xFF;
	/*set the initial value of the Timer rCounter to 0x0000*/
	TCNT0 = 0x00;
}

void setupISR(void){
	EIMSK |=0b00110000; // Select INT5, INT4
	EICRB |= 0b00001010; //active low for INT5 and INT4
	
	PCICR |= _BV(PCIE0); // Enable PC Interrupt
	PCMSK0 |= 0b11101110; // Select PCINT7, PCINT6, PCINT5, PCINT3, PCINT2, PCINT1
	
}

void setupPorts(void){
	// Port Setup
	DDRC = 0xFF; //LEDs Debugging

	DDRB &= ~(_BV(PB7) | _BV(PB6) | _BV(PB5)|_BV(PB3)|_BV(PB2)|_BV(PB1)); // Clear pins PB7:5, PB3:1
	// PB7:5m,PB3:1  are now inputs
	DDRE &= ~(_BV(PE5) | _BV(PE4)); // Clear pins PE5:4
	// PE5, PE4 are now inputs
	
	PORTB |= _BV(PB7) | _BV(PB6) | _BV(PB5)|_BV(PB3)|_BV(PB2)|_BV(PB1);
	PORTE |= _BV(PE5) | _BV(PE4); 
}

// Timer0 Output Compare A Interrupt (NOT USED)
ISR(TIMER0_COMPA_vect){
	PORTC ^= _BV(PC6);	
}

// Joystick - Right
ISR(INT4_vect){ //on PD3; active low
	right_key_press=1;
}

// Joystick - Down
ISR(INT5_vect){ //on PE4; active low
	//PORTC ^= _BV(PC6);
	down_key_press=1;
}

ISR(PCINT0_vect){
	uint8_t changedbits;

	changedbits = PINB ^ portbhistory;
	portbhistory = PINB;

	// Joystick UP - PORTB7
	if(changedbits & (1 << PB7))
	{
		up_key_press=1;
	}
	
	// Joystick LEFT - PORTB6
	if(changedbits & (1 << PB6))
	{				
		left_key_press=1;
	}
	
	// Joystick SELECT - PORTB5
	if(changedbits & (1 << PB5))
	{
		//PORTC ^= _BV(PC3);
		if(menuState==3) menuState = 4;	
		
	}

	
	// Calibrate Screen Button - PORTB3
	if(changedbits & (1 << PB3))
	{
		menuState = 3;
	}
	
	// Ramp Down - PORTB2
	if(changedbits & (1 << PB2))
	{
		rampDownSet = 1;
		menuState = 2;
	}
	
	// Pause/Resume Button - PORTB1
	if(changedbits & (1 << PB1))
	{
		if(programPause==0){
			menuState = 0;
		} else if(programPause==1){
			menuState = 1;
		}
	}
	
}// End PCINT0
