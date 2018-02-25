/*
########################################################################
# MILESTONE : 4B
# PROGRAM : DC Motor and ADC
# PROJECT : Lab4B: DC Motor and ADC
# GROUP : 7
# NAME 1 : Owen, Anderberg, V00862140
# NAME 2 : Ben, Duong, V00839087
# DESC : Controlling a DC motor through ADC input
# DATA
# REVISED 2018-FEB-24
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <avr/interrupt.h>
#include <math.h>
//#include "LinkedQueue.h"
#include "interrupt.h"

/*Function Declarations*/
void setupPWM(int motorDuty);
void setupADC(void);
void setupISR(void);
void motorControl(int s, uint8_t d);//accepts speed and direction:speed range (0->100), direction possibilities {0b11,0b10,0b01,0b00}

/*User Defines*/
#define highNibMask 0xF0
#define lowNibMask 0x0F
#define DC_CW 0x02 	//dc motor clock-wise
#define DC_CCW	0x01	//dc motor counter-clockwise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 30 //50 is maximum for sustainability

/*Global Variables*/
volatile unsigned int ADCResult; //16 bits: 0 => (2^17-1)
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1)
volatile unsigned char SWITCH_DIRECTION;
unsigned int stepperSigOrd[4] = {0b00110010,0b00010110,0b00101001,0b00001101};
	//int motorRotation[]= {0b00110000,0b00000110,0b00101000,0b00000101};

/* Main Routine */
int main(int argc, char *argv[]){
	/*User Variables*/
	//uint8_t stepperSpeed = 0x00;
	//uint8_t stepperDirection = 0x00; //greater than 0 => clockwise (CW); 0 => counter-clockwise (CCW)
	//int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	//int stepperIteration = 0b00001101;
	//uint8_t motorDirection = 0b00;
	uint8_t LEDisplay = 0x00;
	SWITCH_DIRECTION=0x00;
	int i;


	/*initializations*/
	cli(); //disable interrupts
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM;
	setupADC();
	setupISR();
	
	/*Port I/O Definitions*/
	DDRA = 0xFF; /* Sets all pins on Port A to output: stepper motor control */
	/*stepper motor connections to MCU: PA5:0 = EN0, L1, L2, EN1, L3, L4*/
	DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRE = 0x00; /*PE6=HallEffect for stepper*/
	DDRF = 0x00; /*PF1=ADC1 pin*/

	
	sei(); //enable interrupts
	TCCR1B |= _BV(CS10);//Sets timer 1 to run at CPU clock, disable all function and use as pure timer
	ADCSRA |= _BV(ADSC); //initialize the ADC, start one conversion at the beginning
	
	PORTB &= 0b11110000; //apply brake to Vcc
	PORTB |=0b1000; //start motor counter-clockwise (i.e. conveyor is moving towards sort tray)
	/*code begins*/
	while(1){
		/*DC Motor Conveyor Test*/
		if(ADCResultFlag){			
			LEDisplay = (ADCResult/4);
			//PORTC = LEDisplay;
			ADCResultFlag = 0; //reset flag
			//PORTC = LEDisplay;
			OCR0A = LEDisplay; //send correct duty cycle to motor
			ADCSRA |= _BV(ADSC); //re-initialize conversion (Feb 19, 2018: not connected to external interrupt)
		}
		//check for input that direction should be changed
		if(SWITCH_DIRECTION>0){
			if ((PINB & 0b00001111)==0b00001000){
				PORTB &= 0b11110000; //apply brake to Vcc
				for(i=0;i<1000;i++){}//arbitrary delay
				PORTB |=0b00000100; //clockwise (i.e. conveyor is moving towards sort tray)
			} else {
				PORTB &= 0b11110000; //apply brake to Vcc
				for(i=0;i<1000;i++){}//arbitrary delay
				PORTB |=0b00001000; //counter-clockwise (i.e. conveyor is moving towards sort tray)
			}
			SWITCH_DIRECTION = 0; //reset flag
		}
		/*stepper function testing*/
		//stepperControl(10,&stepperPosition,&stepperIteration);
		
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	//TCCR0B &= 0b11110111;//WGM02 set to 0; (_BV(2) => 0x01 << 2)
	//TIMSK0 |= _BV(1);//enable interrupt for execution upon compare match in Timer/Counter 0; UNNEEDED due to sei(); above
	//TCCR0A &= 0b10111111;
	TCCR0B |= _BV(CS01);//Set clock pre-scalar to first prescaling: 488Hz measured on PB7*
	//TCCR0B &= 0b11111101;
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(0); //AVcc reference (3.3V);read from ADC 1
	ADMUX &= 0b11100001; //reading from PF1 (ADC1); ADC0 works, but MCU has thermistor on pin...
	//PORTF &= 0b11111110;
}
void setupISR(void){
	EIMSK |= _BV(INT6);  //enable INT6
	EICRB |= _BV(ISC61) | _BV(ISC60);//rising edge interrupt
}
void motorControl(int s, uint8_t d){
	uint8_t dutyCycle = 0;
	if(((PINB & 0b00001100) >> 2) != d){ //if current direction doesn't match new direction
		PORTB &= 0b11110011; //stop motor
		PORTB |= 0b0011 | ((d & 0b11) << 2); //start motor in specified direction
	}
	dutyCycle = s*2.55;
	OCR0A = dutyCycle;//set duty cycle
}

/**********INTERRUPT SERVICE ROUTINES**********/
/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	ADCResult = ADCL;
	ADCResult += ADCH << 8;
	ADCResultFlag = 1;
}
ISR(INT6_vect){
	int i;
	SWITCH_DIRECTION=1;
	for(i=0;i<1000;i++){}//arbitrary delay
	//bad practice, but good for demonstration purposes
	while((PINE & 0b01000000)==0b01000000){ //while switch is still pressed
		for(i=0;i<1000;i++){}//arbitrary delay
	}
}





