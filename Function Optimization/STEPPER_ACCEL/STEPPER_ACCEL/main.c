/*
 * STEPPER_ACCEL.c
 *
 * Created: 3/1/2018 8:02:22 AM
 * Author : Owen
 */ 

/*
########################################################################
# MILESTONE : 4A
# PROGRAM : Stepper Control
# PROJECT : Automated Sorting
# GROUP : 7
# NAME 1 : Owen, Anderberg, V00862140
# NAME 2 : Ben, Duong, V00839087
# DESC : Controlling a stepper through a function with simple input
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
void stepperControl(int steps,int *stepperPos, int *stepperIt);
void stepperHome(int *stepperPos, int *stepperIt);
void setupPWM(int motorDuty);
void setupISR(void);
void setupADC(void);
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
volatile unsigned char HallEffect;
unsigned int stepperSigOrd[4] = {0b00110110,0b00101110,0b00101101,0b00110101};
	//int motorRotation[]= {0b00110000,0b00000110,0b00101000,0b00000101};

/* Main Routine */
int main(int argc, char *argv[]){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	/*User Variables*/
	//uint8_t stepperSpeed = 0x00;
	//uint8_t stepperDirection = 0x00; //greater than 0 => clockwise (CW); 0 => counter-clockwise (CCW)
	int stepperPosition = 0; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0;
	//uint8_t motorDirection = 0b00;
	HallEffect=0x00;

	/*initializations*/
	cli(); //disable interrupts
	//setupPWM(CONVEYOR_SPEED); //DC Motor PWM;
	setupISR();
	//setupADC();
	
	/*Port I/O Definitions*/
	DDRA = 0xFF; /* Sets all pins on Port A to output: stepper motor control */
	/*stepper motor connections to MCU: PA5:0 = EN0, L1, L2, EN1, L3, L4*/
	//DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	//DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRE = 0x00; /*PE6=HallEffect for stepper*/
	DDRF = 0x00; /*PF1=ADC1 pin*/
	initTimer1();
	timer2Init();
	sei(); //enable interrupts; currently breaks Timer 1...
	
	/*code begins*/
	
	mTimer2(1000);
	PORTC=0b11000000;
	mTimer2(1000);
	PORTC=stepperSigOrd[stepperIteration];
	mTimer2(1000);
	
	uint8_t LEDOutput=0b00000001;
	int i, k;
	//LED beginning sequence

	while(1){
		EIMSK|=_BV(INT6); //re-initialize Hall Effect interrupt
		for(i=0;i<1;i++){
			for(k=0;k<8;k++){
				PORTC = (LEDOutput << k);
				mTimer2(200);
			}
			for(k=6;k>=0;k--){
				PORTC = (LEDOutput << k);
				mTimer2(200);
			}
		}
		HallEffect=0x00;
		stepperHome(&stepperPosition, &stepperIteration);
		
		mTimer2(4000);
		/*stepper function testing*/
		stepperControl(17,&stepperPosition, &stepperIteration); //~30 degrees
		mTimer2(2000);
		stepperControl(33,&stepperPosition, &stepperIteration); //~60 degrees
		mTimer2(2000);
		stepperControl(100,&stepperPosition, &stepperIteration); //180 degrees
		mTimer2(2000);
		stepperControl(-100,&stepperPosition, &stepperIteration); //-180 degrees
		mTimer2(2000);
		stepperControl(-33,&stepperPosition, &stepperIteration); //~-60 degrees
		mTimer2(2000);
		stepperControl(-17,&stepperPosition, &stepperIteration); //~-60 degrees
		mTimer2(2000);
		stepperControl(-100,&stepperPosition, &stepperIteration); //~-30 degrees
		mTimer2(2000);
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos, int *stepperIt){
	/*function variable declarations*/
	int i=0;
	int k=0;
	uint8_t maxDelay = 15; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 7; //5ms corresponds to 200 steps per second; or 1 revolution per second
	//starts skipping steps at 5ms, 6ms is fine, but choosing 7ms for a safe buffer
	//really good combinations: maxDelay:minDelay => 15:7,
	uint8_t differential = maxDelay - minDelay;
	uint8_t delay = maxDelay;
	int PORTAREGSet = *stepperIt;
	int DIRECTION = 1;
	uint16_t absSteps = abs(steps); //compute absolute value now to save computations in "for" loop
	if(absSteps<(differential*2)){
		minDelay=maxDelay-absSteps/2;
		differential = maxDelay - minDelay;
	}
	//determine direction 
	if (steps > 0) DIRECTION = 1;// positive or clock-wise
	else if (steps < 0) DIRECTION = -1; //negative or counter-clock-wise
	
	/*perform one stepper cycle before "for" loop so there is no wasted delay at
	beginning or end of stepper motion*/
	PORTAREGSet+=DIRECTION;
	if(PORTAREGSet==4)PORTAREGSet=0;
	if(PORTAREGSet==-1)PORTAREGSet=3;
	TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32)
	TCNT2=0x00; //set timer equal to zero; note timer is already counting based on clock prescalar
	if ((TIFR2 & 0x01) == 0x01)TIFR2|=0x01; //if TOV2 flag is set to 1, reset it to zero
	PORTA = stepperSigOrd[PORTAREGSet];//initialize first step
	for(i=2;i<=absSteps;i++){
		
		//ramp up
		if((absSteps-i) > differential){ //the "added" negative one causes it to slow down one step early
			if(delay>minDelay)delay -= 1;
			else delay = minDelay;
		} else { //ramp down if the amount of steps left are less than the differential between max and min delays
			if(delay<maxDelay)delay += 1;
			else delay = maxDelay;
		}
		/*determine direction and then iterate through stepper signals in correct direction*/
		PORTAREGSet+=DIRECTION;
		if(PORTAREGSet==4)PORTAREGSet=0;
		if(PORTAREGSet==-1)PORTAREGSet=3;
		k=0; //reset counter for timer
		while (k<delay){ //iterate through given count
			if ((TIFR2 & 0x01) == 0x01){ //if overflow has occurred in counter
				TIFR2|=0x01; //reset overflow flag by writing a 1 to TOV2 bit
				k+=1;
				//equivalent; TIFR2 |= _BV(TOV2)
			}
		}
		PORTA = stepperSigOrd[PORTAREGSet];//move stepper after first delay
	}
	TCCR2B&=0b11111000; //disable timer 2
	*stepperIt=PORTAREGSet;
	//*stepperIt=stepperSigOrd[(CURRENT_ITERATION+DIRECTION*(i-1))%4]; //set value of current iteration to variable address
	*stepperPos += steps;
	*stepperPos %= 200; //represents 200 (0->199) steps of stepper positioning in a circle
	return; //returns nothing
}
void stepperHome(int *stepperPos, int *stepperIt){
	uint8_t delay = 30; //20ms corresponds to 50 steps per second
	int i=0;
	int x=0;
	uint8_t offset=2; //arbitrary at this point
	uint8_t DIRECTION=1; //1 for clockwise, -1 for counter-clockwise
	PORTA=0x00;
	while (!HallEffect){
		PORTA = stepperSigOrd[i];
		mTimer2(delay);
		i++;
		if (i==4)i=0;
	}
	i--;
	HallEffect=0x00;
	EIMSK&=0b10111111;//disable hall effect sensor interrupt (INT6) 
	/*Insert code here to compensate for offset --ODA CURRENTLY CAUSES MISSTEP... WHY?*/
	for (x=0;x<offset;x++){
		i+=DIRECTION;
		if (i==4)i=0;
		if (i==-1)i=3;
		PORTA = stepperSigOrd[i];
		mTimer2(delay);
	}

	//
	*stepperIt = i;//modulus is heavy in terms of computation, but doesn't matter in this function
	PORTA = stepperSigOrd[i];
	*stepperPos=0; //base stepper position (on black)

}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	//TCCR0B &= 0b11110111;//WGM02 set to 0; (_BV(2) => 0x01 << 2)
	//TIMSK0 |= _BV(1);//enable interrupt for execution upon compare match in Timer/Counter 0; UNNEEDED due to sei(); above
	//TCCR0A &= 0b10111111;
	TCCR0B |= _BV(CS01);//Set clock pre-scalar to No-Prescaling: 3.905kHz measured on PB7*
	//TCCR0B &= 0b11111101;
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
}
void setupISR(void){
	EIMSK |= _BV(INT6); //| _BV(INT2) | _BV(INT0); //enable INT6, INT2 and INT0
	//EICRA |= _BV(ISC21) | _BV(ISC20) | _BV(ISC01) | _BV(ISC00); //rising edge interrupt
	EICRB |= _BV(ISC61); //falling edge for INT6 Hall Effect
	//EICRB &= 0b11101111;
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(0); //AVcc reference (3.3V);read from ADC 1
	ADMUX &= 0b11100001; //reading from PF1 (ADC1); ADC0 works, but MCU has thermistor on pin...
	//PORTF &= 0b11111110;
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
/*Button interrupt for emergency: shut-off dc motor, disable stepper, shut off, ensure nothing can be turned on*/
/*
ISR(INT0_vect){ // on PD0; KILL SWITCH
	PORTB &= 0b11110011; //stop motor
	
}

//sensor 3: 2nt Optical Inductive, Active HIGH starts AD conversion
ISR(INT2_vect){
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	ADCSRA |= _BV(ADSC);
}
*/
ISR(INT6_vect){ //Active low for hall effect sensor on PE4
	HallEffect=0x01;
}
/*
//ADC ISR: triggered when ADC is completed
ISR(ADC_vect){
	ADCResult = ADCL;
	ADCResult += ADCH << 8;
	ADCResultFlag = 1;
	
}

*/

