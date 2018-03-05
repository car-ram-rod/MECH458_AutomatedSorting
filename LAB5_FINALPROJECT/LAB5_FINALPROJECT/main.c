/*
########################################################################
# MILESTONE : Final Project
# PROGRAM : 5: Automated Sorting
# PROJECT : Lab5:
# GROUP : X
# NAME 1 : Owen, Anderberg, V00862140
# NAME 2 : Ben, Duong, V00839087
# DESC : Automated sorting of cylindrical objects using sensors, DC and Stepper motors
# DATA
# REVISED 2018-FEB-9
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <avr/interrupt.h>
#include <math.h>
#include "LinkedQueue.h"
#include "interrupt.h"

/*Function Declarations*/
void stepperControl(int steps,int *stepperPos,int *stepperIt);
void stepperHome(int *stepperPos,int *stepperIt);
void setupPWM(int motorDuty);
void setupISR(void);
void setupADC(void);
void motorControl(int s, uint8_t d);//accepts speed and direction:speed range (0->100), direction possibilities {0b11,0b10,0b01,0b00}

/*User Defines*/
#define highNibMask 0xF0
#define lowNibMask 0x0F
#define DC_REVERSE 0x02 	//dc motor clock-wise
#define DC_FORWARD 0x01	//dc motor counter-clockwise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 30 //50 is maximum for sustainability

/*Global Variables*/
volatile unsigned char ADCResult; //8 bits: 0 => (2^9-1); stores result of ADC conversion
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1); thats that ADC conversion is complete
volatile unsigned char HallEffect; //becomes set during stepper homing to know position
unsigned int stepperSigOrd[4] = {0b00110010,0b00010110,0b00101001,0b00010101};

/* Main Routine */
int main(int argc, char *argv[]){
	/*User Variables*/
	//uint8_t stepperSpeed = 0x00;
	//uint8_t stepperDirection = 0x00; //greater than 0 => clockwise (CW); 0 => counter-clockwise (CCW)
	int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0b00001101;
	//uint8_t motorDirection = 0b00;
	//uint8_t LEDisplay = 0x00;

	/*initializations*/
	cli(); //disable interrupts
	/*initialize clock to 8MHz*/
	CLKPR = _BV(CLKPCE);
	CLKPR = 0;
	
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM;
	setupISR();
	setupADC();
	initTimer1();
	/*Port I/O Definitions*/
	DDRA = 0xFF; /* Sets all pins on Port A to output: stepper motor control */
	/*stepper motor connections to MCU: PA5:0 = EN0, L1, L2, EN1, L3, L4*/
	DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRE = 0x00; /*PE4=HallEffect for stepper*/
	DDRF = 0x00; /*PF1=ADC1 pin*/

	
	sei(); //enable interrupts
	TCCR1B |= _BV(CS10);//Sets timer 1 to run at CPU clock, disable all function and use as pure timer
	ADCSRA |= _BV(ADSC); //initialize the ADC, start one conversion at the beginning

	// PORTB &= 0b0000; //start motor in specified direction
	//PORTB |=0b1000;
	HallEffect=0x00; //set HallEffect equal to zero so while loop is continuous until break out
	stepperHome(&stepperPosition,&stepperIteration);
	HallEffect=0x00;
	while(1){
		/*DC Motor Conveyor Test*/
		//motorControl(CONVEYOR_SPEED,DC_FORWARD);//forwards at 30%
		/*stepper function testing*/
		stepperControl(50,&stepperPosition,&stepperIteration); /*stepper function testing*/
		mTimer(1000);

		
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos,int *stepperIt){
	/*function variable declarations*/
	int i=0;
	uint8_t maxDelay = 20; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 10; //5ms corresponds to 200 steps per second; or 1 revolution per second
	uint8_t differential = maxDelay - minDelay;
	uint8_t delay = maxDelay;
	uint8_t offset = 0;
	char DIRECTION = 1;
	int CURRENT_ITERATION = 0;
	unsigned int absSteps = abs(steps); //compute absolute value now to save computations in "for" loop
	
	/*determine last known location of the stepper motor and apply an offset*/
	if (*stepperIt == stepperSigOrd[0]) offset=0;
	else if (*stepperIt == stepperSigOrd[1]) offset=1;
	else if (*stepperIt == stepperSigOrd[2]) offset=2;
	else if (*stepperIt == stepperSigOrd[3]) offset=3;
	//special case when not enough time to fully ramp-up; re-set minDelay
	if(absSteps<(differential*2)){
		minDelay=maxDelay-absSteps/2;
		differential = maxDelay - minDelay;
	}
	//determine direction
	if (steps > 0) DIRECTION = 1;// positive or clock-wise
	else if (steps < 0) DIRECTION = -1; //negative or counter-clock-wise
	else DIRECTION = 0;
	
	CURRENT_ITERATION = offset + DIRECTION;//saves some math later during "for" loop
	
	for(i=0;i<absSteps;i++){
		//ramp up
		if((absSteps-i) > differential){
			if(delay>minDelay)delay -= 1;
			else delay = minDelay;
			} else { //ramp down if the amount of steps left
			if(delay<maxDelay)delay += 1;
			else delay = maxDelay;
		}
		/*determine direction and then iterate through stepper signals in correct direction*/
		PORTA = stepperSigOrd[(CURRENT_ITERATION+DIRECTION*i)%4];
		//PORTC = stepperSigOrd[(CURRENT_ITERATION+DIRECTION*i)%4];
		mTimer(delay);
	}
	
	*stepperIt=stepperSigOrd[(CURRENT_ITERATION+DIRECTION*(i-1))%4]; //set value of current iteration to variable address
	*stepperPos += steps;
	*stepperPos %= 200; //represents 200 (0->199) steps of stepper positioning in a circle
	
	return; //returns nothing
}
void stepperHome(int *stepperPos,int *stepperIt){
	uint8_t delay = 30; //20ms corresponds to 50 steps per second
	int i=0;
	PORTA=0x00;
	while (HallEffect==0){
		PORTA = stepperSigOrd[i%4];
		mTimer(delay);
		i++;
	}
	/*Insert code here to compensate for offset*/
	//
	//	
	*stepperPos=0; //base stepper position (on black)
	*stepperIt = stepperSigOrd[(i-1)%4]; //remember current setting of stepper
}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	//TCCR0B &= 0b11110111;//WGM02 set to 0; (_BV(2) => 0x01 << 2)
	//TIMSK0 |= _BV(1);//enable interrupt for execution upon compare match in Timer/Counter 0; UNNEEDED due to sei(); above
	//TCCR0A &= 0b10111111;
	TCCR0B |= _BV(CS01) | _BV(CS00);//Set clock pre-scalar (8MHz*1/64): 488Hz measured on PB7*
	//TCCR0B &= 0b11111101;
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
	PORTB &= 0xF0; //Apply Vcc brake to conveyor
}
void setupISR(void){
	/*INT(7:4) => PE(7:4); INT(3:0) => PD(3:0)*/
	EIMSK |= (_BV(INT6) | _BV(INT2)) | (_BV(INT0)); //enable INT6, INT2 and INT0
	EICRA |= _BV(ISC21) | _BV(ISC20) | _BV(ISC01) | _BV(ISC00); //rising edge interrupt; EICRA determines INT3:0
	EICRB |= _BV(ISC61); //falling edge for INT6 Hall Effect; EICRB determines INT7:4
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(MUX0) | _BV(ADLAR); //AVcc reference (3.3V);read from ADC 1;output left-adjusted
	ADMUX &= 0b11100001; //reading from PF1 (ADC1); ADC0 works, but MCU has thermistor on pin...
	//PORTF &= 0b11111110;
}
void motorControl(int s, uint8_t d){//note that DC motor driver expects inverted bits
	uint8_t dutyCycle = 0;
	static uint8_t oldDirection;
	if((oldDirection & 0b00000011) != d){ //if current direction doesn't match new direction
		PORTB &= 0b11110000; //apply Vcc Brake
		PORTB |= ((~d & 0b11) << 2); //start motor in specified direction
		oldDirection=d;
	}
	dutyCycle = s*2.55;
	OCR0A = dutyCycle;//set duty cycle
}

/**********INTERRUPT SERVICE ROUTINES**********/
/*Button interrupt for emergency: shut-off dc motor, disable stepper, shut off, ensure nothing can be turned on*/

ISR(INT0_vect){ // on PD0; KILL SWITCH
	PORTB &= 0b11110011; //stop motor	
}

ISR(INT1_vect){ // on PD0; KILL SWITCH
	PORTB &= 0b11110011; //stop motor	
}

/*sensor 3: 2nt Optical Inductive, Active HIGH starts AD conversion*/
ISR(INT2_vect){
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	ADCSRA |= _BV(ADSC);
}

ISR(INT6_vect){ //Active low for hall effect sensor on PE6
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	HallEffect=0x01;
}

/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	/*ADLAR not set=0*/
	//ADCResult = ADCL;
	//ADCResult += ADCH << 8;
	/*ADLAR set=1; reduced resolution (lose two least significant bits), but more efficient*/
	ADCResult = ADCH;
	ADCResultFlag = 1;
}





