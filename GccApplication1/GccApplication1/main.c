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
#define DC_CW 0x02 	//dc motor clock-wise
#define DC_CCW	0x01	//dc motor counter-clockwise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 30 //50 is maximum for sustainability

/*Global Variables*/
volatile unsigned int ADCResult; //16 bits: 0 => (2^17-1)
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1)
volatile unsigned char HallEffect;
unsigned int stepperSigOrd[4] = {0b00110010,0b00010110,0b00101001,0b00001101};
	//int motorRotation[]= {0b00110000,0b00000110,0b00101000,0b00000101};

/* Main Routine */
int main(int argc, char *argv[]){
	/*User Variables*/
	//uint8_t stepperSpeed = 0x00;
	//uint8_t stepperDirection = 0x00; //greater than 0 => clockwise (CW); 0 => counter-clockwise (CCW)
	int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0b00001101;
	//uint8_t motorDirection = 0b00;
	uint8_t LEDisplay = 0x00;
	HallEffect=0x00;



	/*initializations*/
	cli(); //disable interrupts
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM;
	//setupISR();
	setupADC();
	
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
	
	/*code begins*/
	int motortest=0;
	int steppertest=1;
	// PORTB &= 0b0000; //start motor in specified direction
	//PORTB |=0b1000;
	stepperHome(&stepperPosition,&stepperIteration);
	HallEffect=0x00;
	while(1){
		/*DC Motor Conveyor Test*/
		
		if (motortest>0){
			for (int k=1; k<=10;k++){
				motorControl(k*10,DC_CW);//forwards?
				mTimer(300);
			}
			for (int k=9; k>=0;k++){
				motorControl(k*10,DC_CW);
				mTimer(200);
			}
			for (int k=1; k<=10;k++){
				motorControl(k*10,DC_CCW); //Backwards?
				mTimer(300);
			}
			for (int k=9; k>=0;k++){
				motorControl(k*10,DC_CCW);
				mTimer(200);
			}
		}
		if(steppertest>0){
			/*stepper function testing*/
			stepperControl(17,&stepperPosition,&stepperIteration); //~30 degrees
			stepperControl(33,&stepperPosition,&stepperIteration); //~60 degrees
			stepperControl(100,&stepperPosition,&stepperIteration); //180 degrees
			stepperControl(-100,&stepperPosition,&stepperIteration); //-180 degrees
			stepperControl(33,&stepperPosition,&stepperIteration); //~-60 degrees
			stepperControl(17,&stepperPosition,&stepperIteration); //~-30 degrees
		}
		/*stepper function testing*/
		//stepperControl(-100,&stepperPosition,&stepperIteration); /*stepper function testing*/
		/*If rising edge has been recognized*/
		if(ADCResultFlag){
			
			LEDisplay = (ADCResult/4);
			//PORTC = LEDisplay;
			ADCResultFlag = 0; //reset flag
			OCR0A = LEDisplay;
			ADCSRA |= _BV(ADSC); //re-initialize conversion (Feb 19, 2018: not connected to external interrupt)
		}
		/*stepper function testing*/
		//stepperControl(10,&stepperPosition,&stepperIteration);
		
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos,int *stepperIt){
	/*function variable declarations*/
	int i=0;
	uint8_t maxDelay = 20; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 12; //5ms corresponds to 200 steps per second; or 1 revolution per second
	uint8_t differential = maxDelay - minDelay;
	uint8_t delay = maxDelay;
	uint8_t offset = 0;
	uint8_t MODE = 0x00;
	unsigned int absSteps = abs(steps); //compute absolute value now to save computations in "for" loop
	/*determine last known location of the stepper motor and apply an offset*/
	if (*stepperIt == stepperSigOrd[0]) offset=1;
	else if (*stepperIt == stepperSigOrd[1]) offset=2;
	else if (*stepperIt == stepperSigOrd[2]) offset=3;
	else if (*stepperIt == stepperSigOrd[3]) offset=0;
	
	if (absSteps>=(differential*2)) MODE = 0x02;
	else if(absSteps<(differential*2)) MODE = 0x00;
	
	for(i=0;i<absSteps;i++){
		/*determine whether to ramp-up or ramp-down speed/delay*/
		if(MODE > 0x00){ //general case: stepper has enough 'steps' to fully ramp-up && ramp-down
			if((absSteps-i) > differential){
				if(delay>minDelay)delay -= 1;
				else delay = minDelay;
				}else{
				if(delay<maxDelay)delay += 1;
				else delay = maxDelay;
			}
			}else if(MODE == 0x00){ //special case: stepper doesn't have enough time to fully ramp-up or ramp-down
			if(i<=(absSteps/2)){
				if(delay>minDelay)delay -= 1;
				else delay = minDelay;
				}else{
				if(delay<maxDelay)delay += 1;
				else delay = maxDelay;
			}
		}
		/*determine direction and then iterate through stepper signals in correct direction*/
		if(steps > 0){
			PORTA = stepperSigOrd[(i+offset)%4];
			*stepperPos += 1;
			//PORTC = stepperSigOrd[(i+offset)%4]; //debugging
			mTimer(delay);
			}else if(steps < 0){
			PORTA = stepperSigOrd[((-1)*(i+offset))%4];
			*stepperPos -= 1;
			//PORTC = stepperSigOrd[((-1)*(i+offset))%4]; //debugging
			mTimer(delay);
		}
		*stepperPos %= 200; //represents 200 (0->199) steps of stepper positioning in a circle
		
	}
	/*better method would be to compare to Stepper Position*/
	*stepperIt = PINA & 0b00111111; //remember current setting of stepper
	PORTA &= 0b11011011; //disable stepper motion while leaving other

	/*stepper motor control signal order*/
	//PORTB = 0b00110010; //Step 1
	//mTimer(dekay);
	//PORTB = 0b00001110; //Step 2
	//mTimer(1000);
	//PORTB = 0b00101010; //Step 3
	//mTimer(1000);
	//PORTB = 0b00001101; //Step 4
	//mTimer(1000);
	return; //returns nothing
}
void stepperHome(int *stepperPos,int *stepperIt){
	uint8_t maxDelay = 20; //20ms corresponds to 50 steps per second
	int i=0;
	while ((PINA & 0b00010000)==0b00010000){//hall effect sensor on PB4
		PORTA = stepperSigOrd[i%4];
		PORTC = stepperSigOrd[i%4];
		mTimer(maxDelay);
		i++;
		i%=200;
	}
	/*Correct for offset*/
	//INSERT CODE HERE --ODA
	/*
	int i;
	for(i=0; i<101;i++){
		PORTA = stepperSigOrd[i%4];
		PORTC = stepperSigOrd[i%4];
		mTimer(maxDelay);
	}*/
	*stepperPos=0; //base stepper position
	*stepperIt = PINA & 0b00111111; //remember current setting of stepper
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
	EIMSK |= (_BV(INT4) | _BV(INT2)) | (_BV(INT0)); //enable INT2 and INT0
	EICRA |= _BV(ISC21) | _BV(ISC20) | _BV(ISC01) | _BV(ISC00); //rising edge interrupt
	EICRB |= _BV(ISC41); //falling edge for INT4 Hall Effect
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

ISR(INT0_vect){ // on PD0; KILL SWITCH
	PORTB &= 0b11110011; //stop motor
	
}

/*sensor 3: 2nt Optical Inductive, Active HIGH starts AD conversion*/
ISR(INT2_vect){
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	ADCSRA |= _BV(ADSC);
}

ISR(INT4_vect){ //Active low for hall effect sensor on PE4
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	HallEffect=0x01;
}

/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	ADCResult = ADCL;
	ADCResult += ADCH << 8;
	ADCResultFlag = 1;
}





