/*
########################################################################
# MILESTONE : Final Project
# PROGRAM : 6: Automated Sorting
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
#define AL_REFLECTIVITY 800 //minimum reflectivity of aluminum
#define FE_REFLECTIVITY 400 //minimum reflectivity of steel
#define WH_REFLECTIVITY 800 //minimum reflectivity of white plastic
#define BL_REFLECTIVITY 400 //minimum reflectivity of black plastic

/*Global Variables*/
volatile unsigned int ADCResult; //8 bits: 0 => (2^9-1); stores result of ADC conversion
volatile unsigned int ADCResultFlag; //8 bits: 0 => (2^9-1); thats that ADC conversion is complete
volatile unsigned int HallEffect; //becomes set during stepper homing to know position
volatile unsigned int opt1Flag; //set when 1st optical sensor is triggered (OI sensor)
volatile unsigned int opt2Flag; //set when 2nd optical sensor is triggered (OR sensor)
volatile unsigned int optExitFlag; //object is at end of conveyor
volatile unsigned int inductiveFlag; //an inductive flag is picked up
unsigned int stepperSigOrd[4] = {0b00110110,0b00101110,0b00101101,0b00110101};

/* Main Routine */
int main(int argc, char *argv[]){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	/*User Variables*/
	int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0x00;
	int stepperMovement = 0x00;
	uint8_t oldADCResult = 0x00;
	int OI_Count = 0x00; //count of objects that have hit optical sensor 1 (OI)
	int RL_Count = 0x00; //count of objects that have had their reflectivities quantified
	int OR_Count = 0x00; //count of objects that have hit optical sensor 2 (OR)
	int EX_Count = 0x00; //count of objects that have hit optical sensor 3 (EX)
	int OIOR_Count = 0x00; //count of objects between optical sensors 1 and 2
	uint16_t tempReflArray[8];
	uint8_t tempType=0;
	uint8_t tempFerrous=0;
	//initialize structure to store material characteristics
	typedef struct material{
		uint16_t reflectance; //10 bit value
		int type; //black=0;;aluminum=50;white=100;steel=150
		uint8_t inductive; //states whether object is ferrous or not (0=>non-ferrous;1=>ferrous)
		}material;
	//initialize array of structures for storage	
	struct material materialArray[64];
	/*initializations*/
	cli(); //disable interrupts
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM setup;
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
	// PORTB &= 0b1110000; //apply Vcc brake to motor
	//PORTB |=0b1000; //start motor in specified direction
	HallEffect=0x00; //set HallEffect equal to zero so while loop is continuous until break out
	stepperHome(&stepperPosition,&stepperIteration);
	
	/*initialize flags and counters*/
	opt1Flag=0x00;
	opt2Flag=0x00;
	inductiveFlag=0x00;
	optExitFlag=0x00;
	
	motorControl(CONVEYOR_SPEED,DC_FORWARD);//conveyor forward (counter-clock-wise)
	while(1){
		if(opt1Flag){
			opt1Flag=0x00; //reset flag
			ADCSRA |= _BV(ADSC); //initialize the ADC
			OI_Count+=1; //add one to amount of objects unsorted
			OIOR_Count+=1;
		} 
		if(ADCResultFlag && OIOR_Count>0){ //if an ADC conversion is complete and there is an object between optical sensors 1 and 2
			ADCResultFlag=0; //reset flag
			if(ADCResult>oldADCResult){ //reflectivity is increasing still
				oldADCResult=ADCResult;
			}else if((ADCResult<0x04) && (ADCResult<oldADCResult)){ //minimal to no reflection AND reflectivities have been reducing	
				tempReflArray[RL_Count]=oldADCResult;//value of oldADCResult is added to a temporary array
				RL_Count+=1;//add one to amount of objects that have had their reflectivities measured
				oldADCResult=0x00;//reset oldADCResult to 0 for the next objects reflectivites to be measured
			}
			ADCSRA |= _BV(ADSC); //re-trigger ADC
		} else ADCResultFlag=0; //reset flag
		if(opt2Flag){
			opt2Flag=0x00; //reset flag
			if(inductiveFlag){ //object is metal: aluminum (light), steel (dark)
				inductiveFlag=0x00;
				tempFerrous=1;
				//based on reflectivity, make an objective decision
				if (tempReflArray[OR_Count]>AL_REFLECTIVITY){
					tempType=150;//object is aluminum
					} else {
					tempType=50;//object is steel
				}
			} else { //object is plastic: white (light), black (dark)
				tempFerrous=0;
				if (tempReflArray[OR_Count]>WH_REFLECTIVITY){
					tempType=100;//object is white plastic
					} else {
					tempType=0;//object is black plastic
				}
			}
			//add reflectivity, object type (black=0;aluminum=50;white=100;steel=150), and ferrousity (made-up word ':)') of object to structure
			materialArray[OR_Count].reflectance=tempReflArray[OR_Count];//unneccesary, but for completeness it exists
			materialArray[OR_Count].inductive=tempFerrous;
			materialArray[OR_Count].type=tempType;
			OR_Count+=1;
			OIOR_Count-=1; //decrement count of objects between optical sensors 1 and 2
		}
		if(optExitFlag){ //object has hit sensor at end of conveyor
			optExitFlag=0x00; //reset flag
			//corresponding positions (black=0;aluminum=50;white=100;steel=150)
			//if object type matches stepper location; do nothing...
			stepperMovement=stepperPosition-materialArray[EX_Count].type;
			if (stepperMovement!=0){//if object type doesn't match stepper location; stop motor, move stepper, start motor
				PORTB &=0xF0; //Apply Vcc brake to motor
				//stepper rotation logic
				if (stepperMovement==150) stepperMovement=-50;
				else if (stepperMovement==-150) stepperMovement=50;
				else if (stepperMovement== 100) stepperMovement=-100; //counter-clockwise is more efficient for particular stepper
				stepperControl(stepperMovement, &stepperPosition, &stepperIteration);//rotate stepper to proper location
				PORTB |=0b00001000; //start motor forwards
			}		
			EX_Count+=1;
		}
	//efficient modulus for counters; forces them to stay within 0->63 as struct array only has 64 places
	OI_Count &= 0b00111111;
	RL_Count &= 0b00111111;
	OR_Count &= 0b00111111;
	EX_Count &= 0b00111111;
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos, int *stepperIt){
	/*function variable declarations*/
	int i=0;
	int k=0;
	uint8_t maxDelay = 22; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 12; //5ms corresponds to 200 steps per second; or 1 revolution per second
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
	uint8_t delay = 20; //20ms corresponds to 50 steps per second
	int i=0;
	int x=0;
	uint8_t offset=8; //arbitrary at this point
	uint8_t DIRECTION=1; //1 for clockwise, -1 for counter-clockwise
	PORTA=0x00;
	while (HallEffect==0){
		PORTA = stepperSigOrd[i];
		mTimer(delay);
		i++;
		if (i==4)i=0;
	}
	EIMSK&=0b10111111;//disable hall effect sensor interrupt (INT6)
	/*Insert code here to compensate for offset*/
	for (x=0;x<offset;x++){
		i+=DIRECTION;
		if (i==4)i=0;
		if (i==-1)i=3;
		PORTA = stepperSigOrd[i];
		mTimer(delay);
	}
	//
	*stepperIt = i;//modulus is heavy in terms of computation, but doesn't matter in this function
	*stepperPos = 0; //base stepper position (on black)
}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	TCCR0B |= _BV(CS01) | _BV(CS00);//Set clock pre-scalar (8MHz*1/64): 488Hz measured on PB7*
	//TCCR0B &= 0b11111101;
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
	PORTB &= 0xF0; //Apply Vcc brake to conveyor
}
void setupISR(void){
	/*INT(7:4) => PE(7:4); INT(3:0) => PD(3:0)*/
	//rising edge on INT2: EICRA |= _BV(ISC21) | _BV(ISC20);
	//falling edge on INT2: EICRA |= _BV(ISC21);
	EIMSK |=0b01011111; //initialize INT6,4:0
	EICRA |= 0b10111010; //rising edge on INT2; falling edge detection on INT0
	EICRB |= 0b00100010; //active low for INT6 and INT4
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(MUX0); //AVcc reference (3.3V);read from ADC 1;output left-adjusted
	ADMUX &= 0b11100001; //reading from PF1 (ADC1); ADC0 works, but MCU has thermistor on pin...
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
ISR(INT0_vect){ // on PD0; active low KILL SWITCH
	PORTB &= 0b11110000; //stop motor by applying Vcc break
}
/*sensor 1: OI: 1st Optical-Inductive-Near Reflective sensor*/
ISR(INT1_vect){ // on PD1; active low
	opt1Flag=0x01;
}
/*sensor 3: OR: 2nd Optical-Reflective-Near Inductive sensor*/
ISR(INT2_vect){ // on PD2; active high
	opt2Flag=0x01;
}
/*sensor 4: IN: Inductive sensor*/
ISR(INT3_vect){ //on PD3; active low
	inductiveFlag=0x01;
}
/*sensor 5: EX: 3rd Optical-Near exit of conveyor*/
ISR(INT4_vect){ //on PE4; active low
	optExitFlag=0x01;
}
/*sensor 6: HE: Hall Effect sensor; used for homing stepper*/
ISR(INT6_vect){ //on PE6; Active low for hall effect sensor 
	HallEffect=0x01;
}

/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	ADCResult = ADCL;
	ADCResult += ADCH << 8;
	ADCResultFlag = 1;
}





