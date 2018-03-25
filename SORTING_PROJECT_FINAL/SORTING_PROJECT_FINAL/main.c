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
#include "interrupt.h"
#include "SSD1306.h"
#include "FrameBuffer.h"
#include "I2C.h"
#include "glcdfont.c"
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
#define DC_REVERSE 0x01 	//dc motor clock-wise
#define DC_FORWARD 0x02	//dc motor counter-clockwise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 35 //50 is maximum for sustainability
//#define AL_REFLECTIVITY 300 //minimum reflectivity of aluminum (largest number measured is around 100)
//#define FE_REFLECTIVITY 700 //minimum reflectivity of steel (largest number measured around 500)
//#define WH_REFLECTIVITY 955 //minimum reflectivity of white plastic (highest number around 945-955)
//#define BL_REFLECTIVITY 990 //minimum reflectivity of black plastic (lowest number measured around 970)
/*Global Variables*/
volatile unsigned int ADCResult; //8 bits: 0 => (2^9-1); stores result of ADC conversion
//volatile unsigned int systemFlag; //bits(4:0) = {ADCResultFlag,optExitFlag,opt2Flag,inductiveFlag,opt1Flag}
//volatile unsigned int stepEarlyCount;
volatile unsigned int lowADC;
volatile unsigned int ADCFilterCount;
volatile unsigned int lowADCArray[4];
//volatile unsigned int lowADCArray[8];
//volatile unsigned int lowADCArray[16];
//volatile unsigned int lowADCArray[32];
volatile unsigned int inductiveArray[64];
volatile unsigned int typeArray[64];
volatile unsigned int reflectivityArray[64];
volatile unsigned int ADCAverage; //needs to be able to hold a maximum of 0x2000
volatile int is;
volatile unsigned char ADCCompleteFlag; //allows the ADC conversions to stop if no object is in front of RL sensor
volatile unsigned int OI_Count; //count of objects that have hit optical sensor 1 (OI)
volatile unsigned int RL_Count; //count of objects that have had their reflectivities quantified
volatile unsigned int OR_Count; //count of objects that have hit optical sensor 2 (OR)
volatile unsigned int EX_Count; //count of objects that have hit optical sensor 3 (EX)
volatile int OIRL_Count = 0x00; //count of objects between optical sensor 1 and passed optical sensor 2
//volatile unsigned char opt1Flag; //set when 1st optical sensor is triggered (OI sensor)
volatile unsigned char inductiveFlag; //an inductive flag is picked up
//volatile unsigned int opt2Flag; //set when 2nd optical sensor is triggered (OR sensor)
volatile unsigned char optExitFlag; //object is at end of conveyor
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1); thats that ADC conversion is complete
volatile unsigned char pauseFlag; //corresponds to a system pause function
volatile unsigned char systemRampFlag; //corresponds to system ramp down function
//volatile unsigned char HallEffect; //becomes set during stepper homing to know position
unsigned int stepperSigOrd[4] = {0b00110110,0b00101110,0b00101101,0b00110101};
// Menu States
volatile uint8_t programPause = 0;
volatile uint8_t menuState = 0;
volatile uint8_t barState = 1;
volatile uint8_t rampDownSet = 0; //Might not need
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up
// Key Press State Variables
volatile uint8_t right_key_press = 0;
volatile uint8_t left_key_press = 0;
volatile uint8_t down_key_press = 0;
volatile uint8_t up_key_press = 0;
// Calibration Settings
volatile uint16_t blkCali = 990;
volatile uint16_t whtCali = 960;
volatile uint16_t almCali = 300;
volatile uint16_t stlCali = 700;
/* Main Routine */
int main(int argc, char *argv[]){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	/*User Variables*/
	int i=0x00; //solely used in for loops
	int stepperPosition = 0x00; //stepper position w.r.t. 360 degrees (circle); steps 0-200 => degrees 0-360
	int stepperIteration = 0x00;
	int stepperMovement = 0x00;
	int tempType = 0;
	uint8_t falseInductFlag=0x00;
	uint8_t BL_Count = 0x00;
	uint8_t WH_Count = 0x00;
	uint8_t ST_Count = 0x00;
	uint8_t AL_Count = 0x00;
	int RLEX_Count = 0x00; //count of objects that have had their reflectivity measured, but not reached sensor 3 (EX)
	uint8_t tempFerrous=0;
	uint8_t objTestCount=0;//variables used in menuState==5 to show min, max, and average "minimum" ADC values of objects
	uint16_t minADCTest=0xFFFF;
	uint16_t maxADCTest=0xFFFF;
	uint16_t aveADCTest=0;
	uint16_t ADCTestArray[8]={0};
	/*initializations*/
	cli(); //disable interrupts
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM setup;
	setupISR();
	setupADC();
	i2cInit();	// I2C initialization
	SSD1306Init();	// OLED Initialization
	clear();	// Clear display buffer
	show();		// Send data/command to OLED
	drawRunning();
	timer0Init();
	timer1Init();
	timer3Init();
	/*Port I/O Definitions*/
	DDRA = 0x3F; /* Sets pins 5:0 on Port A to output: stepper motor control */
		/*stepper motor connections to MCU: PA5:0 = EN0, L1, L2, EN1, L3, L4*/
		/*Hall effect sensor on PA7*/
	DDRB = 0x10; /*controls dc motor: PB4=PWM signal*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF3; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRE = 0x0F; /*interrupts on upper nibble;PE3:0={INA,INB,ENA,ENB}*/
	DDRF = 0x00; /*PF1=ADC1 pin*/	
	PORTB |= _BV(PB7) | _BV(PB6) | _BV(PB5)|_BV(PB3)|_BV(PB2)|_BV(PB1);//menu code
	PORTE |= _BV(PE5) | _BV(PE4); //menu code
	sei(); //enable interrupts
	/*initialize flags and counters */
	OR_Count=0;
	RL_Count=0;
	OI_Count=0;
	EX_Count=0;
	OIRL_Count=0;
	falseInductFlag=0;
	inductiveFlag=0x00;
	optExitFlag=0x00;
	ADCResultFlag=0x00;	
	//HallEffect=0x00; 
	//stepEarlyCount=0x00;
	PORTC=0b10101010;
	mTimer0(2000);
	PORTC=0b00001111;
	mTimer0(2000);
	stepperHome(&stepperPosition,&stepperIteration); //home stepper
	motorControl(CONVEYOR_SPEED,DC_FORWARD);//conveyor forward (counter-clock-wise)
	while(1){
		while(menuState==0){
			programPause = 1;
			if (inductiveFlag){ //triggered on a falling edge when a ferrous material is in front of inductive sensor
				if (falseInductFlag==0x00){
					falseInductFlag=0x01;
					TCCR3B |= _BV(CS30); //clock pre-scalar (clk/1); initialize clock counting
					TCNT3=0x00; //set timer equal to zero
					if ((TIFR3 & 0x01) == 0x01)TIFR3|=0x01; //if TOV3 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
				} //because of the closeness of interrupts OI and IN sensor, reliance on OI_Count w.r.t. inductive, delay given
				if ((TIFR3 & 0x01) == 0x01){ //if counter has overflowed ~>8ms; time to allow OI_Count to change
					inductiveFlag=0; //reset flag; allow flag to reset again after 8ms
					TCCR3B&=0b11111000; //disable timer 3
					falseInductFlag=0x00; //reset flag
					inductiveArray[(OI_Count-1)%64]=0x01; //set the actual current object to inductive=1; modulus of 64
				}
			} 
			if(ADCResultFlag){ //If the minimum reflectivity has been reached for an object
				ADCResultFlag=0; //reset flag
				//ADCAverage=lowADC;
				ADCAverage=0;
				//ADCAverage=lowADC;
				for(i=0;i<4;i++){
					ADCAverage+=lowADCArray[ADCFilterCount];
					ADCFilterCount++;
					ADCFilterCount&=0b00000011; //modulus of 8 with positive incrementing variables
				}
				/*for(i=0;i<8;i++){
					ADCAverage+=lowADCArray[ADCFilterCount];
					ADCFilterCount++;
					ADCFilterCount&=0b00000111; //modulus of 8 with positive incrementing variables
				}*/
				/*for(i=0;i<32;i++){
					ADCAverage+=lowADCArray[ADCFilterCount];
					ADCFilterCount++;
					ADCFilterCount&=0b00011111; //modulus of 8 with positive incrementing variables
				}*/
				/*for(i=0;i<16;i++){
					ADCAverage+=lowADCArray[ADCFilterCount];
					ADCFilterCount++;
					ADCFilterCount&=0b00001111; //modulus of 16 with positive incrementing variables
				}*/	
				ADCAverage>>=2; //division by 4 with chopping arithmetic
				//ADCAverage>>=3; //division by 8 with chopping arithmetic
				//ADCAverage>>=4; //division by 16 with chopping arithmetic
				//ADCAverage>>=5; //division by 32 with chopping arithmetic
				PORTC=ADCAverage&0x00FF;
				PORTD&=0x0F;
				PORTD|=((ADCAverage&0x0300)>>3);
				tempFerrous=inductiveArray[RL_Count]; //store whether object was ferrous or non-ferrous
				inductiveArray[RL_Count]=0x00; //reset inductive array to zero; otherwise, array will produce errors if more than 64 objects are sorted
				//sorting objects by reflectivity
				if (ADCAverage<300)typeArray[RL_Count]=150;//object is aluminum
				else if(ADCAverage<850)typeArray[RL_Count]=50;//object is steel
				else if(ADCAverage<960){
					typeArray[RL_Count]=100;//object is white
					//if(!tempFerrous)typeArray[RL_Count]=100;//object is white
					//else typeArray[RL_Count]=50;//object is steel when it is dark
				}
				else{//object is non-reflective
					typeArray[RL_Count]=0;//object is black plastic when no ferrous material exists
					//if(!tempFerrous) typeArray[RL_Count]=0;//object is black plastic when no ferrous material exists
					//else typeArray[RL_Count]=50;//object is steel when it is dark			 
				}
				RL_Count++;//add one to amount of objects that have had their reflectivity's measured
				RLEX_Count+=1;
				OIRL_Count-=1;			
				//ADCCompleteFlag=0x01; //set flag to tell system there is no ADC conversions occurring
			}
			if(optExitFlag){ //object has hit sensor at end of conveyor
				//corresponding positions (black=0;aluminum=50;white=100;steel=150)
				//counters are working properly with low pass filters
				//PORTC = EX_Count;
				//mTimer2(1000);
				//PORTC = OI_Count;
				//mTimer2(500);
				//PORTC = RL_Count;
				//mTimer2(500);
				tempType=typeArray[EX_Count];
				stepperMovement=stepperPosition-tempType;
				if (stepperMovement){//if object type doesn't match stepper location; stop motor, move stepper, start motor
					//PORTC=0b00111100;
					PORTE &=0xF0; //Apply Vcc brake to motor
					//stepper rotation logic; value of steps to rotate stepper is kept between 1:100)		
					if (abs(stepperMovement)>100){
						if (stepperMovement<0) stepperMovement+=200;
						else stepperMovement-=200;
					}
					stepperControl(stepperMovement, &stepperPosition, &stepperIteration);//rotate stepper to proper location
					PORTE |=0b00000100; //start motor forwards
				}
				if (tempType==0)BL_Count += 0x01;
				else if (tempType==50)ST_Count += 0x01;
				else if (tempType==100)WH_Count += 0x01;
				else if (tempType==150)AL_Count += 0x01;
				RLEX_Count-=1;
				EX_Count++;
				optExitFlag=0; //reset flag
			}
			if(OIRL_Count<=0)OI_Count=RL_Count; //OI optical sensor is unreliable;set equal to RL sensor count if no objects between sensors
			//efficient modulus for counters; forces them to stay within 0->63 as struct array only has 64 places
			OI_Count &= 0b00111111;//modulus of 64
			RL_Count &= 0b00111111;
			OR_Count &= 0b00111111;
			EX_Count &= 0b00111111;
		}//end of sort code
		if(menuState==1){//pause function
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
			//mTimer(10); // Remove, placeholder to simulated that sorting has finished
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
		if (menuState==5){
			objTestCount=0;
			while((menuState==5) && (objTestCount<8)){
				if(ADCResultFlag){ //If the minimum reflectivity has been reached for an object
					ADCResultFlag=0; //reset flag
					ADCAverage=0;
					for(i=0;i<4;i++){
						ADCAverage+=lowADCArray[ADCFilterCount];
						ADCFilterCount++;
						ADCFilterCount&=0b00000011; //modulus of 8 with positive incrementing variables
					}	
					ADCAverage>>=2; //division by 4 with chopping arithmetic
					ADCTestArray[objTestCount]=ADCAverage;
					if(objTestCount==0)maxADCTest=ADCAverage;//initialize max value found on first object
					if(ADCAverage<minADCTest)minADCTest=ADCAverage;
					if(ADCAverage>maxADCTest)maxADCTest=ADCAverage;					
					objTestCount++;
				}
			}
			aveADCTest=0;
			if(objTestCount==8){//if 8 objects have had reflectivity quantified, print result to screen
				for(i=0;i<8;i++) aveADCTest+=ADCTestArray[i];
				aveADCTest>>=3; //division by 8 with chopping
				//print minADCTest, maxADCTest, and aveADCTest to screen
			}
		}	
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}

/*function allows control of direction and quantity of steps to */
void stepperControl(int steps,int *stepperPos, int *stepperIt){
	/*function variable declarations*/
	int i=0; //step quantity
	int k=0; //timer counter
	uint8_t maxDelay = 15; //20ms corresponds to 50 steps per second
	uint8_t minDelay = 7; //5ms corresponds to 200 steps per second; or 1 revolution per second
	uint8_t differential = maxDelay - minDelay;
	uint8_t delay = maxDelay;
	int PORTAREGSet = *stepperIt;
	int DIRECTION = 1;
	uint16_t absSteps = abs(steps); //compute absolute value now to save computations in "for" loop
	if (steps > 0) DIRECTION = 1;// positive or clock-wise
	else if (steps < 0) DIRECTION = -1; //negative or counter-clock-wise
	else DIRECTION=0;		
	if(absSteps<(differential*2)){ //if there isn't enough time for stepper to fully ramp up to full speed
		minDelay=maxDelay-absSteps/2;
		differential = maxDelay - minDelay;
	}
	/*perform one stepper cycle before "for" loop so there is no wasted delay at beginning or end of stepper motion*/
	PORTAREGSet+=DIRECTION;
	if(PORTAREGSet==4)PORTAREGSet=0;
	if(PORTAREGSet==-1)PORTAREGSet=3;
	TCCR0B |= _BV(CS01); //clock pre-scalar (clk/8)
	TCNT0=0x00; //set timer equal to zero; note timer is already counting based on clock prescalar
	if ((TIFR0 & 0x01) == 0x01)TIFR0|=0x01; //if TOV0 flag is set to 1, reset it to zero
	PORTA = stepperSigOrd[PORTAREGSet];//initialize first step
	for(i=2;i<=absSteps;i++){	
		//ramp up
		if((absSteps-i) > (differential+1)){ //the "added" one causes it to slow down one step early
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
		while (k<(delay*4)){ //iterate through given count; multiplied by four as it is now a 250ms timer
			if ((TIFR0 & 0x01) == 0x01){ //if overflow has occurred in counter
				TIFR0|=0x01; //reset overflow flag by writing a 1 to TOV2 bit;equivalent => TIFR2 |= _BV(TOV2)
				k++;
			}
		}
		PORTA = stepperSigOrd[PORTAREGSet];//move stepper after first delay
	}
	TCCR0B&=0b11111000; //disable timer 0
	*stepperIt=PORTAREGSet;//set value of current iteration to variable address of stepperIteration Variable
	*stepperPos -= steps; //iterating through steps positively is clockwise; however, when moving clockwise position is decremented
	*stepperPos %= 200; //represents 200 (0->199) steps of stepper positioning in a circle
	return; //returns nothing
}
////--ODA: CHANGE SO NO INTERRUPT IS USED FOR HALL EFFECT, simply check for voltage on an input pin
void stepperHome(int *stepperPos, int *stepperIt){
	uint8_t delay = 30; //20ms corresponds to 50 steps per second
	int i=0;
	int x=0;
	uint8_t offset=2; //arbitrary at this point
	uint8_t DIRECTION=1; //1 for clockwise, -1 for counter-clockwise
	PORTA=0x00;
	while (PINA&0b10000000){ //Active low for hall effect sensor triggering
		PORTA = stepperSigOrd[i];
		mTimer0(delay);
		i++;
		if (i==4)i=0;
	}
	i--;
	/*Insert code here to compensate for offset */
	for (x=0;x<offset;x++){
		i+=DIRECTION;
		if (i==4)i=0;
		if (i==-1)i=3;
		PORTA = stepperSigOrd[i];
		mTimer0(delay);
	}
	*stepperIt = i;//set current stepper iteration
	*stepperPos=0; //base stepper position (on black)
}
/*initializing the dc motor*/
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR2A |= _BV(WGM20) | _BV(WGM21) | _BV(COM2A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	TCCR2B |= _BV(CS22);//Set clock pre-scalar (8MHz*1/64): 488Hz measured on PB4*
	dutyCycle = motorDuty*2.55;
	OCR2A = dutyCycle;//set duty cycle/start motor
	PORTE &= 0xF0; //Apply Vcc brake to conveyor
}
void setupISR(void){
	/*INT(7:4) => PE(7:4); INT(3:0) => PD(3:0)*/
	//Ex: rising edge on INT2: EICRA |= _BV(ISC21) | _BV(ISC20);
	//Ex: falling edge on INT2: EICRA |= _BV(ISC21);
	//see ISR routines interrupt functions
	EIMSK |= _BV(INT7) |_BV(INT6)|_BV(INT5)|_BV(INT4)|_BV(INT3)|_BV(INT2);//initialize INT 7,6,3,2
	EICRA |= _BV(ISC21) | _BV(ISC20) | _BV(ISC31);
	//EICRA |= 0b10110000; //rising edge trigger (active low) for OI (INT2); falling edge detection (active low) for IN (INT3)
	EICRB |= _BV(ISC71) | _BV(ISC61) | _BV(ISC60) | _BV(ISC51) | _BV(ISC41);
	PCICR |= _BV(PCIE0); // Enable PC Interrupt
	PCMSK0 |= 0b11101110; // Select PCINT7, PCINT6, PCINT5, PCINT3, PCINT2, PCINT1
	//EICRB |= 0b00001011; //rising edge trigger (active high) for OR (INT6); falling edge detection (active low) for EX (INT7)
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
		PORTE &= 0b11110000; //apply Vcc Brake
		PORTE |= ((~d & 0b11) << 2); //start motor in specified direction
		oldDirection=d;
	}
	dutyCycle = s*2.55;
	OCR0A = dutyCycle;//set duty cycle
}

/**********INTERRUPT SERVICE ROUTINES**********/

ISR(INT0_vect){ // on PD0; taken for LCD Screen
}
ISR(INT1_vect){ // on PD1; taken for LCD Screen
}
/*sensor 1: OI: 1st Optical-Inductive-Near Inductive sensor*/
ISR(INT2_vect){ // on PD2; active low; triggered on rising-edge
	if(menuState<2){ //only adds to counters if in sorting mode or pause mode
		OI_Count+=1;
		OIRL_Count+=1;	
	}
}
/*sensor 2: IN: Inductive sensor*/
ISR(INT3_vect){ //on PD3; active low; triggered on falling-edge
	inductiveFlag=0x01;
}
/*sensor 3: OR: 2nd Optical-Reflective-Near Reflective sensor*/
ISR(INT6_vect){ // on PD6; active high; triggered on rising-edge
	lowADC=0xFFFF;
	ADCSRA|= _BV(ADSC); //trigger ADC (i.e. begin ADC conversion)
	//OR_Count+=1;	
	//ADCCompleteFlag=0x00; // tell system ADC conversions are occurring
}
/*sensor 5: EX: 3rd Optical-Near exit of conveyor*/
ISR(INT7_vect){ //on PE7; active low; triggered on falling-edge
	optExitFlag=0x01;
}
/*ADC ISR: triggered when ADC is completed*/
ISR(ADC_vect){
	if (lowADC>ADC){ //if ADC result is still decreasing (i.e. if object's reflectivity is increasing)
		lowADC=ADC; //ADC holds the entire 10 bit value in a 16bit variable; lowADC set for future comparison
		lowADCArray[ADCFilterCount]=lowADC;
		ADCFilterCount++; //increment array location being set
		//ADCFilterCount&=0b00000111; //modulus of 8
		//ADCFilterCount&=0b00001111; //modulus of 16
		//ADCFilterCount&=0b00011111; //modulus of 32
		ADCFilterCount&=0b000000011; //modulus of 4
		//highByteADC=ADCH;
		//lowByteADC=ADCL;
	}
	if (PINE&0b01000000) ADCSRA|= _BV(ADSC); //if there is still an object in OR beam keep initializing ADC conversions
	//if (ADC<(lowADC+40)) ADCSRA|= _BV(ADSC); //if there is still an object keep initializing ADC conversions
	else ADCResultFlag = 1;
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
	if(changedbits & (1 << PB7)) up_key_press=1;
	// Joystick LEFT - PORTB6
	if(changedbits & (1 << PB6)) left_key_press=1;
	// Joystick SELECT - PORTB5
	if(changedbits & (1 << PB5)) if(menuState==3) menuState = 4;
	// Calibrate Screen Button - PORTB3
	if(changedbits & (1 << PB3)) menuState = 3;	
	// Ramp Down - PORTB2
	if(changedbits & (1 << PB2))
	{
		rampDownSet = 1;
		menuState = 2;
	}	
	// Pause/Resume Button - PORTB1
	if(changedbits & (1 << PB1))
	{
		if(programPause==0)menuState = 0; 
		else if(programPause==1) menuState = 1;
	}	
}// End PCINT0







