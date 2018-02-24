/*########################################################################
# MILESTONE : 4A
# PROGRAM : 1
# PROJECT : MyFirstProject
# GROUP : 5
# NAME 1 : Benjamin, Duong, V00839087
# NAME 2 : Owen, Anderberg, Student ID
# DESC : This program tests the functionality of the Stepper motor and the PWM output
# DATA
# REVISED
########################################################################
*/


/* Include Libraries */
#include <stdlib.h>
#include <avr/io.h>

/* Declarations */
void LEDTest(void);
void motorInitialization(void);
void mTimer(int count);
void PWMDriver(void);
void moveMotor(int direction, int degrees, int delay);
//void moveCW(int degrees, int delay);
//void moveCCW(int degrees, int delay);

/* Global Variables */
int currentStep = 0;
int motorRotation[]= {0b00110000,0b00000110,0b00101000,0b00000101};

int main(int argc, char *argv[]){	
	
	TCCR1B |= _BV(CS10);	// 1/8th pre-scaler (1MHz)
	DDRD = 0xFF;			// Set Port D to output bits
	DDRA = 0xFF;			// Set Port A to output bits
	DDRC = 0xFF;			// Set Port C to output bits
	
	LEDTest();

	PWMDriver();			// Runs PWM function
	
	motorInitialization();
	mTimer(2000);
	LEDTest();
	
	while(1)
	{
	
		moveMotor(0, 30, 20);
		mTimer(1000);
		moveMotor(0, 60, 20);
		mTimer(1000);
		moveMotor(0, 180, 20);
		mTimer(1000);
		moveMotor(1, 30, 20);
		mTimer(1000);
		moveMotor(1, 60, 20);
		mTimer(1000);
		moveMotor(1, 180, 20);
		mTimer(5000);
		
		LEDTest();
	}//end while loop
	
	
return(0);
}




//########## SUB ROUTINE TO INDICATE SOFTWARE LOAD SUCCESSS ##########
/* Turns on LEDs as a visual confirmation that program has loaded */
void LEDTest()
{
	PORTC = 0xFF;
	mTimer(150);
	PORTC = 0x00;
	mTimer(150);
	PORTC = 0xFF;
	mTimer(500);
	PORTC = 0x00;
	
}//end LEDTest function

//########## STEPPER MOTOR INITIALIZATION ########
void motorInitialization(){
	moveMotor(0, 121, 20);
}

//########## STEPPER MOTOR CONTROL ##########
void moveMotor(int direction, int degrees, int delay){
	
	int steps;
	steps = (degrees/1.8) + 0.5;				// Converts desired degree movement to appropriate steps
	if(direction == 0)
	{	
		for(int i=0; i<steps+1; i++)
		{
			PORTA = motorRotation[currentStep];
			PORTC = motorRotation[currentStep];		//Output to PORTC LEDs for visual indicator
			if(currentStep == 3)
			{
				currentStep=0;	// Check if step variable as reached the end of the motorRotation array, if yes, set step to 0
			}else currentStep++;					// Else increment through array
			mTimer(delay);
		}
		
	}else if(direction == 1)
	{		
		for(int i=0; i<steps; i++)
		{
			PORTA = motorRotation[currentStep];
			PORTC = motorRotation[currentStep];		//Output to PORTC LEDs for visual indicator
			if(currentStep == 0)
			{
				currentStep=3;
			}else currentStep--;
			mTimer(delay);
		}
		
	}//end else if
}


//########## CLOCK WISE STEPPER MOTOR CONTROL ##########
// NOT USED
void moveCW(int degrees, int delay)
{
	int steps;
	steps = (degrees/1.8) + 0.5;				// Converts desired degree movement to appropriate steps
	
	for(int i=0; i<steps+1; i++)
	{
		PORTA = motorRotation[currentStep];
		PORTC = motorRotation[currentStep];
		if(currentStep == 3) 
		{
			currentStep=0;	// Check if step variable as reached the end of the motorRotation array, if yes, set step to 0
		}else currentStep++;					// Else increment through array
		mTimer(delay);	
	}
	//PORTA = 0x00;
	//PORTC = 0x00;
}//end moveCW function

//########## COUNTER CLOCKWISE STEPPER MOTOR CONTROL ##########
// NOT USED
void moveCCW(int degrees, int delay)
{
	int steps;
	steps = (degrees/1.8) + 0.5;			//Converts desired degree movement to appropriate steps
	
	for(int i=0; i<steps; i++)
	{
		PORTA = motorRotation[currentStep];
		PORTC = motorRotation[currentStep];
		if(currentStep == 0) 
		{
			currentStep=3;
		}else currentStep--;
		mTimer(delay);	
	}
	//PORTA = 0x00;
	//PORTC = 0x00;
}//end moveCCW function

//########## TIMER FUNCTION ##########
void mTimer(int count)
{ 
	/* Variable Declarations */
	int i;
	i = 0;
	TCCR1B |= _BV(WGM12);
	OCR1A = 0x03e8;		//1000 cycles or 1ms
	TCNT1 = 0x0000;
	TIMSK1 = TIMSK1 |0b00000010;
	TIFR1 |= _BV(OCF1A);
	while (i < count){
		if ((TIFR1 & 0x02) == 0x02)
		{
			TIFR1 |= _BV(OCF1A);
			i++;
		}//end if
	}// while end
	return;
}//end mTimer function

//########## PWM CONTROL FUNCTION ##########
void PWMDriver()
{
			
	DDRB = 0xFF;				// Set Port B to output bits
		
	//Step 1
	TCCR0A |= _BV(WGM01);		// set WGM01 bits to 0B00000001
	TCCR0A |= _BV(WGM00);		// set WGM00 bits to 0B00000010
	
	//Step 2
	TIMSK0 |= _BV(OCIE0A);		// enable output compare interrupt enable
	
	//Step 3
	TCCR0A |= _BV(COM0A1); 		// Clear OC0A on Compare Match, set OC0A at TOP

	//Step 4
	TCCR0B |= _BV(CS01); 		// 1/8 pre-scaler (1MHz)

	//Step 5
	OCR0A = 0x7F; 				// Set up 50% duty cycle 

	//Step 6
	//DDRB = 0xFF				//Already done at beginning of function
	return;
}//end PWMDriver function
