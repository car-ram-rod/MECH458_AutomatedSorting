/*
 * REFLECTIVITIES.c
 * Desc: To determine reflectivity minimimums for all four types of materials
 * These materials are Black (plastic), White (plastic), Aluminum (metal), Steel (metal)
 * In final project demonstration the lab tech will cover one of the plastic pieces with duct
 * tape most likely to mess up the reflectivity.
 *		Solutions: 1) test a metal object with duct tape around it to determine the ADC result
		2) use inductive sensor, which will pick up steel (ferrous material) quite well. Al is non-ferrous
 * Operation Options:
	1) Create secondary timer that triggers ADC conversion if the flag has been reset in main
	2) Trigger ADC conversion manually inside main every time the flag is reset.
	*have button that resets oldADCResult to keep testing
	*read from joystick on board to increase or decrease conveyor speed for optimization purposes
 * Created: 3/1/2018 8:03:13 AM
 * Author : Owen
 */ 
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <avr/interrupt.h>
#include <math.h>
/*Function Declarations*/
void setupPWM(int motorDuty);
void setupISR(void);
void setupADC(void);
void motorControl(int s, uint8_t d);//accepts speed and direction:speed range (0->100), direction possibilities {0b11,0b10,0b01,0b00}
void initTimer1(void);
void mTimer(int count);
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
volatile unsigned char ADC_RESET;

/*Beginning of main program*/
int main(void){
	uint8_t oldADCResult=0x00;
	cli(); //disable all interrupts
	initTimer1();
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM
	setupISR(); //interrupt initializations
	setupADC(); //ADC initializations
	DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRF = 0x00; /*PF1=ADC1 pin*/
	sei(); //enable all interrupts
	PORTB &= 0b11110000; //apply brake to Vcc
	motorControl(CONVEYOR_SPEED,DC_FORWARD); //start conveyor towards stepper
	ADCSRA |= _BV(ADSC); //initialize the ADC, start one conversion at the beginning
	while (1){
		if ((ADCResultFlag!=0) && (ADCResult>oldADCResult)){
			oldADCResult=ADCResult;
			PORTC=oldADCResult;
			ADCResultFlag=0x00;
			ADCSRA |= _BV(ADSC);
		}
		//mTimer(10) //--ODA Edit;Does mTimer break with other interrupts engaged?
		if (ADC_RESET){
			oldADCResult=0x00;
			ADC_RESET=0x00;
		}
	}
	return (0); //This line returns a 0 value to the calling program
}
void setupPWM(int motorDuty){
	uint8_t dutyCycle = 0;
	/*DC MOTOR PWM SETUP (runs conveyor)*/
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1); /*set to Fast PWM; OCRx updated at TOP; TOV set on MAX; Clear OC0A on Compare Match, set OC0A at TOP*/
	TCCR0B |= _BV(CS01);//Set clock pre-scalar (1MHz*1/8): 488Hz measured on PB7*
	dutyCycle = motorDuty*2.55;
	OCR0A = dutyCycle;//set duty cycle/start motor
	PORTB &= 0xF0; //Apply Vcc brake to conveyor
}
void setupISR(void){
	/*INT(7:4) => PE(7:4); INT(3:0) => PD(3:0)*/
	//EIMSK |= _BV(INT2); //enable INT2
	//EICRA |= _BV(ISC21) | _BV(ISC20); //rising edge interrupt; EICRA determines INT3:0
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
void initTimer1 (void){ //initialize Timer 1 for CTC (Clear Timer on Compare)
	/*set Waveform Generation mode to Clear Timer*/
	/*set WGM bits to 0100*/
	/*note WGM is spread over two registers*/
	TCCR1B |= _BV(WGM12);
	/*set Output Compare Register for 1000 cycles (1ms)*/
	OCR1A = 0x03E8;
	/*set the initial value of the Timer rCounter to 0x0000*/
	TCNT1 = 0x0000;
}
void mTimer(int count){ // delay microsecond
	int i = 0; //initialize loop counter
	/*Enable the output compare interrupt enable*/
	//TIMSK1 = TIMSK1 | 0b00000010; // --ODA edit: becomes
	/*initialize timer 1 to run at CPU clock (1MHz)*/
	TCCR1B |= _BV(CS10);
	/* Clear the timer interrupt flag and begin timer */
	TIFR1 |= _BV(OCF1A);
	while (i<count){
		if ((TIFR1 & 0x02) == 0x02){
			//clear interrupt flag by WRITING a ONE to the bit
			TIFR1 |= _BV(OCF1A);
			i++; //increment loop counter
			/*Timer resets automatically due to WGM settings*/
		} //if end
	}//while end
	TCCR1B &= 0b11111000; //shut off timer 1
	return;
} //mTimer

/**********INTERRUPT SERVICE ROUTINES**********/
/*sensor 3: 2nt Optical Inductive, Active HIGH starts AD conversion*/
ISR(INT2_vect){ //unused --ODA
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	ADCSRA |= _BV(ADSC);
}
ISR(ADC_vect){ //ADCResult is left-adjusted (i.e. the upper most byte is taken; 2 LSB' are discarded)
	ADCResult = ADCH;
	ADCResultFlag = 1;
}
ISR(INT6_vect){
	int i;
	ADC_RESET=1;
	//bad practice, but good for demonstration purposes
	for(i=0;i<1000;i++){}//arbitrary delay (allow button to settle)
	//mTimer(25);
	while((PINE & 0b01000000)==0b01000000){ //while switch is still pressed
		for(i=0;i<1000;i++){}//arbitrary delay
	}
}
