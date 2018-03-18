/*
 * REFLECTIVITY2.0.c
 *
 * Created: 3/16/2018 7:53:55 AM
 * Author : Owen
 */ 

/*
 * REFLECTIVITIES.c
 * Desc: To determine reflectivity minimimums for all four types of materials
 * These materials are Black (plastic), White (plastic), Aluminum (metal), Steel (metal)
 * In final project demonstration the lab tech will cover one of the plastic pieces with duct
 * tape most likely to mess up the reflectivity.
 *		Solutions: 1) test a metal object with duct tape around it to determine the ADC result
		2) use inductive sensor, which will pick up steel (ferrous material) quite well. Al is picked up slower (non-ferrous)
 * Operation Options:
	1) Create secondary timer that triggers ADC conversion if the flag has been reset in main
	2) Trigger ADC conversion manually inside main every time the flag is reset.
	3) Trigger ADC conversion in interrupt if object is still in OR sensor
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
void timer2Init(void);
void timer3Init(void);
void mTimer(int count);
void mTimer2(int count);
void mTimer3(int count);
/*User Defines*/
#define highNibMask 0xF0
#define lowNibMask 0x0F
#define DC_REVERSE 0x01 		//dc motor counter-clockwise
#define DC_FORWARD 0x02			//dc motor clock-wise
#define DC_BRAKE 0x00 //dc motor brake
#define CONVEYOR_SPEED 30 //50 is maximum for sustainability
/*Global Variables*/
volatile unsigned int ADCResult; //16 bits: 0 => (2^9-1); stores result of ADC conversion
volatile unsigned char ADCResultFlag; //8 bits: 0 => (2^9-1); thats that ADC conversion is complete
volatile unsigned int lowADCArray[8];
volatile unsigned int ADCFilterCount;
volatile unsigned char ADC_RESET;
volatile unsigned int lowADC;
//volatile unsigned int highByteADC;
//volatile unsigned int lowByteADC;
volatile unsigned int f;

/*Beginning of main program*/
int main(void){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	int i=0;
	uint16_t ADCAverage = 0; //needs to be able to hold a maximum of 0x2000
	cli(); //disable all interrupts
	initTimer1();
	timer2Init();
	timer3Init();
	setupPWM(CONVEYOR_SPEED); //DC Motor PWM
	setupISR(); //interrupt initializations
	setupADC(); //ADC initializations
	DDRB = 0xFF; /*controls dc motor: PB7=PWM signal PB3:0={INA,INB,ENA,ENB}*/
	DDRC = 0xFF; //LEDs Debugging
	DDRD = 0xF0; //upper nibble for on-board bi-color LEDs, interrupts on lower nibble	PORTD3:0=INT3:0
	DDRF = 0x00; /*PF1=ADC1 pin*/
	sei(); //enable all interrupts
	PORTB &= 0b11110000; //apply brake to Vcc
	PORTC=0b10101010;
	mTimer2(2000);
	PORTC=0b00000000;
	mTimer2(2000);
	motorControl(CONVEYOR_SPEED,DC_FORWARD); //start conveyor towards stepper
	ADC_RESET=0;
	while (1){
		if (ADCResultFlag){ //object has passed OR sensor and reflectivity has been measured
			ADCAverage=0;
			for(i=0;i<8;i++){
				ADCAverage+=lowADCArray[ADCFilterCount];
				ADCFilterCount++;
				ADCFilterCount&=0x07; //modulus of 8 with positive incrementing variables
			}
			ADCAverage>>=3; //division by 8 with chopping, not rounding
			PORTC=ADCAverage;
			PORTD=((ADCAverage&0x0300)>>3); //0b01100000 if true //green green
		}
	}
	return (0); //This line returns a 0 value to the calling program
}
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
	//EIMSK |= _BV(INT6) |_BV(INT2); //enable INT6
	EIMSK |= _BV(INT2); //enable INT6
	EICRA |= _BV(ISC21) | _BV(ISC20); //rising edge interrupt; EICRA determines INT3:0
	//EICRB |= _BV(ISC61); //falling edge
}
void setupADC(void){
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //adc scalar = 32;
	ADMUX |= _BV(REFS0) | _BV(MUX0); //AVcc reference (3.3V);read from ADC 1;output right-adjusted
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
void timer2Init(void){ //clock is turned on during interval of use and then off when unused
	//sei(); enables all interrupts thus following is unneccessary
	//TIMSK2 |= _BV(TOIE2); //enable Timer/Counter 2 Overflow interrupt; sets TOV2 bit in TIFR2 register upon overflow
	TCCR2A=0; //Mode 0:normal port operation; keeps counting no matter what; means you have to reset the TOV2 flag
	//TOP=0xFF; Update is immediate
	//TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32); starts timer
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
void mTimer2(int count){
	int i=0;
	TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32)
	TCNT2=0x00; //set timer equal to zero
	if ((TIFR2 & 0x01) == 0x01)TIFR2|=0x01; //if TOV2 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
	while (i<count){ //iterate through given count
		if ((TIFR2 & 0x01) == 0x01){ //if overflow has occurred in counter
			TIFR2|=0x01; //reset overflow flag by writing a 1 to TOV2 bit
			i+=1;
			//equivalent; TIFR2 |= _BV(TOV2)
		}
	}
	TCCR2B&=0b11111000; //disable timer 2
}
void timer3Init(void){ //clock is turned on during interval of use and then off when unused
	TCCR3A=0; //Mode 0:normal port operation; keeps counting no matter what; means you have to reset the TOV3 flag
}
void mTimer3(int count){ //16 bit timer 8.192ms per cycle
	int i=0;
	TCCR3B |= _BV(CS30); //clock pre-scalar (clk/1)
	TCNT3=0x00; //set timer equal to zero
	if ((TIFR3 & 0x01) == 0x01)TIFR3|=0x01; //if TOV3 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
	while (i<count){ //iterate through given count
		if ((TIFR3 & 0x01) == 0x01){ //if overflow has occurred in counter
			TIFR3|=0x01; //reset overflow flag by writing a 1 to TOV2 bit
			i+=1;
			//equivalent; TIFR2 |= _BV(TOV2)
		}
	}
	TCCR3B&=0b11111000; //disable timer 2
}
/**********INTERRUPT SERVICE ROUTINES**********/
/*sensor 3: 2nt Optical Reflective, Active HIGH starts AD conversion*/
ISR(INT2_vect){ //unused --ODA
	//when there is a rising edge on PD2, ADC is triggered which is currently ADC1 (PF1)
	lowADC=0xFFFF;
	ADCSRA|= _BV(ADSC); //trigger ADC (i.e. begin ADC conversion)
}
ISR(ADC_vect){ //ADCResult is left-adjusted (i.e. the upper most byte is taken; 2 LSB' are discarded)
	if (lowADC>ADC){ //if ADC result is still decreasing (i.e. if object's reflectivity is increasing)
		lowADC=ADC; //ADC holds the entire 10 bit value in a 16bit variable; lowADC set for future comparison
		lowADCArray[ADCFilterCount]=lowADC;
		ADCFilterCount++; //increment array location being set
		ADCFilterCount&=0b00000111; //modulus of 8
		//highByteADC=ADCH;
		//lowByteADC=ADCL;
	}
	if ((PIND&0b00000100)==0b00000100) ADCSRA|= _BV(ADSC); //if there is still an object keep initializing ADC conversions
	else{
		ADCResultFlag=1;
		//PORTC=lowByteADC;
		//PORTD=((highByteADC&0b00000011)<< 5); //0b00100000 if true //green green
	}
}
ISR(INT6_vect){
	ADC_RESET=1;
	mTimer2(15); //debounce period
	while((PINE & 0b01000000)==0b01000000)mTimer2(25); //while switch is still pressed
}




