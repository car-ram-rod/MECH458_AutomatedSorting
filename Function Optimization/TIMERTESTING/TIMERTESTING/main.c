/*
 * TIMERTESTING.c
 *
 * Created: 3/7/2018 5:26:34 PM
 * Author : Owen
 */ 

/*
 * TIMERTESTING.c
 *
 * Created: 3/7/2018 5:26:34 PM
 * Author : Owen
 */ 

/*
########################################################################
# MILESTONE : 2
# PROGRAM : 2: Basic C & MCU Output
# PROJECT : Lab2
# GROUP : 7
# NAME 1 : Owen, Anderberg, V00862140
# NAME 2 : Ben, Duong, V00839087
# DESC : This program does a series of LED acrobatics with timer interrupts (e.g. Knight Rider)
# DATA
# REVISED 2018-JAN-27
########################################################################
*/
#include <stdlib.h> // the header of the general purpose standard library of C programming language
#include <avr/io.h>// the header of i/o port
#include <avr/io.h>
#include <avr/interrupt.h>

/*Function Declarations*/
void mTimer(int count);
void mTimer2(int count);
void timer2Init(void);

/*User Defines*/
#define highNibMask 0xF0
#define lowNibMask 0x0F

/* Main Routine */
int main(int argc, char *argv[]){
	CLKPR = _BV(CLKPCE);/*initialize clock to 8MHz*/
	CLKPR = 0;
	/*User Variables*/
	uint8_t LEDOutput = 0;
	uint8_t knightRider = 0;
	uint16_t LEDShift = 0;
	int k;
	int i;

	/*Timer setup*/
	TCCR1B |= _BV(CS10); //set clock prescaler: bit 0 of TCCR1B register (TCCR1B |= 0x01)
	timer2Init();
	DDRC = 0b11111111; /* Sets all pins on Port C to output */
	DDRD |= 0xF0; /* Sets upper nibble on Port D to output */

	while(1){ //run program continuously
		/*LAB1: On board LED testing (D5, D2)*/
		for(k=0;k<2;k++){
			PORTD = 0b10100000; //D5=Red; D2=Green
			mTimer2(1000);
			PORTD = 0b01010000; //D5=Green; D2=Red
			mTimer2(1000);
			PORTD = 0b11110000; //D5=Yellow; D2=Yellow
			mTimer2(1000);
			PORTD = 0b10010000; //D5=Red; D2=Red
			mTimer2(1000);
			PORTD = 0b01100000; //D5=Green; D2=Green
			mTimer2(1000);
		}

		knightRider=1; //enable knightRider functionality
		/*LAB1: Knight Rider function. Uses software delay while LAB2 will use hardware interrupts*/
		if(knightRider>0){
			//start from center and turn ON LEDs until all LEDs ON; reset.
			for(k=0;k<2;k++){
				PORTC = 0b00011000; //set only middle LEDs to ON
				mTimer2(200);
				PORTC |= 0b00100100;
				mTimer2(200);
				PORTC |= 0b01000010;
				mTimer2(200);
				PORTC |= 0b10000001;
				mTimer2(200);
				PORTC = 0b00000000;
				mTimer2(200);
			}
			//start from center and turn ON LEDs going to exterior LEDs using bit shifting
			LEDOutput = 0b00011000;
			for(i=0;i<2;i++){
				for (k=0;k<4;k++){
					PORTC = (((LEDOutput & highNibMask) << k) + ((LEDOutput & lowNibMask) >> k));
					mTimer2(200);
				}
				for (k=2;k>0;k--){
					PORTC = (((LEDOutput & highNibMask) << k) + ((LEDOutput & lowNibMask) >> k));
					mTimer2(200);
				}
			}
			PORTC = 0b00011000;
			mTimer2(200);
			PORTC = 0b00000000;
			mTimer2(200);
			// push one LED back and forth across display
			LEDOutput=0b00000001;
			for(i=0;i<2;i++){
				for(k=0;k<8;k++){
					PORTC = (LEDOutput << k);
					mTimer2(200);
				}
				for(k=6;k>0;k--){
					PORTC = (LEDOutput << k);
					mTimer2(200);
				}
			}
			//bit shift grouping of 3 LEDs back and forth across display; 2 LEDs pushed off display at extremes.
			for(i=0;i<2;i++){
				for(k=0;k<10;k++){
					LEDShift = (0b00000111 << k) >> 2; //3 bits are shifted using a 16 bit variable; otherwise overflow occurs
					LEDOutput = LEDShift; //lower byte of 16 bit variable set equal to 8 bit variable for clarity.
					PORTC = LEDOutput;
					mTimer2(200);
				}
				for(k=8;k>0;k--){
					LEDShift = (0b00000111 << k) >> 2;
					LEDOutput = LEDShift;
					PORTC = LEDOutput;
					mTimer2(200);
				}
			}
			PORTC = 0b00000001;
			mTimer2(200);
			//turn-off all LEDs
			PORTC = 0b00000000;
			PORTD = 0b00000000;
		}
	}
	return (0); //This line returns a 0 value to the calling program
	// generally means no error was returned
}
/*
Timer interrupt using timer1 on micro controller AT90USB1287
Accuracy tested over multiple tests with 10 second timer: confirmed within 50ms including user error
*/
void mTimer(int count){ // delay microsecond
	int i = 0; //initialize loop counter
	/*set Waveform Generation mode to Clear Timer*/
	/*set WGM bits to 1000 in TCCR1B register (Timer/Counter1 Control Register B)*/
	/*note WGM is spread over two registers but we only need to set one for this purpose*/
	TCCR1B |= _BV(WGM12);
	/*set Output Compare Register for 1000 cycles (1ms)*/
	OCR1A = 0x03E8;
	/*set the initial value of the Timer rCounter to 0x0000*/
	TCNT1 = 0x0000;
	/*Enable the output compare interrupt enable*/
	TIMSK1 = TIMSK1 | 0b00000010;
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
	return;
} //mTimer
void timer2Init(void){
	TIMSK2 |= _BV(TOIE2); //enable Timer/Counter 2 Overflow interrupt; sets TOV2 bit in TIFR2 register upon overflow
	TCCR2A=0; //Mode 0:normal port operation; keeps counting no matter what; means you have to reset the TOV2 flag
		//TOP=0xFF; Update is immediate
	TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32)
}
void mTimer2(int count){
	int i=0;
	TCNT2=0x00;
	while (i<count){
		if ((TIFR2 & 0x01) == 0x01){
			TIFR2|=0x01;
			i+=1;
			//equivalent; TIFR2 |= _BV(TOV2)
		}
	}
}

