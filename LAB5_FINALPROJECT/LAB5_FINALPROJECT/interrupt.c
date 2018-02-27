#include "interrupt.h"
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
	//TIMSK1 = TIMSK1 | 0b00000010; // --ODA edit
	/*initialize timer with prescalar of 1*/
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
	TCCR1B &= 0b11111000; //shut off timer 
	return;
} //mTimer
