#include "interrupt.h"
void timer1Init (void){ //initialize Timer 1 for CTC (Clear Timer on Compare)
	TCCR1A = 0;
	TIMSK1 |= _BV(TOIE1); //enable timer 1 overflow interrupt
	// set the initial value of the Timer rCounter to 0x0000
	//TCNT1 = 0x0000;
}

/*at a clock frequency of 8MHz this is a 1.024ms timer for each while loop cycle*/
/*e.g. (32/8MHz)x(0xFF=256)=0.001024s=1.024ms*/
void timer0Init(void){ //clock is turned on during interval of use and then off when unused
	//sei(); enables all interrupts thus following is unneccessary 
	//TIMSK2 |= _BV(TOIE2); //enable Timer/Counter 2 Overflow interrupt; sets TOV2 bit in TIFR2 register upon overflow
	TCCR0A=0; //Mode 0:normal port operation; keeps counting no matter what; means you have to reset the TOV2 flag
		//TOP=0xFF; Update is immediate
	//TCCR2B |= _BV(CS20) | _BV(CS21); //clock pre-scalar (clk/32); starts timer
}
void mTimer0(int count){
	int i=0;
	int l=0;
	//TCCR0B |= _BV(CS00) | _BV(CS01); //clock pre-scalar (clk/64)
	TCCR0B |=_BV(CS01); //clock pre-scalar (clk/8)
	TCNT0=0x00; //set timer equal to zero
	if ((TIFR0 & 0x01) == 0x01)TIFR0|=0x01; //if TOV0 flag is set to 1, reset to 0 by setting bit to 1 (confused?)
	for (l=0;l<4;l++){
		i=0;
		while (i<count){ //iterate through given count
			if ((TIFR0 & 0x01) == 0x01){ //if overflow has occurred in counter
				TIFR0|=0x01; //reset overflow flag by writing a 1 to TOV0 bit
				i+=1;
				//equivalent; TIFR0 |= _BV(TOV0)
			}
		}
	}
	TCCR0B&=0b11111000; //disable timer 0
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
	TCCR3B&=0b11111000; //disable timer 3
}

/*void timer1Init (void){ //initialize Timer 1 for CTC (Clear Timer on Compare)
	//set Waveform Generation mode to Clear Timer
	//set WGM bits to 0100
	//note WGM is spread over two registers
	TCCR1B |= _BV(WGM12);
	//set Output Compare Register for 1000 cycles (1ms)
	OCR1A = 0x03E8;
	// set the initial value of the Timer rCounter to 0x0000
	TCNT1 = 0x0000;
	return;
}
void mTimer(int count){ // delay microsecond
	int i = 0; //initialize loop counter
	//Enable the output compare interrupt enable//
	//TIMSK1 = TIMSK1 | 0b00000010; // --ODA edit: becomes
	//initialize timer 1 with prescalar of 1/64//
	TCCR1B |= _BV(CS11) | _BV(CS10);
	// Clear the timer interrupt flag and begin timer //
	TIFR1 |= _BV(OCF1A);

	while (i<count){
		if ((TIFR1 & 0x02) == 0x02){
			//clear interrupt flag by WRITING a ONE to the bit
			TIFR1 |= _BV(OCF1A);
			i++; //increment loop counter
			//Timer resets automatically due to WGM settings
		} //if end
	}//while end
	TCCR1B &= 0b11111000; //shut off timer 1
	return;
} //mTimer
*/