#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void mTimer(int count); // delay : function declaration
void timer1Init(void); //initialize Timer 1 for CTC (Clear Timer on Compare)
void mTimer2(int count);
void timer2Init(void); //initialize Timer 2 for normal operation
void mTimer3(int count);
void timer3Init(void); //initialize Timer 3 for normal operation
#endif // INTERRUPT_H_
