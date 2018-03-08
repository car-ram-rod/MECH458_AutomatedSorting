#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void mTimer(int count); // delay : function declaration
void initTimer1(void); //initialize Timer 1 for CTC (Clear Timer on Compare)
void mTimer2(int count);
void timer2Init(void);
#endif // INTERRUPT_H_
