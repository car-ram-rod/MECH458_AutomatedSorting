#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

void mTimer(int count); // delay : function declaration
void initTimer1(void);
#endif // INTERRUPT_H_
