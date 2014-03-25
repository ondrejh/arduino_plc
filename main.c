#define F_CPU 16000000

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#include "uart.h"

/** main program body */
int main(void)
{
    /// board settings
    /*PORTB &= ~(1<<5);
    DDRB  |=  (1<<5);
    // button inputs
    DDRC  &= ~0x07;
    PORTC |=  0x07;*/

    // initializations
    init_uart();

    // interrupt enable
    sei();

    set_sleep_mode(SLEEP_MODE_IDLE);

    while(1)
    {
        sleep_enable();
        sleep_cpu();
        sleep_disable();
    }

    return -1;
}
