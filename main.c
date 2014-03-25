#define F_CPU 16000000

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#include "ds18b20.h"
#include "uart.h"

/** main program body */
int main(void)
{
    // initializations
    init_uart();

    // interrupt enable
    sei();

    ds18b20_sensor_t s; // create sensors context
    ds18b20_init(&s,&PORTD,&PIND,&DDRD,2); // init context

    set_sleep_mode(SLEEP_MODE_IDLE);

    while(1)
    {
        ds18d20_start_conversion(&s); // start temperature conversion
        _delay_ms(750); // wait for conversion result (min 750ms)
        ds18b20_read_conversion(&s); // get data from sensor
        printdec(s.data.temp);
        /*sleep_enable();
        sleep_cpu();
        sleep_disable();*/
    }

    return -1;
}
