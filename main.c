#define F_CPU 16000000

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#include "ds18b20.h"
#include "uart.h"
#include "timer.h"
#include "dht21.h"

/** main program body */
int main(void)
{
    // initializations
    init_uart();

    init(); // timer

    // interrupt enable
    sei();

    ds18b20_sensor_t ds; // create sensors context
    ds18b20_init(&ds,&PORTD,&PIND,&DDRD,2); // init context

    //set_sleep_mode(SLEEP_MODE_IDLE);
    dht21_sensor_t dht;
    dht21_init(&dht,&PORTD,&PIND,&DDRD,5);

    while(1)
    {
        /// ds18b20 test
        int ds_start_tim = micros();
        ds18d20_start_conversion(&ds); // start temperature conversion
        ds_start_tim = micros()-ds_start_tim;
        _delay_ms(750); // wait for conversion result (min 750ms)
        int ds_read_tim = micros();
        ds18b20_read_conversion(&ds); // get data from sensor
        ds_read_tim = micros()-ds_read_tim;

        // DISPLAY DATA
        char answbuff[64];
        int len;
        strcp(&answbuff[0],"DS18B20: ");
        sprint_int(&answbuff[strlen(answbuff)],ds_start_tim);
        strcp(&answbuff[strlen(answbuff)],"us ");
        sprint_int(&answbuff[strlen(answbuff)],ds_read_tim);
        strcp(&answbuff[strlen(answbuff)],"us ");
        sprint_int(&answbuff[strlen(answbuff)],ds.data.temp);
        if (ds.valid)
        {
            int len = strlen(answbuff); answbuff[len]=answbuff[len-1]; answbuff[len-1]='.';
            strcp(&answbuff[len+1],"C OK\r\n");
        }
        else
        {
            strcp(&answbuff[strlen(answbuff)]," ERROR\r\n");
        }
        uart_puts(answbuff);


        /// dht21 test
        int durrat_micros = micros(); // measure how long it took
        dht21_read(&dht);
        durrat_micros = micros()-durrat_micros;


        // DISPLAY DATA
        //char answbuff[64]
        strcp(&answbuff[0],"DHT21: ");
        sprint_int(&answbuff[strlen(answbuff)],durrat_micros);
        strcp(&answbuff[strlen(answbuff)],"us ");
        switch(dht.error)
        {
            case DHTLIB_OK:
                sprint_int(&answbuff[strlen(answbuff)],dht.humidity);
                len = strlen(answbuff); answbuff[len]=answbuff[len-1]; answbuff[len-1]='.';
                strcp(&answbuff[len+1],"% ");
                sprint_int(&answbuff[strlen(answbuff)],dht.temperature);
                len = strlen(answbuff); answbuff[len]=answbuff[len-1]; answbuff[len-1]='.';
                strcp(&answbuff[len+1],"C ");
                strcp(&answbuff[strlen(answbuff)],"OK\r\n");
                break;
            case DHTLIB_ERROR_CHECKSUM: strcp(&answbuff[strlen(answbuff)],"Checksum error\r\n"); break;
            case DHTLIB_ERROR_TIMEOUT: strcp(&answbuff[strlen(answbuff)],"Time out error\r\n"); break;
            default: strcp(&answbuff[strlen(answbuff)],"Unknown error\r\n"); break;
        }
        uart_puts(answbuff);


        _delay_ms(2000);

        /*sleep_enable();
        sleep_cpu();
        sleep_disable();*/
    }

    return -1;
}
