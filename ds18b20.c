/**
 * ds18b20 sensor module for arduino (based on module for msp430g2553@1MHz)
 *
 * It's not complette onewire implementation. It's one purpose module for connecting
 * one sensor per one pin instead. For onewire protocol implementation google OneWire.
 *
 * author: ondrejh.ck@email.cz
 * date: 3.2014
 *
 * example usage (PORT D, PIN 2):
 *
 *    ds18b20_sensor_t s; // create sensors context
 *    ds18b20_init(&s,&PORTD,&PIND,&DDRD,2); // init context
 *    ...
 *    ds18b20_start_conversion(&s); // start temperature conversion
 *    sleep(1); // wait for conversion result (min 750ms)
 *    ...
 *    ds18b20_read_conversion(&s); // get data from sensor
 *    ...
 *    if ((s.valid)==true) // check if read data valid
 *    {
 *      printf("Treg %d\n",s.data.temp); // show it
 *    }
 *    else
 *    {
 *      printf("!ERROR!\n"); // show error
 *    }
 *
 * edit 3.2014: module repurposed for arduino
 */

#define F_CPU 16000000

//#include <msp430g2553.h>
//#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "ds18b20.h"
#include "crc8.h"

#define wait(x) _delay_us(x)


// local function prototypes

void ds18b20_bus_reset(ds18b20_sensor_t *s);

void ds18b20_write_zero(ds18b20_sensor_t *s);
void ds18b20_write_one(ds18b20_sensor_t *s);
void ds18b20_write_byte(ds18b20_sensor_t *s, uint8_t b);

int ds18b20_read_bit(ds18b20_sensor_t *s);
uint8_t ds18b20_read_byte(ds18b20_sensor_t *s);


// local functions implementation

void ds18b20_bus_reset(ds18b20_sensor_t *s)
{
    // pull bus low
    *(s->port_dir) |=  (s->port_mask); // P1DIR |= 0x80;
    // wait 480us
    wait(480);
    // release the bus
    *(s->port_dir) &= ~(s->port_mask); // P1DIR &= ~0x80;
    // wait 480us
    wait(480);
}

void ds18b20_write_zero(ds18b20_sensor_t *s)
{
    cli();//_BIC_SR(GIE);      // Disable interrupts

    // pull bus low
    *(s->port_dir) |=  (s->port_mask); // P1DIR |= 0x80;
    // wait 60us
    wait(60);
    // release the bus
    *(s->port_dir) &= ~(s->port_mask); // P1DIR &= ~0x80;

    sei();//_BIS_SR(GIE);      // Enable interrupts
}

void ds18b20_write_one(ds18b20_sensor_t *s)
{
    cli();//_BIC_SR(GIE);      // Disable interrupts

    // pull bus low
    *(s->port_dir) |=  (s->port_mask); // P1DIR |= 0x80;
    // release the bus
    *(s->port_dir) &= ~(s->port_mask); // P1DIR &= ~0x80;
    // wait 60us
    wait(60);

    sei();//_BIS_SR(GIE);      // Enable interrupts
}

void ds18b20_write_byte(ds18b20_sensor_t *s, uint8_t b)
{
    uint8_t mask = 0x01;
    while (mask!=0)
    {
        if ((b&mask)!=0) ds18b20_write_one(s);
        else ds18b20_write_zero(s);
        mask<<=1;
    }
}

int ds18b20_read_bit(ds18b20_sensor_t *s)
{
    cli();//_BIC_SR(GIE);      // Disable interrupts

    int retval = 0;
    // pull bus low
    *(s->port_dir) |=  (s->port_mask);
    // release the bus
    *(s->port_dir) &= ~(s->port_mask);
    // test input
    if ((*(s->port_in)&(s->port_mask))!=0) retval = 1;
    // wait 60 us
    wait(60);

    // return
    sei();//_BIS_SR(GIE);      // Enable interrupts
    return retval;
}

uint8_t ds18b20_read_byte(ds18b20_sensor_t *s)
{
    uint8_t mask = 0x01;
    uint8_t retval = 0;
    while (mask!=0)
    {
        if (ds18b20_read_bit(s)) retval|=mask;
        mask<<=1;
    }
    return retval;
}


// interface functions implementation

// sensors context and port initialization
void ds18b20_init(ds18b20_sensor_t *s,
                  volatile uint8_t *p_out,
                  const volatile uint8_t *p_in,
                  volatile uint8_t *p_dir,
                  int pin)
{
    // copy arguments into context
    (s->port_out) = p_out;
    (s->port_dir) = p_dir;
    //(s->port_ren) = p_ren;
    (s->port_in)  = p_in;
    // get port mask
    s->port_mask = (1<<pin);

    // setup port
    *(s->port_dir) &= ~(s->port_mask);
    *(s->port_out) &= ~(s->port_mask);
}

// send start conversion command
void ds18d20_start_conversion(ds18b20_sensor_t *s)
{
    ds18b20_bus_reset(s);
    ds18b20_write_byte(s,0xCC);
    ds18b20_write_byte(s,0x44);
}

// read converted data and test crc
void ds18b20_read_conversion(ds18b20_sensor_t *s)
{
    ds18b20_bus_reset(s);
    ds18b20_write_byte(s,0xCC);
    ds18b20_write_byte(s,0xBE);
    int i;
    bool allzeros = true;
    for (i=0;i<10;i++) {s->data.d[i]=ds18b20_read_byte(s); if (s->data.d[i]!=0) allzeros=false;}
    if ((!allzeros)&&(crc8(s->data.d,8)==s->data.d[8]))
    {
        s->valid = true;
    }
    else s->valid = false;
}
