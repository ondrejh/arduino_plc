// Based on:
//
//    FILE: dht.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.09
// PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// 0.1.09 optimize size: timeout check + use of mask
// 0.1.08 added formula for timeout based upon clockspeed
// 0.1.07 added support for DHT21
// 0.1.06 minimize footprint (2012-12-27)
// 0.1.05 fixed negative temperature bug (thanks to Roseman)
// 0.1.04 improved readability of code using DHTLIB_OK in code
// 0.1.03 added error values for temp and humidity when read failed
// 0.1.02 added error codes
// 0.1.01 added support for Arduino 1.0, fixed typos (31/12/2011)
// 0.1.0 by Rob Tillaart (01/04/2011)
//
// inspired by DHT11 library
//
// Released to the public domain
//

#include "timer.h"

#include "dht21.h"

#define F_CPU 16000000
#include <util/delay.h>


// #define TIMEOUT 10000
// uint16_t for UNO, higher CPU speeds => exceed MAXINT.
// works for DUE
// 16 MHz => 10000
// 84 MHz => 52500
// 100MHz => 62500
#define TIMEOUT (F_CPU/1600)

/*#define DHT_PIN_LOW() do{PORTD&=~0x20;DDRD|=0x20;}while(0)
#define DHT_PIN_HIGH() do{PORTD|=0x20;DDRD|=0x20;}while(0)
#define DHT_PIN_SET_INPUT() do{DDRD&=~0x20;PORTD|=0x20;}while(0)
#define DHT_PIN_READ ((PIND&0x20)!=0)*/

#define DHT_PIN_LOW(x) do{*(x->port_out)&=~(x->port_mask);*(x->port_dir)|=(x->port_mask);}while(0)
#define DHT_PIN_HIGH(x) do{*(x->port_out)|=(x->port_mask);*(x->port_dir)|=(x->port_mask);}while(0)
#define DHT_PIN_SET_INPUT(x) do{*(x->port_dir)&=~(x->port_mask);*(x->port_out)|=(x->port_mask);}while(0)
#define DHT_PIN_READ(x) ((*(x->port_in)&(x->port_mask))!=0)

#define DHT_STARTUP_LOW_DELAY_US 2000
#define DHT_STARTUP_HIGH_DELAY_US 40

void dht21_init(dht21_sensor_t *dht,
                volatile uint8_t *p_out,
                const volatile uint8_t *p_in,
                volatile uint8_t *p_dir,
                int pin)
{
    // copy arguments into context
    (dht->port_out) = p_out;
    (dht->port_dir) = p_dir;
    //(s->port_ren) = p_ren;
    (dht->port_in)  = p_in;
    // get port mask
    dht->port_mask = (1<<pin);

    // setup port
    *(dht->port_dir) &= ~(dht->port_mask);
    *(dht->port_out) &= ~(dht->port_mask);
}

// return values:
// DHTLIB_OK
// DHTLIB_ERROR_TIMEOUT
int dht_read(dht21_sensor_t *dht)//uint8_t pin)
{
    /// INIT BUFFERVAR TO RECEIVE DATA
    uint8_t mask = 128;
    uint8_t idx = 0;

    /// EMPTY BUFFER
    for (uint8_t i=0; i< 5; i++) dht->buffer[i] = 0;

    /// REQUEST SAMPLE
    //pinMode(pin, OUTPUT);
    //digitalWrite(pin, LOW);
    DHT_PIN_LOW(dht);
    _delay_us(DHT_STARTUP_LOW_DELAY_US); // at least 1ms (1000 doesn't work)
    //digitalWrite(pin, HIGH);
    DHT_PIN_HIGH(dht);
    _delay_us(DHT_STARTUP_HIGH_DELAY_US);
    //pinMode(pin, INPUT);
    DHT_PIN_SET_INPUT(dht);

    /// GET ACKNOWLEDGE or TIMEOUT
    unsigned int loopCnt = TIMEOUT;
    //while(digitalRead(pin) == LOW)
    while(!DHT_PIN_READ(dht))
    if (--loopCnt == 0)
    {
        dht->error = DHTLIB_ERROR_TIMEOUT;
        return DHTLIB_ERROR_TIMEOUT;
    }

    loopCnt = TIMEOUT;
    //while(digitalRead(pin) == HIGH)
    while(DHT_PIN_READ(dht))
    if (--loopCnt == 0)
    {
        dht->error = DHTLIB_ERROR_TIMEOUT;
        return DHTLIB_ERROR_TIMEOUT;
    }

    /// READ THE OUTPUT - 40 BITS => 5 BYTES
    for (uint8_t i=0; i<40; i++)
    {
        loopCnt = TIMEOUT;
        //while(digitalRead(pin) == LOW)
        while(!DHT_PIN_READ(dht))
        if (--loopCnt == 0)
        {
            dht->error = DHTLIB_ERROR_TIMEOUT;
            return DHTLIB_ERROR_TIMEOUT;
        }

        unsigned long t = micros();

        loopCnt = TIMEOUT;
        //while(digitalRead(pin) == HIGH)
        while(DHT_PIN_READ(dht))
        if (--loopCnt == 0)
        {
            dht->error = DHTLIB_ERROR_TIMEOUT;
            return DHTLIB_ERROR_TIMEOUT;
        }

        if ((micros() - t) > 40) dht->buffer[idx] |= mask;
        mask >>= 1;
        if (mask == 0)   // next byte?
        {
            mask = 128;
            idx++;
        }
    }

    return DHTLIB_OK;
}

// return values:
// DHTLIB_OK
// DHTLIB_ERROR_CHECKSUM
// DHTLIB_ERROR_TIMEOUT
int dht21_read(dht21_sensor_t *dht)//uint8_t pin)
{
    // READ VALUES
    int rv = dht_read(dht);
    if (rv != DHTLIB_OK)
    {
        dht->humidity    = DHTLIB_INVALID_VALUE;  // invalid value, or is NaN prefered?
        dht->temperature = DHTLIB_INVALID_VALUE;  // invalid value
        return rv; // propagate error value
    }

    // CONVERT AND STORE
    dht->humidity = (unsigned int)dht->buffer[0]<<8 | dht->buffer[1];

    dht->temperature = (unsigned int)dht->buffer[2]<<8 | dht->buffer[3];

    // TEST CHECKSUM
    uint8_t sum = dht->buffer[0] + dht->buffer[1] + dht->buffer[2] + dht->buffer[3];
    if (dht->buffer[4] != sum)
    {
        dht->error = DHTLIB_ERROR_CHECKSUM;
        return DHTLIB_ERROR_CHECKSUM;
    }

    dht->error = DHTLIB_OK;
    return DHTLIB_OK;
}

//
// END OF FILE
//
