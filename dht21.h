#ifndef _DHT21_H_
#define _DHT21_H_

//
//    FILE: dht.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.09
// PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// see dht.cpp file
//

#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2
#define DHTLIB_INVALID_VALUE	-999

typedef struct {
    volatile uint8_t *port_out;
    const volatile uint8_t *port_in;
    volatile uint8_t *port_dir;
    uint8_t port_mask;

    uint16_t humidity;
    uint16_t temperature;
    int16_t error;

    uint8_t buffer[5];
} dht21_sensor_t;

void dht21_init(dht21_sensor_t *dht,
                volatile uint8_t *p_out,
                const volatile uint8_t *p_in,
                volatile uint8_t *p_dir,
                int pin);

int dht_read(dht21_sensor_t *dht);
int dht21_read(dht21_sensor_t *dht);//uint8_t pin)

unsigned int humidity;
unsigned int temperature;

uint8_t bits[5];  // buffer to receive data

#endif
