#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <inttypes.h>

// union converting data into temperature
typedef union{
    uint8_t d[9];
    int16_t temp;
} ds18b20_data_t;

// sensor context struct
typedef struct {
    volatile uint8_t *port_out;
    const volatile uint8_t *port_in;
    volatile uint8_t *port_dir;
    uint8_t port_mask;

    ds18b20_data_t data;
    bool valid;
} ds18b20_sensor_t;

// sensors context and port initialization
void ds18b20_init(ds18b20_sensor_t *s,
                  volatile uint8_t *p_out,
                  const volatile uint8_t *p_in,
                  volatile uint8_t *p_dir,
                  int pin);

// send start conversion command
void ds18d20_start_conversion(ds18b20_sensor_t *s);
// read converted data and test crc
void ds18b20_read_conversion(ds18b20_sensor_t *s);

#endif
