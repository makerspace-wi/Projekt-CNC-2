/*
 * FanControl ATtiny45.c
 *
 * Controls a DC-fan based on 2 temperature sensors
 *
 * Timer 0: PWM output on OC0A, controls the DC fan
 * Timer 1: regular timer interrupt (2 Hz) triggers temperature measurement once per second
 *
 * 25-Oct-2015  Stephan Laage-Witt
 */
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// configuration parameters  -------------------------------------------------------------------------
#define TEMP_LOW 300         // lower limit of fan speed control range (1/10 degree Celsius)
#define TEMP_HIGH 600        // upper limit of fan speed control range (1/10 degree Celsius)
#define FAN_OFF 255          // PWM setting for "fan off"
#define FAN_LOW 225          // PWM setting for "fan starting at minimal speed"
#define FAN_HIGH 0           // PWM setting for "fan at maximum speed"
#define FAN_STARTUP_DELAY 30 // duration of the energy-pulse to start fan from standstill (msec)
#define POWER_ON_DELAY 2     // runtime of the fan at power-on (sec)

// DS18B20 instructions codes ------------------------------------------------------------------------
#define DS18B20_CMD_SKIPROM       0xcc
#define DS18B20_CMD_CONVERTTEMP   0x44
#define DS18B20_CMD_RSCRATCHPAD   0xbe
#define DS18B20_CMD_WSCRATCHPAD   0x4e

// Ports and pins ------------------------------------------------------------------------------------
#define ERROR_LED_DDR DDRB    // Data direction for error LED
#define ERROR_LED_PORT PORTB  // Output port for error LED
#define ERROR_LED_INPORT PINB // Input port for error LED (used to toggle LED by writing)
#define ERROR_LED_PIN 2       // Pin number for error LED

#define DS18B20_DDR DDRB      // Data direction for temperature sensor ports
#define DS18B20_PORT PORTB    // Output port for temperature sensor
#define DS18B20_INPORT PINB   // Input port for temperature sensor
#define DS18B20_PIN1 3        // Pin number for 1st sensor
#define DS18B20_PIN2 4        // Pin number for 2nd sensor

// global variables ----------------------------------------------------------------------------------
volatile uint8_t interrupt_cnt = 0; // interrupt counter, every second interrupt is used
uint8_t error_flag = 0;

/* interrupt service handler for timer 1 -------------------------------------------------------------
* Wakes up the CPU, 2 times per second
*/
ISR(TIMER1_COMPA_vect) {
  if (++interrupt_cnt == 2)        // wakes up CPU and increments interrupt counter
    interrupt_cnt = 0;
}

/* ds18b20_reset ---------------------------------------------------------------------------------
* Initializes the temperature sensor DS18B20
* Input: pin number on port
* Sets error_flag (global variable) in case of problems
*/
void ds18b20_reset(uint8_t pin) {
  DS18B20_PORT &= ~(1<<pin); // set pin to "low"
  cli();                     // stop interrupts
  DS18B20_DDR |= (1<<pin);   // switch port direction to output, port pin is low
  _delay_us(480.0);          // wait ...
  DS18B20_DDR &= ~(1<<pin);  // switch data direction to high and ...
  DS18B20_PORT |= (1<<pin);  // ... set pull-up resistors
  _delay_us(60);             // wait again ...
  if (DS18B20_INPORT & (1<<pin))
    error_flag = 1;          // read port: 1 -> error, no response from sensor, 0 -> okay
  sei();                     // resume interrupts
  _delay_us(420);            // wait
}

/* ds18b20_write_bit -----------------------------------------------------------------------------
* writes a data bit to temperature sensor DS18B20
* Input: pin number on port, data bit
*/
void ds18b20_write_bit(uint8_t pin, uint8_t bit) {
  DS18B20_PORT &= ~(1 << pin);  // set pin to "low"
  cli();                        // stop interrupts
  DS18B20_DDR |= (1 << pin);    // switch port direction to output, port pin is low
  _delay_us(1.0);               // wait ...
  if (bit) {                    // in case of logic 1:
    DS18B20_DDR &= ~(1 << pin); // ... change port direction to input
    DS18B20_PORT |= (1<<pin);   // ... activate pull-up resistor
  };
  _delay_us(60.0);              // wait ...
  DS18B20_DDR &= ~(1 << pin);   // in all cases: change port direction to input
  DS18B20_PORT |= (1<<pin);     // activate pull-up resistor
  sei();                        // resume interrupts
}

/* ds18b20_read_bit ------------------------------------------------------------------------------
* Reads a single bit from temperature sensor DS18B20
* Input: pin number on port
* Output: data bit
*/
uint8_t ds18b20_read_bit(uint8_t pin) {
  uint8_t bit = 0;

  DS18B20_PORT &= ~(1 << pin); // set pin to "low"
  cli();                       // stop interrupts
  DS18B20_DDR |= (1 << pin);   // switch port direction to output, port pin is low
  _delay_us(1.0);              // wait ...
  DS18B20_DDR &= ~(1 << pin);  // switch port direction to input
  DS18B20_PORT |= (1<<pin);    // activate pull-up resistor
  _delay_us(14.0);             // wait ...
  if (DS18B20_INPORT & (1 << pin)) bit = 1; // read data from sensor and set return value
  sei();                       // resume interrupts
  _delay_us(45);               // wait
  return(bit);                 // return data bit
}

/* ds18b20_read_byte -----------------------------------------------------------------------------
* Reads a complete byte from temperature sensor DS18B20
* Input: pin number on port
* Output: data byte
*/
uint8_t ds18b20_read_byte(uint8_t pin) {
  uint8_t i = 8, rd_byte = 0;

  while (i--) {
    rd_byte >>= 1;
    rd_byte |= (ds18b20_read_bit(pin) << 7); // read data bits 8 times and shift left into data byte
  };
  return(rd_byte);                           // return data byte
};

/* ds18b20_write_byte ----------------------------------------------------------------------------
* Writes a complete byte to temperature sensor DS18B20
* Input: pin number on port, data byte
*/
void ds18b20_write_byte(uint8_t pin, uint8_t wr_byte) {
  uint8_t i = 8;

  while (i--) {
    ds18b20_write_bit(pin, wr_byte & 1); // write data bit 8 times
    wr_byte >>= 1;
  };
};

/* ds18b20_init ----------------------------------------------------------------------------------
* Initializes the temperature sensor with 9 bit resolution
* Input: pin number on port
*/
void ds18b20_init(uint8_t pin) {
  ds18b20_reset(pin);                               // initiate communication via reset
  ds18b20_write_byte(pin, DS18B20_CMD_SKIPROM);     // ignore internal ROM
  ds18b20_write_byte(pin, DS18B20_CMD_WSCRATCHPAD); // write to scratch-pad of the sensor
  ds18b20_write_byte(pin, 0x00);
  ds18b20_write_byte(pin, 0x00);
  ds18b20_write_byte(pin, 0b00011111);              // set R1 and R0 to 9 bit resolution
};

/* read_temperatur -------------------------------------------------------------------------------
* Reads temperature from sensor DS18B20
* Input: pin number on port
* Output: temperature (1/10 degreed Celsius, signed integer)
*/
int16_t read_temperatur(uint8_t pin) {
  uint8_t temp1, temp2;
  int16_t temperature;

  ds18b20_reset(pin);                               // initiate communication line via reset
  ds18b20_write_byte(pin, DS18B20_CMD_SKIPROM);     // ignore internal ROM
  ds18b20_write_byte(pin, DS18B20_CMD_CONVERTTEMP); // kick off conversion
  while (!ds18b20_read_bit(pin));                   // wait until DS18B20 is ready
  ds18b20_reset(pin);                               // initiate communication line via reset
  ds18b20_write_byte(pin, DS18B20_CMD_SKIPROM);     // ignore internal ROM
  ds18b20_write_byte(pin, DS18B20_CMD_RSCRATCHPAD); // read scratch pad
  temp1 = ds18b20_read_byte(pin);                   // reading low byte
  temp2 = ds18b20_read_byte(pin);                   // reading high byte
  ds18b20_reset(pin);                               // close communication line

  temperature = temp1 >> 4;
  temperature |= temp2 << 4;
  temperature *= 10;                       // build temperature from low and high byte
  if (temp1 & 0x0f) {
    if (temperature > 0) temperature += 5; // resolution in 9 bit mode is 0.5 degree
    else temperature -= 5;
  };

  return(temperature);                     // return temperature
};

/* main -------------------------------------------------------------------------------------------*/
int main(void) {
  int16_t temp1, temp2, temp_max;
  uint8_t power_on = POWER_ON_DELAY;

  ERROR_LED_DDR |= (1 << ERROR_LED_PIN);
  ERROR_LED_PORT |= (1 << ERROR_LED_PIN); // switch off error LED

  // prepare sleep mode as idle mode
  set_sleep_mode(SLEEP_MODE_IDLE);        // interrupts wake-up CPU, counters keep running

  // initialize PWM via timer 0
  DDRB |= (1<<0);                                   // data direction for OC0A is output
  TCCR0A = (1 << COM0A0) | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // set OC0A on compare match (datasheet page 101)
  TCCR0B = (0 << CS02) | (0 << CS01) | (1 << CS00); // pre-scaler is clock / 0 (datasheet page 105)
  OCR0A = FAN_HIGH;                                 // switch fan on for startup sequence

  // initialize timer 1 for regualr interrupts, approx. 2 times per second (245 * 16384 * 1/8000000 = 0.501 seconds)
  OCR1A = 245;             // compare register
  TCCR1 |= (1<<CTC1) | (1<<CS13)| (1<<CS12) | (1<<CS11) | (1<<CS10); // pre-scaler 16384, CTC1 = clear timer on compare mode (datasheet page 90)
  TIMSK |= (1 << OCIE1A);  // enable timer interrupt
  sei();                   // enable system interrupts

  // initialize temperature sensors in 9 bit resolution mode
  ds18b20_init(DS18B20_PIN1);
  ds18b20_init(DS18B20_PIN2);

  while(1)
  {
    if (interrupt_cnt == 0) {                       // uses every other interrupt only

      if (error_flag) {
        ERROR_LED_INPORT |= (1 << ERROR_LED_PIN);   // check for temperature sensor communication errors
      };

      // read temperatures
      temp1 = read_temperatur(DS18B20_PIN1);
      temp2 = read_temperatur(DS18B20_PIN2);
      if (power_on > 0) {                           // if power-on sequence: keep the fan running
        temp_max = TEMP_HIGH;
        --power_on;
      } else {
        temp_max = (temp1 > temp2 ? temp1 : temp2); // otherwise: identify temperature maximum
      };

      // calculate fan speed and set PWM accordingly
      if (temp_max < TEMP_LOW) {                    // temperature below TEMP_LOW: fan is off
        OCR0A = FAN_OFF;
      } else if (temp_max >= TEMP_HIGH) {           // temperature above TEMP_HIGH: fan on full power
        OCR0A = FAN_HIGH;
      } else {                                      // temperatures within speed control range
        if (OCR0A == FAN_OFF) {                     // in case the fan is off:
          OCR0A = FAN_HIGH;                         // ... kick off with energy pulse
          _delay_ms(FAN_STARTUP_DELAY);
        };
        OCR0A = FAN_LOW - (uint32_t) (temp_max - TEMP_LOW) * (FAN_LOW - FAN_HIGH) / (TEMP_HIGH - TEMP_LOW);
      };

      // check for over-temperature and set LED accordingly
      if (!error_flag) {
        if (temp_max >= TEMP_HIGH)                  // if temperature is above speed control range:
          ERROR_LED_PORT &= ~(1<<ERROR_LED_PIN);    // ... set error LED is continuously on
        else
          ERROR_LED_PORT |= (1<<ERROR_LED_PIN);     // otherwise switch error LED off
      };

    };
    sleep_mode();                                   // enter sleep mode to save energy
  }
}
