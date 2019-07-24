#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "SPI.h"

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB

#define MAX31856_SPI_SS 4
 
#define MAX31856_CONFIG_3WIRE          0x10
#define MAX31856_CONFIG_REG            0x00
#define MAX31856_CONFIG_BIAS           0x80
#define MAX31856_CONFIG_MODEAUTO       0x40
#define MAX31856_CONFIG_MODEOFF        0x00
#define MAX31856_CONFIG_FAULTSTAT      0x02
#define MAX31856_CONFIG_1SHOT          0x20
#define MAX31856_RTDMSB_REG           0x01
#define MAX31856_FAULTSTAT_REG        0x07
#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set
  SPI_PORTX &= ~(1<<MAX31856_SPI_SS);

  SPI_ReadByte(addr);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = SPI_ReadByte(0xFF);
    //Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  SPI_PORTX |= (1<<MAX31856_SPI_SS); 
  //Serial.println();
}

uint8_t readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  
  return ret;
}

void writeRegister8(uint8_t addr, uint8_t data) {
  SPI_PORTX &= ~(1<<MAX31856_SPI_SS);
  SPI_ReadByte(addr | 0x80);   // make sure top bit is set
  SPI_ReadByte(data);
  SPI_PORTX |= (1<<MAX31856_SPI_SS); 
}

void enableBias(uint8_t b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS;       // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS;       // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void autoConvert(uint8_t b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void max31865_setup(void) {
  SPI_DDRX |= (1<<MAX31856_SPI_SS);
  SPI_PORTX |= (1<<MAX31856_SPI_SS);
  
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_3WIRE;
  //Serial.print(t, HEX);
  writeRegister8(MAX31856_CONFIG_REG, t);
  enableBias(0);
  autoConvert(0);
  clearFault();  
}

uint16_t readRTD (void) {
  clearFault();
  enableBias(1);
  _delay_us(10);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;      
  writeRegister8(MAX31856_CONFIG_REG, t);
  _delay_us(65);

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

float max31865_temperature(void) {
  // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= RREF;
  
  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RNOMINAL;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;
  
  if (temp >= 0) return temp;

  // ugh.
  Rt /= RNOMINAL;
  Rt *= 100;      // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt;  // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt;  // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt;  // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt;  // ^5
  temp += 1.5243e-10 * rpoly;
  
  //dtostrf(temp, 4, 2, s);
  return temp;
}

uint8_t max31865_readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}
