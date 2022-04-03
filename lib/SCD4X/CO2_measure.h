#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C"
{
#endif

void CO2_value();
void printUint16Hex(uint16_t value);
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2);
uint16_t getco2();

#ifdef __cplusplus
}
#endif