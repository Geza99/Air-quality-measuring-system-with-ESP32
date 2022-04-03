/*! @file EEPROM_utils.h
 @brief Menu for setting reading/writing EEPROM */
/*!
 * @defgroup EEPROM API
 * @{*/
#ifndef EEPROM_UTILS_H_
#define EEPROM_UTILS_H_

#include <Arduino.h>
#include <EEPROM.h>

#define MEM_WRITTEN_FLAG  0
#define SSID_ADDRESS      1
#define PASS_ADDRESS     32
#define STATE_FLAG       64
#define STATE_ADDR       65

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief This API writes a string to memory
 * 
 * @param address Addr4ess to start writing
 * @param a       String to be written to EEPROM
 * @param l       Length of string to be written to EEPROM
 */

void EEPROM_write_data(uint8_t address, String a, uint8_t l);

/**
 * @brief This API reads a string to memory
 * 
 * @param address   Address to read from
 * @param read_data Returned string
 * @return None
 */
void EEPROM_read_data(uint8_t address, const char** read_data);

/**
 * @brief This API writes the default SSID and PASS in memory.
 *        It also signals that EEPROM was written.
 * 
 */
void EEPROM_write_defaults(void);

/**
 * @brief This API gets the value of the flag signalling if EEPROM has been populated.
 *        'x' means data is written
 * 
 * @return uint8_t Value of the flag location in EEPROM
 */
uint8_t EEPROM_get_flag(void);

/**
 * @brief This API gets the value of the flag signalling if EEPROM has been populated
 *        with STATE value.
 *        'y' means data is written
 * 
 * @return uint8_t Value of the flag location in EEPROM
 */
uint8_t EEPROM_get_state_flag(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* EEPROM_UTILS_H_ */
/** @}*/