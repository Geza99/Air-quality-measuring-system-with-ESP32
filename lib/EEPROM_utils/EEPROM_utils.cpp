/**\mainpage
 *
 * File		EEPROM_utils.c
 * @date	15 Oct 2018
 * @version	1.0.0
 *
 */

/*! @file EEPROM_utils.c
 @brief Menu for reading/writing EEPROM */
#include "EEPROM_utils.h"

/**
 * @brief This API writes a string to memory
 */

void EEPROM_write_data(uint8_t address, String a, uint8_t l)
{
    /* index */
    uint8_t i = 0;
    /* address */
    uint8_t addr = address;

    while (i<l)
    {
        EEPROM.put(addr, a[i]);
        addr++;
        i++;
    }

    EEPROM.put(addr, '~');
}

/**
 * @brief This API reads a string to memory
 */
void EEPROM_read_data(uint8_t address, const char** read_data)
{
    /* index */
    uint8_t i = 0;
    /* address */
    uint8_t addr = address;
    /* read byte */
    char data;
    /* String */
    char *read_String = strdup("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

    /* Read first byte */
    data = EEPROM.read(addr);

    /* Place first byte in returned string*/
    read_String[0]=data;

    /* Read until tilde character encountered */
    while ((data != '~')&&(i<32))
    {
        addr++;
        i++;
        data = EEPROM.read(addr);
        read_String[i]=data;
    }

    read_String[i]='\0';

    *read_data = read_String;
}

/**
 * @brief This API writes the default SSID and PASS in memory.
 *        It also signals that EEPROM was written.
 */
void EEPROM_write_defaults(void)
{
    String SSID = "BTIA_2ndFloor";
    String PASS = "RobertBosch0100";

    /* Signal that EEPROM has been initialized */
    EEPROM.put(MEM_WRITTEN_FLAG, 'x');

    /* Write default SSID */
    EEPROM_write_data(SSID_ADDRESS, SSID, SSID.length());

    /* Write default PASS */
    EEPROM_write_data(PASS_ADDRESS, PASS , SSID.length());

    /* Save the changes to the EEPROM */
    EEPROM.commit();
}

/**
 * @brief This API gets the value of the flag signalling if EEPROM has been populated.
 *        'x' means data is written.
 */
uint8_t EEPROM_get_flag(void)
{
    uint8_t data = 0;

    data = EEPROM.read(MEM_WRITTEN_FLAG);

    return data;
}

/**
 * @brief This API gets the value of the flag signalling if EEPROM has been populated
 *        with STATE value.
 *        'y' means data is written
 */
uint8_t EEPROM_get_state_flag(void)
{
    uint8_t data = 0;

    data = EEPROM.read(STATE_FLAG);

    return data;
}