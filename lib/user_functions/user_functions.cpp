/**\mainpage
 *
 * File		menu.c
 * @date	16 Oct 2018
 * @version	1.0.0
 *
 */

/*! @file user_functions.c
 @brief Low-level functions for writing/reading I2C data */

#include "user_functions.h"
#include "EEPROM_utils.h"

/* Measured values */
uint16_t iaq_val;
uint8_t acc_val;
float temp_val;
float p_val;
uint8_t rh_val;

uint8_t init_I2C(void)
{
    Wire.begin(21,22,0);    
    Wire1.begin(18,19,0);
    return 0;
}

 /*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */
 
    /* Write the data */
    for (int index = 0; index < data_len; index++) {
        Wire.write(reg_data_ptr[index]);
    }
 
    return (int8_t)Wire.endTransmission();
}

/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();
 
    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */
 
    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
 
    return comResult;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep_BME(uint32_t t_ms)
{
    delay(t_ms);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    /* Populate global variables for the measured values so that data can be retrieved in main */
    iaq_val = (uint16_t)iaq;
    //iaq_val = iaq_val | 0x0800; // add 3 sec interval mask
    acc_val = iaq_accuracy;
    temp_val = temperature;
    p_val = pressure/1000;
    rh_val = (uint8_t)humidity;

    // Serial.print("  [  ");
    // Serial.print(timestamp/1e6);
    // Serial.println("] T: ");
    // Serial.print(temperature);
    // Serial.print("| rH: ");
    // Serial.print(humidity);
    // Serial.print("| IAQ: ");
    // Serial.print(iaq);
    // Serial.print(" (");
    // Serial.print(iaq_accuracy);
    // Serial.print("| Static IAQ: ");
    // Serial.print(static_iaq);
    // Serial.print("| CO2e: ");
    // Serial.print(co2_equivalent);
    // Serial.print("| bVOC: ");
    // Serial.print(breath_voc_equivalent);
    // Serial.println(")");
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    if(EEPROM_get_state_flag() == 't')//changed from 'b' to 't' so that state is never loaded 
                                      //leads to unpredictable operation of the sensor
    {
        /* Load the state string from a previous use of BSEC, stored in EEPROM */
        for(int i = 0; i <= n_buffer; i++)
        {
            state_buffer[i] = EEPROM.read(STATE_ADDR + i);
        }

        Serial.println("State loaded from EEPROM.");

        return n_buffer;
    }
    Serial.println("No state saved in EEPROM!");
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    /* Signal that EEPROM has been initialized with STATE */
    EEPROM.put(STATE_FLAG, 'b');

    /* Write to the EEPROM byte by byte */
    for(int i = 0; i <= length; i++)
    {
        EEPROM.put(STATE_ADDR + i, state_buffer[i]);
    }

    /* Save the changes to the EEPROM */ 
    EEPROM.commit();

    Serial.println("State saved.");
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    /* Load a provided config string into serialized_settings */
    for (int i = 0; i < n_buffer; i++)
    {
        //Mode 1 - measurement at 5 min
        config_buffer[i] = bsec_config_iaq[i];
    }

    return n_buffer;
}

/**
 * @brief Get the temp object
 * 
 * @return float Temperature value
 */
float get_temp(void)
{
    return temp_val;
}

/**
 * @brief Get the pressure object
 * 
 * @return float Presure value
 */
float get_p(void)
{
    return p_val;   
}

/**
 * @brief Get the iaq object
 * 
 * @return uint16_t IAQ value
 */
uint16_t get_iaq_RAM_val(void)
{
    return iaq_val;
}

/**
 * @brief Get the iaq accuracy object
 * 
 * @return uint8_t IAQ acc value
 */
uint8_t get_iaq_RAM_acc_val(void)
{
    return acc_val;
}

/**
 * @brief Get the rh object
 * 
 *  @return float RH value
 */
uint8_t get_rh(void)
{
    return rh_val;
}