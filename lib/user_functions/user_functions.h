/*! @file user_functions.h
 @brief Low-level functions for writing/reading I2C data */
/*!
 * @defgroup MENU API
 * @{*/
#ifndef USER_FUNCTIONS_H_
#define USER_FUNCTIONS_H_

#include <Arduino.h>
#include <Wire.h>
#include "bsec_integration.h"
#include "bsec_serialized_configurations_iaq.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

uint8_t init_I2C(void);

 /*!
 * @brief Write operation in either Wire or SPI
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len);

/*!
 * @brief Read operation in either Wire or SPI
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len);

/*!
 * @brief System specific implementation of sleep function
 */
void sleep_BME(uint32_t t_ms);

/*!
 * @brief           Capture the system time in microseconds
 */
int64_t get_timestamp_us();

/*!
 * @brief Handling of the ready outputs
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent);
/*!
 * @brief Load previous library state from non-volatile memory
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);

/*!
 * @brief Save library state to non-volatile memory
 */
void state_save(const uint8_t *state_buffer, uint32_t length);

/*!
 * @brief Load library config from non-volatile memory
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);

/**
 * @brief Get the temp object
 */
float get_temp(void);

/**
 * @brief Get the pressure object
 */
float get_p(void);

/**
 * @brief Get the iaq object
 */
uint16_t get_iaq_RAM_val(void);

/**
 * @brief Get the iaq accuracy object
 */
uint8_t get_iaq_RAM_acc_val(void);

/**
 * @brief Get the rh object
 */
uint8_t get_rh(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* USER_FUNCTIONS_H_ */
/** @}*/