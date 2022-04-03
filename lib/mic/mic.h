/*! @file menu.h
 @brief Functions for managing Noise level measurements */
/*!
 * @defgroup MIC API
 * @{*/
#ifndef MIC_H_
#define MIC_H_

#include <Arduino.h>
#include <driver/i2s.h>
#include "SoundTables.h"
//#include "arduinoFFT.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @brief This API is used to initialize I2S communication.
 */
void init_mic(void);

// /*!
//  * @brief This API is used to get peak to peak value read from 128 samples.
//  */
// void get_peak_to_peak(void);

/*!
 * @brief This API is used to get the max value read from 128 samples.
 */
void get_max(void);

// /**
//  * @brief Get the peak_to_peak_val object
//  */
// int get_peak_to_peak_val(void);

/**
 * @brief Get the max_val object
 * 
 *  @return int max value
 */
int get_max_val(void);

// /**
//  * @brief compute db FS
//  */
// int compute_db_FS(int value);

/* Smart citizen kit implementation */
/* This piece of code is modified from the repository 
   https://github.com/fablabbcn/smartcitizen-kit-20
   This is protected by the GPL License V3.0
   */
void getReading(void);
/* Modified version 
Added the sencond parameter, samples_read
*/
bool FFT(int32_t *source, int samples_read);
void arm_bitreversal(int16_t * pSrc16, uint32_t fftLen, uint16_t * pBitRevTab);
void arm_radix2_butterfly( int16_t * pSrc, int16_t fftLen, int16_t * pCoef);
void applyWindow(int16_t *src, const uint16_t *window, uint16_t len);
double dynamicScale(int32_t *source, int16_t *scaledSource, int samples_read);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* MIC_H_ */
/** @}*/

