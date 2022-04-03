/*! @file averages.h
 @brief Menu for computeing averages for air quality and loudness */
/*!
 * @defgroup AVERAGES API
 * @{*/
#ifndef AVERAGES_H_
#define AVERAGES_H_

#include <Arduino.h>

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @brief This API is used to increase no of values counter.
 */
void increase_counter(void);

/*!
 * @brief This API is used to reset no of values counter.
 */
void reset_counter(void);

/*!
 * @brief This API is used to reset stored values for air
 *        quality and loudness.
 */
void reset_stored_values(void);

/*!
 * @brief This API is used to add to the stored values for air
 *        quality and loudness the current measurements.
 */
void add_to_stored_values(int air_q, int loudness);

/*!
 * @brief This API is used to compute average for air quality.
 */
int compute_air_q_average(void);

/*!
 * @brief This API is used to compute average for loudness.
 */
int compute_loudness_average(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* MENU_H_ */
/** @}*/