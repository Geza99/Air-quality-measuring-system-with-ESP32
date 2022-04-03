/**\mainpage
 *
 * File		averages.cpp
 * @date	15 Nov 2018
 * @version	1.0.0
 *
 */

/*! @file menu.c
 @brief API for computing averages for air quality and loudness */
#include "averages.h"

/* Variables used for computing average for loudness and air quality */
static int value_no;
static int AQ_value;
static int loudness_value;

/*!
 * @brief This API is used to increase no of values counter.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void increase_counter(void)
{
    value_no++;
}

/*!
 * @brief This API is used to reset no of values counter.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void reset_counter(void)
{
    value_no = 0;
}

/*!
 * @brief This API is used to reset stored values for air
 *        quality and loudness.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void reset_stored_values(void)
{
    AQ_value = 0;
    loudness_value = 0;
}

/*!
 * @brief This API is used to add to the stored values for air
 *        quality and loudness the current measurements.
 * 
 * @param[in] air_q    current air quality value
 * @param[in] loudness current loudness value
 *
 * @return Nothing
 */
void add_to_stored_values(int air_q, int loudness)
{
    AQ_value += air_q;
    loudness_value += loudness;
}

/*!
 * @brief This API is used to compute average for air quality.
 *
 * @param[in] None
 *
 * @return Average of air quality.
 */
int compute_air_q_average(void)
{
    return int(AQ_value/value_no);
}

/*!
 * @brief This API is used to compute average for loudness.
 *
 * @param[in] None
 *
 * @return Average of loudness.
 */
int compute_loudness_average(void)
{
    return int(loudness_value/value_no);
}