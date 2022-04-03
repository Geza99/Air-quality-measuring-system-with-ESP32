/*! @file menu.h
 @brief Menu for setting SSID and Password */
/*!
 * @defgroup MENU API
 * @{*/
#ifndef MENU_H_
#define MENU_H_

#include <Arduino.h>

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @brief This API is used to activate the menu.
 *        After a period of inactivity or after 
 *        a wrong combination of keys (no $$$) the device 
 *        will start to function normally.
*
 *        @param[in] None
 *
 *        @return 1 if menu should be entered
 *                0 if menu not requested
 */
uint8_t menu_combination(void);

/*!
 * @brief This API is used to enter menu.
 *        After a period of inactivity the device 
 *        will start to function normally.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void enter_menu(void);

/*!
 * @brief This API is used to display menu options.
 *        After a period of inactivity the device 
 *        will start to function normally.
 *
 * @param[in] None
 *
 * @return Nothing
 */
void disp_options(void);


#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* MENU_H_ */
/** @}*/