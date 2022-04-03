/**\mainpage
 *
 * File		menu.cpp
 * @date	15 Oct 2018
 * @version	1.0.0
 *
 */

/*! @file menu.c
 @brief Menu for setting SSID and Password */
#include "menu.h"
#include "EEPROM_utils.h"

/*!
 * @brief This API is used to activate the menu.
 *        After a period of inactivity or after 
 *        a wrong combination of keys the device 
 *        will start to function normally.
 */
uint8_t menu_combination(void)
{
    /* character red from serial connection */
    char c[3] ={'x','x','x'};
    /* number of key presses */
    uint8_t no_keys = 0;
    /* index for timeout */
    uint8_t x = 0;

    Serial.println("Enter menu?");

    /* Execute until keyboard pressed three times */
    while (no_keys<3)
    {   
        /* If key pressed, then mmeorize it */
        if (Serial.available() > 0)
        {
            /* Read the character */
            c[no_keys] = Serial.read();
            no_keys++;
            x = 0;
        }

        /* No key pressed, so increase timeout counter */
        x++;
        /* Wait for 10ms and then try to read serial again */
        delay(10);

        /* If timeout, then exit */ 
        if ((x>=200)||(no_keys==3))
        {   
            break;
        }
    }

    /* Check to see if enter requested - correct passphrase - $$$*/
    if((c[0]=='$')&&(c[1]=='$')&&(c[2]=='$'))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*!
 * @brief This API is used to enter menu.
 *        After a period of inactivity the device 
 *        will start to function normally.
 */
void enter_menu(void)
{
    /* character red from serial connection */
    char c='x';
    /* index for timeout */
    uint64_t x = 0;
    /* SSID */
    String ssid = "";
    /* PASS */
    String pass = "";

    /* display the options of the menu */
    disp_options();
    
    /* Execute while not QUIT selected and not TIMEOUT */
    while ((c!='q')&&(c!='3'))
    {
        /* If key pressed, then select menu option */
        if (Serial.available() > 0)
        {
            /* Read the character */
            c = Serial.read();

            /* Select option depending on character */
            switch(c){
                case '1':
                    /* Clear the terminal */
                    Serial.write(0x0c);
                    Serial.println("1. Enter new SSID:");
                    /* Get data until ENTER pressed */
                    ssid = Serial.readStringUntil(0x0d);
                    /* Write to EEPROM */
                    EEPROM_write_data(SSID_ADDRESS, ssid , ssid.length());
                    /* Save the changes to the EEPROM */
                    EEPROM.commit();
                    /* display the options of the menu */
                    disp_options();
                    break;
                case '2':
                    /* Clear the terminal */
                    Serial.write(0x0c);
                    Serial.println("2. Enter new password:");
                    /* Get data until ENTER pressed */
                    pass = Serial.readStringUntil(0x0d);
                    /* Write to EEPROM */
                    EEPROM_write_data(PASS_ADDRESS, pass , pass.length());
                    /* Save the changes to the EEPROM */
                    EEPROM.commit();
                    /* display the options of the menu */
                    disp_options();
                    break;
                case '3':
                case 'q':
                    /* Clear the terminal */
                    Serial.write(0x0c);
                    Serial.print("Exiting");
                    delay(300);
                    Serial.print(".");
                    delay(300);
                    Serial.print(".");
                    delay(300);
                    Serial.println(".");
                    break;
                default:
                    break;
            }

            /* Reset timeout counter */
            x = 0;
        }

        /* No key pressed, so increase timeout counter */
        x++;
        /* Wait for 10ms and then try to read serial again */
        delay(10);

        /* If timeout, then exit */ 
        if (x>=1000)
        {   
            /* Set c to q so that While loop exits */
            c='q';
            /* Disp exit message on serial */
            Serial.write(0x0c);
            Serial.print("\r\nExiting");
            delay(300);
            Serial.print(".");
            delay(300);
            Serial.print(".");
            delay(300);
            Serial.println(".");
            delay(1000);
        }
    }
}

/*!
 * @brief This API is used to display menu options.
 *        After a period of inactivity the device 
 *        will start to function normally.
 */
void disp_options(void)
{
    /* buffer */
    const char *buffer;
    /* SSID */
    String ssid = "";
    /* PASS */
    String pass = "";
    
    /* Read data from EEPROM */
    EEPROM_read_data(SSID_ADDRESS,&buffer);
    ssid = String(buffer);
    EEPROM_read_data(PASS_ADDRESS,&buffer);
    pass = String(buffer);

    // Serial.write(0x0c);
    Serial.println("********************************");
    Serial.println("**********WiFi Monitor**********");
    Serial.println("********************************"); 
    Serial.println("1. Enter SSID");
    Serial.print("   Current SSID: ");   
    Serial.println(ssid);
    Serial.println("2. Enter Password");  
    Serial.print("   Current Password: ");   
    Serial.println(pass);
    Serial.println("q. EXIT");  
}

