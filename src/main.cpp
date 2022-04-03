/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved. 
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.ino
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <Arduino.h>
#include "bsec_integration.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "menu.h"
#include "EEPROM_utils.h"
#include "user_functions.h"
#include "mic.h"
#include "averages.h"
#include "CO2_measure.h"
#include "Particle.h"
#include <PubSubClient.h>
#include "ThingSpeak.h"

/**********************************************************************************************************************/
/* defines */
/**********************************************************************************************************************/
#define EEPROM_SIZE         1024
#define SAVE_STATE_SAMPLES 30000

/*LED RGB*/
const int ledPinRED = 10;
const int ledPinGREEN = 11;
const int ledPinBlue = 6;

/**********************************************************************************************************************/
/* global variables */
/**********************************************************************************************************************/
/* connection parameters */
const char* ssid = "yourNetworkName";
const char* password = "yourNetworkPassword";
/* interrupt flag */
volatile int interruptCounter;
/* interrupt (seconds) counter */
uint64_t totalInterruptCounter;
/* hardware timer */ 
hw_timer_t * timer = NULL;
/* portMUX_TYPE variable used synchronize the main loop and the ISR */
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* Establish connection timer counter */
uint8_t connectionCounter;
// Removed because load state does not work properly
// /* Save state counter */
// uint32_t saveCounter = 0;
/* state save flag */
uint8_t save = 0;

/* Number of failed requests */
uint8_t failed_requests = 0;


/*all the constant values may be changed */
WiFiClient GezaClient;  //GezaClient
PubSubClient client(GezaClient);

/*Thingspeak parameters*/
unsigned long channelnumber=1592126;
const char *APIKEY="IK5JSHSI28HZA1B9";

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief       ISR function.
 *              Sets flag so that interrupt is signalled in the main loop.
 *
 * @return      None
 */
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

/*!
 * @brief       Write explanation
 *
 * @return      None
 */
 void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived : ");
  Serial.print(topic);
  Serial.print(" : ");
  for (int i = 0; i < length; i++)
  {
    Serial.println((char)payload[i]);
  }
  if ((char)payload[0] == 'o' && (char)payload[1] == 'n')
  {
    digitalWrite(2, LOW);
  }
  else if ((char)payload[0] == 'o' && (char)payload[1] == 'f' && (char)payload[2] == 'f' ) 
  {
    digitalWrite(2, HIGH);
  }
}

/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void setup()
{   
    ThingSpeak.begin(GezaClient);
    
    /* Init I2C and serial communication */
    Wire.begin();
    Serial.begin(115200);
    pinMode (ledPinRED, OUTPUT);   ////Red pin 10
    pinMode (ledPinGREEN, OUTPUT); //Green pin 11
   // pinMode (ledPinBlue, OUTPUT);  //Blue pin 6
    
    /* Blinking the led*/
    digitalWrite (ledPinRED, LOW);	    // turn on the RED LED
    digitalWrite (ledPinGREEN, LOW);	// turn on the GREEN LED
    //digitalWrite (ledPinBlue, LOW);	   // turn on the Blue LED
    delay(500);
    digitalWrite (ledPinRED, HIGH);	    // turn off the RED LED
    digitalWrite (ledPinGREEN, HIGH);	// turn off the GREEN LED
    //digitalWrite (ledPinBlue, HIGH);	// turn off the Blue LED

    /* BSEC return value */
    return_values_init ret;

    /* Initialize EEPROM */
    EEPROM.begin(EEPROM_SIZE);
    /* Set timeout to 5 seconds when entergin data */
    Serial.setTimeout(5000);

    /* Check EEPROM flag to see if defaults already written */
    /* This will happen only once, after programming */
    if (EEPROM_get_flag()!='x')
    {
        /* Write defaults to memory */
        Serial.print("Writing defaults to memory");
        delay(300);
        Serial.print(".");
        delay(300);
        Serial.print(".");
        delay(300);
        Serial.println(".");
        EEPROM_write_defaults();

        // Here we should set default state, and write it to memory
    }

    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 5.0f, bus_write, bus_read, sleep_BME, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        Serial.println("Error while initializing BME680");
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library");
        return;
    }
    
    /* Display BSEC version */
    bsec_version_t  version;
    bsec_get_version(&version);
    Serial.printf("BSEC version: %d.%d.%d.%d\r\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    /* Initialize I2S */
    init_mic(); 

    /* Enter the main menu for changing connection parameters, if desired ($$$ pressed) */
    if(menu_combination())
    {
        enter_menu();
    }

    /* Read data from EEPROM */
    EEPROM_read_data(SSID_ADDRESS,&ssid);
    EEPROM_read_data(PASS_ADDRESS,&password);

    /* Set prescaler toi 80 on timer so that it count microseconds (clock is 80MHz). */
    timer = timerBegin(0, 80, true);
    /* Bind timer to a handling function. */
    timerAttachInterrupt(timer, &onTimer, true);
    /* Specify the counter value in which the timer interrupt will be generated. */
    timerAlarmWrite(timer, 1000000, true);
    /* Enable the timer with a call to the timerAlarmEnable function. */
    timerAlarmEnable(timer);

    /* Initialize counter for computing averages */
    reset_counter();
    reset_stored_values();

}

#define SAMPLES 128 // make it a power of two for best DMA performance

void loop()
{
    
    /* MAC address */
    String mac = String(WiFi.macAddress());

    /* An interrupt occured (1 second passed) */
    if (interruptCounter > 0) 
    {
        /* Clear the interrupt */
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);

        /* Increase the total interrupt counter, performed every second */
        totalInterruptCounter++;
        /* Increase connection counter */
        connectionCounter++;
        // Removed because load state does not work properly
        // /* Increase saveCounter */
        // saveCounter++;

        // toggle GREEN LED each second
        if ((totalInterruptCounter%2)==0)
        { 
            digitalWrite (ledPinGREEN, LOW);	// turn on the GREEN LED
        }
        else
        {
            digitalWrite (ledPinGREEN, HIGH);	// turn off the GREEN LED
        }

        /* sample BME680 sensor each 3 seconds */
        if ((totalInterruptCounter%3)==0)
        {
            /* We should sample the BME680 sensor here and disp data */
            bsec_iot_op(sleep_BME, totalInterruptCounter, output_ready, state_save, 0, get_timestamp_us);
            // Add curretn values to sum for average and increase number of values counter
            add_to_stored_values(get_iaq_RAM_val(), get_max_val());
            increase_counter();
            
            getReading();

            Serial.print("Connection Counter     ");
            Serial.println(connectionCounter);
            Serial.print("Current IAQ value   ");
            Serial.println(get_iaq_RAM_val());
            Serial.print("Loudness value      ");
            Serial.println(get_max_val()/10);
            Serial.print("Temperature value     ");
            Serial.println(get_temp());
           

            /*Setting ThingSpeak fields*/
            ThingSpeak.setField(1,get_iaq_RAM_val()); //Setting Air quality field on ThingSpeak platform
            ThingSpeak.setField(2,get_max_val()/10);  //Setting loudness field on ThingSpeak platform
            ThingSpeak.setField(3,connectionCounter);  //Setting loudness field on ThingSpeak platform
            ThingSpeak.writeFields(channelnumber,APIKEY); //Writing ThingSpeak fields
            Serial.println("Data sent succesfully to thingspeak!");

                      /*
                   ThingSpeak.writeField(channelnumber,1,get_iaq_RAM_val(),APIKEY);
                   ThingSpeak.writeField(channelnumber,2,compute_loudness_average()/10,APIKEY);
                   ThingSpeak.writeField(channelnumber,3,connectionCounter,APIKEY);
                     */
        }

        /* Reset connection timer every 60 seconds */
        if ((totalInterruptCounter%60)==0)
        {
            Serial.println("Ending loop (60s).");
            connectionCounter=0;
        }
        /* Connect to wiFi every second after 3 readings of the BME (10 seconds) */
        if(connectionCounter==1)
        {
            Serial.println("Setting connection parameters.");
            IPAddress local_IP(0, 0, 0, 0);
            IPAddress gateway(192, 168, 1, 254);
            IPAddress subnet(255, 255, 255, 0);
            IPAddress primaryDNS(8, 8, 8, 8); //optional
            IPAddress secondaryDNS(8, 8, 4, 4); //optional
            WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
            WiFi.begin(ssid,password);
            Serial.print("Connecting          ");
            Serial.println(connectionCounter);
        }
        /* If connected to network send the data */
        else if(connectionCounter==5)
        {
            Serial.println("Checking connection status.");
            if (WiFi.status() == WL_CONNECTED)
            {     
                digitalWrite (ledPinRED, HIGH);	// turn off the RED LED
                Serial.println("Checking Thingspeak server connection status.");
                
                /* Try connecting to Thingspeak server */
                Serial.println("Attempting Thingspeak connection");
            } 
            else
            Serial.println("No internet connection!") ;
        }
    }    
}

/*! @}*/

