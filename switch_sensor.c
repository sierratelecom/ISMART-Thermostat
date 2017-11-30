/*
 * Copyright 2017, Sierra Telecom. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Sierra Telecom ("Sierra") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Sierra hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Sierra's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Sierra.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Sierra
 * reserves the right to make changes to the Software without notice. Sierra
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Sierra does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Sierra product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Sierra's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Sierra against all liability.
 */
/** @file switch_sensor.c
 *
 *  Created on: April 26, 2017
 *      Author: greg.phillips
 */

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif


#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "sensors.h"
#include "imatrix.h"
//#include "Switch_1.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define DEBOUNCE_COUNT  5
#define PROVISION_TIME  10

/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    SWITCH_INIT,
    MONITOR_SWITCH,
    DEBOUNCE_ON,
    WAIT_TILL_OFF,
    DEBOUNCE_OFF,
};
/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/*
 * State variable for switch monitor
 */
uint32_t switch_state;
uint32_t switch_on_time = 0;
extern uint32_t rtc_time;
/******************************************************
 *               Function Definitions
 ******************************************************/

/**
  * @brief Initialize switch
  * @param  None
  * @retval : None
  */
void init_switch(void)
{
    switch_state = SWITCH_INIT;
}

/**
  * @brief process the switch
  * @param  None
  * @retval : None
  */
uint32_t count;
bool provision_sent;
void process_switch(void)
{
    uint32_t value;
   
    switch( switch_state ) {
        case SWITCH_INIT :
            /* 
             * Initialize Switch
             */
            switch_state = MONITOR_SWITCH;
            count = 0;
            break;
        case MONITOR_SWITCH :
            if( Switch_1_Read() == 0 ) {    // Switch Active Low
                count = 0;
                switch_state = DEBOUNCE_ON;
            }
            break;
        case DEBOUNCE_ON :
            if( Switch_1_Read() == 0 ) {    // Still low
                count += 1;
                if( count >= DEBOUNCE_COUNT ) {
                    /*
                     * Switch Pressed - Send Notification, Turn on Blue LED - Wait till off again
                     */
                    value = 1;
                    send_AT_sensor( IMATRIX_UINT32, AT_SENSOR_10, &value );
                    switch_on_time = rtc_time;
                    provision_sent = false;
                    switch_state = WAIT_TILL_OFF;
                }
            } else
                switch_state = MONITOR_SWITCH;
            break;
        case WAIT_TILL_OFF :
            /*
             * Check to see if Provision time down has occured and then send request to reenter provision mode
             */
            if( provision_sent == false ) {
                if( rtc_time > ( switch_on_time + PROVISION_TIME ) ) {
                    provision_sent = true;
                    send_AT_command( AT_PROVISION );
                    Pin_LED_Green_Write(0x01);
                }
            }
            if( Switch_1_Read() != 0 ) {
                Pin_LED_Green_Write(0x00);
                count = 0;
                switch_state = DEBOUNCE_OFF;
            }
            break;
        case DEBOUNCE_OFF :
            if( Switch_1_Read() != 0 ) {
                count += 1;
                if( count >= DEBOUNCE_COUNT ) {
                    /*
                     * Switch Pressed - Send Notification, Turn on Blue LED - Wait till off again
                     */
                    value = 0;
                    send_AT_sensor( IMATRIX_UINT32, AT_SENSOR_10, &value );
                    // set_uint32_sensor_data( ASCB_SENSOR_SWITCH, 1 );
                    switch_state = MONITOR_SWITCH;
                }
            } else
                switch_state = WAIT_TILL_OFF;
            break;
        default :
            switch_state = SWITCH_INIT;
            break;
    }

}

/* [] END OF FILE */

