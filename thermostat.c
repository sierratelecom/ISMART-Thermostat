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
/** @file thermostat.c
 *
 *  Created on: Auguest 17, 2017
 *      Author: greg.phillips
 */

/* Header File Includes */
#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "imatrix.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define LED_ON						(0u)
#define LED_OFF						(1u)
#define COOL_ON                     0x01
#define HEAT_ON                     0x02
/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    MODE_OFF = 0,
    MODE_HEAT,
    MODE_COOL,
    MODE_AUTO,
    NO_MODES,
};

enum {
    THERMO_INIT,
    THERMO_HOME,
    THERMO_AWAY,
};
/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct {
    unsigned int settings_loaded    : 1;
    unsigned int home               : 1;
    unsigned int heat_on            : 1;
    unsigned int cooling_on         : 1;
    unsigned int heating_on         : 1;
    uint16_t state;
    int32_t home_set_point;
    int32_t away_high_set_point;
    int32_t away_low_set_point;
    uint32_t fan_on;
    uint32_t current_mode;
} thermostat_t;
/******************************************************
 *                    Structures
 ******************************************************/
thermostat_t thermostat = 
{
    .state = THERMO_INIT,
    .settings_loaded = false,
    .heat_on = false,
    .fan_on = false,
    .cooling_on = false,
    .heating_on = false,
    .home_set_point = 21,
    .away_low_set_point = 12,
    .away_high_set_point = 30,
    .current_mode = MODE_OFF,
};
/******************************************************
 *               Function Declarations
 ******************************************************/

void cool_off(void);
void cool_on(void);
void heat_off(void);
void heat_on(void);
void fan_off(void);
void fan_on(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern int32_t current_temperature;
/******************************************************
 *               Function Definitions
 ******************************************************/


/*******************************************************************************
* Function Name: init_thermostat
********************************************************************************
*
* Summary:
*  This function initializes the thermostat function
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*   None
*******************************************************************************/
void init_thermostat(void)
{
    uint32_t uvalue;
    int32_t ivalue;
    /*
     *  Get Setting from ISMART board
     */
    if( get_AT_control( RESPONSE_INT32, AT_CONTROL_7, &ivalue ) == true ) {
        thermostat.home_set_point = ivalue;
        if( get_AT_control( RESPONSE_INT32, AT_CONTROL_5, &ivalue ) == true ) {
            thermostat.away_high_set_point = ivalue;
            if( get_AT_control( RESPONSE_INT32, AT_CONTROL_6, &ivalue ) == true ) {
                thermostat.away_low_set_point = ivalue;
                if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_3, &uvalue ) == true ) {
                    thermostat.fan_on = ( uvalue == 0 ? false : true );
                    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_2, &uvalue ) == true ) {
                        if( uvalue == 0 ) {
                            thermostat.cooling_on = false;
                            thermostat.heating_on = false;
                        } else if( uvalue == 1 ) {
                            thermostat.cooling_on = true;
                            thermostat.heating_on = false;
                        } else if( uvalue == 2 ) {
                            thermostat.cooling_on = false;
                            thermostat.heating_on = true;
                        }
                        if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_5, &uvalue ) == true ) {
                            if( uvalue >= NO_MODES )
                                uvalue = MODE_OFF;
                            thermostat.current_mode = uvalue;
                            heat_off();
                            cool_off();
                            fan_off();
                            thermostat.state = THERMO_HOME;
                            return;
                        }
                    }
                }
            }
        }
    }

    return;
}
/*******************************************************************************
* Function Name: process_thermostat
********************************************************************************
*
* Summary:
*  This function processes the thermostat mode
*
* Parameters:
*  None
*
* Return:
*  int
*
* Side Effects:
*   None
*******************************************************************************/
void process_thermostat(void)
{
    uint32_t uvalue, new_mode;
    int32_t ivalue;
    
    /*
     * Don't do anyting until initialized
    */
    if( thermostat.state == THERMO_INIT ) {
        init_thermostat();
        return;
    }
    /*
     * Check if we need to switch to away mode
     * 
     * To be added
     *
     */
            
    /*
     * Update any changes to Modes/Set Points
     *
     * Get Home Set Point
     */
    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_7, &ivalue ) == true ) {
        thermostat.home_set_point = ivalue;
    }
    /*
     * Get Away High Set Point
     */
    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_5, &ivalue ) == true ) {
        thermostat.away_high_set_point = ivalue;
    }
    /*
     * Get Away Low Set Point
     */
    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_6, &ivalue ) == true ) {
        thermostat.away_low_set_point = ivalue;
    }
    /*
     * Do we need to turn the Fan On/Off
     */
    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_3, &uvalue ) == true ) {
        if( uvalue == 0 ) {
            if( thermostat.fan_on == true ) {
                fan_off();
            }
        } else {
            if( thermostat.fan_on == false ) {
                fan_on();
            }
        }
    }
    /*
     * Get current Mode
      */
    if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_4, &uvalue ) == true ) {
        if( uvalue >= NO_MODES )
            uvalue = MODE_OFF;
        new_mode = uvalue;
    } else
        new_mode = thermostat.current_mode; // Now the current mode


    switch( thermostat.state ) {
        case THERMO_INIT :
            init_thermostat();
            break;
        case THERMO_AWAY :
        case THERMO_HOME :
            /*
             *  Process current settings to see what we do
             */
            switch( thermostat.current_mode ) {
                case MODE_COOL :
                    if( new_mode != MODE_COOL ) {   // We are changing modes
                        /*
                         * Make Sure the heating is off
                         */
                        if( thermostat.heating_on == true )
                            heat_off();
                    }
                    /*
                     * do we need to cool from current temp
                     */
                    if( current_temperature > thermostat.home_set_point ) {
                        if( thermostat.cooling_on == false )
                            cool_on();
                    } else {
                        if( thermostat.cooling_on == true )
                            cool_off();
                    }
                    break;
                case MODE_HEAT :
                    if( new_mode != MODE_HEAT ) {   // We are changing modes
                        /*
                         * Make Sure the cooling is off
                         */
                        if( thermostat.cooling_on == true )
                            cool_off();
                    }
                    /*
                     * do we need to heat from current temp
                     */
                    if( current_temperature < thermostat.home_set_point ) {
                        if( thermostat.heating_on == false )
                            heat_on();
                    } else {
                        if( thermostat.heating_on == true )
                            heat_off();
                    }
                    break;
                case MODE_AUTO :
                    /*
                     *  Turn on Heating or Cooling depending on temp
                     */
                    if( current_temperature > thermostat.home_set_point ) {
                        /*
                         * We need to cool from current temp
                         */
                        /*
                         *  Make sure the heat is OFF
                         */
                        if( thermostat.heating_on == true )
                            heat_off();
                        /*
                         * Cool if not already cooling
                         */
                        if( thermostat.cooling_on == false )
                            cool_on();
                    } else if( current_temperature < thermostat.home_set_point ) {
                        /*
                         *  Make sure the Cooling is OFF
                         */
                        if( thermostat.cooling_on == true )
                            cool_off();
                        /*
                         * Heat if not already heating
                         */
                        if( thermostat.heating_on == false )
                            heat_on();
                    } else {
                        /*
                        * We are at the right temp - make sure everything is off
                        */
                        if( thermostat.cooling_on == true )
                            cool_off();
                        if( thermostat.heating_on == true )
                            heat_off();
                    }
                    break;
                case MODE_OFF :
                    /*
                     * We are OFF - make sure everything is off
                     */
                    if( thermostat.cooling_on == true )
                        cool_off();
                    if( thermostat.heating_on == true )
                        heat_off();
                    if( thermostat.fan_on == true )
                        fan_off();

                default :
                    break;
                    }
            break;
    }
    thermostat.current_mode = new_mode; // Now the current mode
    uvalue = ( thermostat.cooling_on == true ? COOL_ON : 0 ) | ( thermostat.heating_on == true ? HEAT_ON : 0 );
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_2, &uvalue );
}

void cool_off(void)
{
    uint32_t value;
    
    thermostat.cooling_on = false;
    value = ( thermostat.cooling_on == true ? COOL_ON : 0 ) | ( thermostat.heating_on == true ? HEAT_ON : 0 );
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_2, &value );

    Pin_LED_Blue_Write(LED_OFF);
    
}
void cool_on(void)
{
    uint32_t value;
    
    thermostat.cooling_on = true;
    value = ( thermostat.cooling_on == true ? COOL_ON : 0 ) | ( thermostat.heating_on == true ? HEAT_ON : 0 );
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_2, &value );
    Pin_LED_Blue_Write(LED_ON);
}
void heat_off(void)
{
    uint32_t value;
    
    thermostat.heating_on = false;
    value = ( thermostat.cooling_on == true ? COOL_ON : 0 ) | ( thermostat.heating_on == true ? HEAT_ON : 0 );
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_2, &value );
    Pin_LED_Red_Write(LED_OFF);
    
}
void heat_on(void)
{
    uint32_t value;
    
    thermostat.heating_on = true;
    value = ( thermostat.cooling_on == true ? COOL_ON : 0 ) | ( thermostat.heating_on == true ? HEAT_ON : 0 );
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_2, &value );
    Pin_LED_Red_Write(LED_ON);
}
void fan_off(void)
{
    uint32_t value;
    
    thermostat.fan_on = false;
    value = 0;
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_3, &value );
    Pin_LED_Green_Write(LED_OFF);
    
}
void fan_on(void)
{
    uint32_t value;
    
    thermostat.fan_on = true;
    value = 1;
    send_AT_control( IMATRIX_UINT32, AT_CONTROL_3, &value );
    Pin_LED_Green_Write(LED_ON);
}

/* [] END OF FILE */
