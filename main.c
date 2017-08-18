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

/** @file main.c
 *
 *  Created on: April 26, 2017
 *      Author: greg.phillips
 */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "controls.h"
#include "sensors.h"
#include "imatrix.h"
#include "time.h"
#include "temp.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define CYCLE_DELAY     1000     // Process loop every 1 second
/******************************************************
 *                   Enumerations
 ******************************************************/

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

/******************************************************
 *               Function Definitions
 ******************************************************/


/**
  * @brief  ISMART / iMatrix Host Processor Application
  *         Communication with a WICED ISMART module using serial port
  *
  *     This Application is for a Thermostat.
  *     The various components of the PSoC are configured to allow measurement of the items needed to handle the functions of a Thermostat
  *     Sensor Data is sent to the ISMART iMatrix Module for relay to the iMatrix Cloud
  *
  *     This sub system consists of the following controls
  *
  *     1. Red, Green Blue LED
  *     2. Set Point for operation
  *     3. Heat On Control
  *     4. Cool On Control
  *     5. Fan
  *
  *     This sub system consists of the following sensors
  *
  *     1. User Input Switch - User by ISMART / iMatrix to put Thing in Provisioning Mode or full Factory Reset
  *     2. Temperature
  *     3. Humidity
  *     4. Occupancy Sensor
  *     5. Proximity Sensor
  *     6. Lux Level
  *
  *
  * @param  None
  * @retval : int - Never returns
  */


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  This function initializes all the resources, and in an infinite loop, measures the temperature from the sensor 
*  readings and to send the data over I2C.
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
int main()
{

    init_iMatrix_interface();
    init_controls();
    init_sensors();
    
    while( 1 ) {
        process_controls();
        process_sensors();
        CyDelay( CYCLE_DELAY );

    }
}


/* [] END OF FILE */
