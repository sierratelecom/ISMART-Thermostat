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
/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    MODE_OFF = 0,
    MODE_HEAT,
    MODE_COOL,
    MODE_AUTO,
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
    uint16_t state;
    int32_t home_set_point;
    int32_t away_set_point;
    uint32_t heat_on;
    uint32_t cool_on;
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
    .cool_on = false,
    .fan_on = false,
    .current_mode = MODE_OFF,
};
/******************************************************
 *               Function Declarations
 ******************************************************/
void init_hardware(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/

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

}

/* [] END OF FILE */
