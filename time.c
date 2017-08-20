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
/** @file time.c
 *
 *  Created on: April 26, 2017
 *      Author: greg.phillips
 */



#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "time.h"
#include "imatrix.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define RTC_UPDATE_TIME     10      // Check 10 secs until set
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
static uint16_t rtc_loaded = false;
static uint32_t rtc_check_time;

uint32_t    ms_time, ms_count, rtc_time;
/******************************************************
 *               Function Definitions
 ******************************************************/
void init_time(void)
{
    /*
     * Initialize the real time clock
     */ 
    rtc_loaded = false;
    rtc_time = 0;
    ms_time = 0;
    ms_count = 0;
    rtc_check_time = rtc_time;
    System_Timer_Start();
    isr_counter_Start();
}

/**
  * @brief Get the RTC from the ISMART iMatrix Module
  * @param  None
  * @retval : None
  */


void process_time(void)
{
    uint32_t time;
    
    if( rtc_loaded == false ) {
        if( rtc_time > ( rtc_check_time + RTC_UPDATE_TIME ) ) {
            if( get_AT_control( RESPONSE_UINT32, AT_CONTROL_0, &time ) == true ) {
                /*
                 * We got the time from the ISMART iMatrix Module see if its non zero
                 */
                if( time != 0 ) {
                    rtc_time = time;
                    rtc_loaded = true;
                } else {
                    rtc_check_time = 0; // RTC_1_GetUnixTime();
                }
            }
        }
    }
}

/* [] END OF FILE */
