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
/** @file pir.c
 *
 *  Created on: Auguest 17, 2017
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

#include "imatrix.h"
#include "pir.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define MOTION_DETECTED             (1u)
#define MOTION_NOT_DETECTED         (0u)
#define THREE_FEET                  (3u)
#define TEN_FEET                    (10u)
#define TWENTY_FEET                 (20u)
#define ADC_CHANNEL_PIR             (3u)
#define SENSOR_RAW_INITIAL          (0)

/* High and low thresholds for the motion detection are determined 
	through experiments */ 
	
/* High Threshold for 3 feet detection (80% of positive peak count) */
#define PIR_WINDOW_HIGH_3FT         (1200)
/* Low Threshold for 3 feet detection (80% of negative peak count) */
#define PIR_WINDOW_LOW_3FT          (-1200)   
/* High Threshold for 10 feet detection (80% of positive peak count) */
#define PIR_WINDOW_HIGH_10FT        (600)
/* Low Threshold for 10 feet detection (80% of negative peak count) */
#define PIR_WINDOW_LOW_10FT         (-600)
/* High Threshold for 20 feet detection (80% of positive peak count) */    
#define PIR_WINDOW_HIGH_20FT        (1200)
/* Low Threshold for 20 feet detection (80% of negative peak count) */   
#define PIR_WINDOW_LOW_20FT         (-1200)

/* Keep PIR Active for at least 5 seconds */
#define PIR_TIMEOUT                 (5)
/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    PIR_INIT,
    PIR_DETECT,
    PIR_DELAY,
};
enum {
    PIR_RESULT_NONE,
    PIR_RESULT_TRUE,
    PIR_RESULT_FALSE,
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
uint16_t check_pir( uint16_t highThreshold, uint16_t  lowThreshold );
/******************************************************
 *               Variable Definitions
 ******************************************************/
extern uint32_t rtc_time;
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize PIR Subsystem
  * @param  None
  * @retval : None
  */
void init_pir(void)
{
    /*
     * Nothing to do all done in HW Setup
     */
}

/**
  * @brief Process PIR
  * @param  None
  * @retval : None
  */

static uint16_t PIR_state = PIR_INIT;
static uint32_t pir_detect_time;
/* Motion detection thresholds */
static int16 highThreshold = PIR_WINDOW_HIGH_3FT;		                           
static int16 lowThreshold = PIR_WINDOW_LOW_3FT;

void process_pir(void)
{
    uint16_t pir_result;
	/* Sensor raw value */
    uint32_t value;
		
    /* Variable that stores the previous detection distance, used for checking if 
        the detection distance is changed by the master(BCP) */
    uint8 DetectionDistance = THREE_FEET;

    /* 
     * Add support code for dynamic distance detection
     *     Add another control set by Apk - if change detected reset mode to INIT
     */
    switch( PIR_state ) {
        case PIR_INIT :
            /* Set the required detection distance */
            switch (DetectionDistance) {
                case THREE_FEET:
                    /* Set second stage PGA gain and thresholds that gives 
                       3 feet detection distance */
                    PIRAmplifierStage2_SetGain(PIRAmplifierStage2_GAIN_1);
                    highThreshold = PIR_WINDOW_HIGH_3FT;
                    lowThreshold = PIR_WINDOW_LOW_3FT;
                    break;
                case TEN_FEET:
                    /* Set second stage PGA gain and thresholds that gives 
                       10 feet detection distance */
                    PIRAmplifierStage2_SetGain(PIRAmplifierStage2_GAIN_2);
                    highThreshold = PIR_WINDOW_HIGH_10FT;
                    lowThreshold = PIR_WINDOW_LOW_10FT;
                    break;
                case TWENTY_FEET:
                    /* Set second stage PGA gain and thresholds that gives 
                        20 feet detection distance */
                    PIRAmplifierStage2_SetGain(PIRAmplifierStage2_GAIN_32);
                    highThreshold = PIR_WINDOW_HIGH_20FT;
                    lowThreshold = PIR_WINDOW_LOW_20FT;
                    break;
                default:
                    /* Set second stage PGA gain and thresholds that gives 
                       3 feet detection distance */
                    PIRAmplifierStage2_SetGain(PIRAmplifierStage2_GAIN_1);
                    highThreshold = PIR_WINDOW_HIGH_3FT;
                    lowThreshold = PIR_WINDOW_LOW_3FT;
                    break;
            }
            PIR_state = PIR_DETECT;
            break;
        case PIR_DETECT :
            if( check_pir( highThreshold, lowThreshold ) == PIR_RESULT_TRUE ) {
                /*
                 * Set notificaiton PIR Motion Detected
                 */
                value = 1;
                send_AT_sensor( IMATRIX_UINT32, AT_SENSOR_3, &value );
                PIR_state = PIR_DELAY;
                pir_detect_time = rtc_time;
            }
            break;
        case PIR_DELAY : 
            pir_result = check_pir( highThreshold, lowThreshold );
            if( pir_result == PIR_RESULT_TRUE) {
                pir_detect_time = rtc_time;
            } else if( pir_result == PIR_RESULT_FALSE ) {
                if( rtc_time > ( pir_detect_time + PIR_TIMEOUT ) ) {
                    /*
                     * PIR Gone away
                     */
                    value = 0;
                    send_AT_sensor( IMATRIX_UINT32, AT_SENSOR_3, &value );
                    PIR_state = PIR_DETECT;
                }
            }
            break;
    }

}

uint16_t check_pir( uint16_t highThreshold, uint16_t  lowThreshold )
{
    uint16_t sensorRawValue;
    
    /* Check if ADC data is ready */
    if(ADC_IsEndConversion(ADC_RETURN_STATUS)) {
        /* Read ADC result */
        sensorRawValue = ADC_GetResult16(ADC_CHANNEL_PIR);
            
        /* Check if motion is detected */
        if((sensorRawValue > highThreshold) ||  (sensorRawValue < lowThreshold))
            return PIR_RESULT_TRUE;
        else
            return PIR_RESULT_FALSE;
    }
    return PIR_RESULT_NONE;
}
/* [] END OF FILE */

