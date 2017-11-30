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
/** @file lux.c
 *
 *  Created on: Auguest 15, 2017
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
#include "lux.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* IIR Filter coefficient */
/* Cut off frequency = fs/(2 * pi * iir_filter_constant).  In this project fs ~= 1 ksps.
This results in a cut-off frequency of 15.91 Hz.  We are using IIR filter as FIR requires 
more order of filter to get the same cut-off frequency*/
#define FILTER_COEFFICIENT_ALS			        (10)

/* Constants for photodiode current calculation */
/* Scale Factor = (VREF / (2048 * 220K)) * 10^9 nA = 2.6633 
   As the TIA produces a negative voltage, the scale factor is made 
   negative */
#define ALS_CURRENT_SCALE_FACTOR_NUMERATOR		(-26633)
#define ALS_CURRENT_SCALE_FACTOR_DENOMINATOR	(10000)

/* Constants for ambient light calculation */
/* Scale Factor = 10000Lx / 3000nA = 3.333 */
#define ALS_LIGHT_SCALE_FACTOR_NUMERATOR		(3333)
#define ALS_LIGHT_SCALE_FACTOR_DENOMINATOR		(1000)

#define PWM_DUTY_SCALE	        (1u)	
#define PWM_DUTY_OFFSET	        (0)
#define ADC_LUX_CHANNEL_ALS     (2u)	
#define READ_WRITE_BOUNDARY     (0u)

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
void init_lux_resources(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize lux sensors
  * @param  None
  * @retval : None
  */
void init_lux(void)
{
    init_lux_resources();
}
/**
  * @brief process lux sensors
  * @param  None
  * @retval : None
  */
void process_lux(void)
{
    	/* This variable is used to store the ADC result */
	int16 adcResult;
	
	/* These are used for firmware low pass filter input and output */
	int16 filterInput;
	int32 filterOutput = 0;
	
    /* Variable to store sensor current and light illuminance */
    int16 alsCurrent;
    uint16 illuminance;
    /* Calculated LUX */
    float lux;
    
	/* Variable to store the PWM Duty Cycle */
	unsigned int pwmDutyCycle;
    /* Check if the ADC data is ready */
	if(ADC_IsEndConversion( ADC_RETURN_STATUS ) ) {
		/* Get the ADC result */
		adcResult = ADC_GetResult16(ADC_LUX_CHANNEL_ALS);	
			
		/* Low pass filter the ADC result */
		filterInput = adcResult;
    	filterOutput = (filterInput + (FILTER_COEFFICIENT_ALS - 1)*filterOutput)/FILTER_COEFFICIENT_ALS;
    				
		/* Calculate the photodiode current */
		alsCurrent = (filterOutput * ALS_CURRENT_SCALE_FACTOR_NUMERATOR)/ALS_CURRENT_SCALE_FACTOR_DENOMINATOR; 
			
		/* If the calculated current is negative, limit it to zero */
		if(alsCurrent < 0) {
				alsCurrent = 0;
        }
			
		/* Calculate the light illuminance */
		illuminance = (alsCurrent * ALS_LIGHT_SCALE_FACTOR_NUMERATOR)/ALS_LIGHT_SCALE_FACTOR_DENOMINATOR;			
		
        /* Get the PWM duty cycle from the light illuminance value */
		pwmDutyCycle = ((unsigned int)illuminance*PWM_DUTY_SCALE)+PWM_DUTY_OFFSET;
        
        lux = (float) pwmDutyCycle;
        
        send_AT_sensor( IMATRIX_FLOAT, AT_SENSOR_7, &lux);
    }

}
/*******************************************************************************
* Function Name: void InitResources(void)
********************************************************************************
*
* Summary:
*  This function initializes lux resources
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
void init_lux_resources(void)
{
    /*
     * Nothing to do here - all done in HW Setup
     */
}
