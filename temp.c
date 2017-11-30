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
/** @file sensors.c
 *
 *  Created on: Auguest 12, 2017
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
#define ADC_CHANNEL_VREF			(0u)
#define ADC_CHANNEL_VTH				(1u)
#define LED_ON						(0u)
#define LED_OFF						(1u)
#define TEMPERATURE_THRESHOLD_HIGH	(3000)
#define TEMPERATURE_THRESHOLD_LOW	(2500)
/* IIR Filter coefficients for each signal */
/* Cut off frequency = fs/(2 * pi * iir_filter_constant).  In this project fs ~= 1 ksps.
This results in a cut-off frequency of 4.97 Hz.  We are using IIR filter as FIR requires 
more order of filter to get the same cut-off frequency*/
#define FILTER_COEFFICIENT_TEMPERATURE	(32u)
/* EzI2C Read/Write Boundary */
#define READ_WRITE_BOUNDARY         (0u)
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
void init_hardware(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
int32_t current_temperature;
/******************************************************
 *               Function Definitions
 ******************************************************/
void init_temp_resources(void);


/*******************************************************************************
* Function Name: init_temp
********************************************************************************
*
* Summary:
*  This function initializes all the resources
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
void init_temp(void)
{
    
    /* Initialize hardware resources */
    init_temp_resources();

    return;
}
/*******************************************************************************
* Function Name: process_temp
********************************************************************************
*
* Summary:
*  This function processes the temperature sensor
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
void process_temp(void)
{
    float float_temp;

    /* Variables to hold the the ADC readings */
    int16 adcResultVREF, adcResultVTH;
    
    /* Filter input and output variables for Vref and Vth measurements */
    int16 filterOutputVref=0;
    int16 filterOutputVth=0;
    
    /* Variables to hold calculated resistance and temperature */
    int16 thermistorResistance, temperature;
    
    /* Check if the ADC data is ready */
    if( ADC_IsEndConversion( ADC_RETURN_STATUS ) )
    {
        /* Read the ADC result for reference and thermistor voltages */
        adcResultVREF = ADC_GetResult16(ADC_CHANNEL_VREF);
        adcResultVTH = ADC_GetResult16(ADC_CHANNEL_VTH);
            
        /* Low pass filter the measured ADC counts of Vref */            
        filterOutputVref = (adcResultVREF + (FILTER_COEFFICIENT_TEMPERATURE - 1) * filterOutputVref) / FILTER_COEFFICIENT_TEMPERATURE;
                    
        /* Low pass filter the measured ADC counts of Vth */         
        filterOutputVth = (adcResultVTH + (FILTER_COEFFICIENT_TEMPERATURE - 1) * filterOutputVth) / FILTER_COEFFICIENT_TEMPERATURE;
                        
        /* Calculate thermistor resistance */
        thermistorResistance = Thermistor_GetResistance(filterOutputVref, filterOutputVth);           
            
        /* Calculate temperature in degree Celsius using the Component API */
        temperature = Thermistor_GetTemperature(thermistorResistance);
            
        /* Turn ON Blue LED if Temperature is <= 25°C */ 
        /* Turn ON both Blue and Red LEDs if the temperature is >25°C and <=30°C */
        /* Turn ON Red LED if temperature is >30°C */
        /*
        * - Not Used
        if (temperature <= TEMPERATURE_THRESHOLD_LOW) {
            Pin_LED_Blue_Write(LED_ON);
            Pin_LED_Red_Write(LED_OFF);
        } else if ((temperature > TEMPERATURE_THRESHOLD_LOW) && (temperature < TEMPERATURE_THRESHOLD_HIGH)) {
            
            Pin_LED_Blue_Write(LED_ON);
            Pin_LED_Red_Write(LED_ON);
        } else {
            Pin_LED_Blue_Write(LED_OFF);
            Pin_LED_Red_Write(LED_ON);
        }   
        */
        float_temp = (float) ( temperature ) / 100.0;
        current_temperature = (int32_t) ( temperature ) / 100;
        send_AT_sensor( IMATRIX_FLOAT, AT_SENSOR_9, &float_temp );
    }
    
}

/*******************************************************************************
* Function Name: void init_temp_resources(void)
********************************************************************************
*
* Summary:
*  This function initializes all the hardware resources for temp reading
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
void init_temp_resources(void)
{
    /*
     * Nothing to do here all done in H/W Setup
     */
}

/* [] END OF FILE */
