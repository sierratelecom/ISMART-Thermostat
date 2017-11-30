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
#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "controls.h"
#include "sensors.h"
#include "imatrix.h"
#include "humidity.h"

/* Constants used to calculate humidity */
/* This is the capacitance of the sensor at 55% RH with 0.1pF resolution */
#define CAPACITANCE_AT_55_RH        (1800)
/* Sensitivity numerator and denominator indicate sensitivity of the sensor */
#define SENSITIVITY_NUMERATOR       (31)
#define SENSITIVITY_DENOMINATOR     (100)
/* Value of reference capacitor.  Note that this value includes the pin capacitance
    and the physical 180pF reference capacitor */
#define CREF                        (1930)
/* Offset Capacitance */
#define COFFSET                     (150)
/* This is raw count equivalent to trace capacitance */
#define OFFSETCOUNT                 (1536)
#define BUFFERSIZE                  (8)
#define READ_WRITE_BOUNDARY         (0)
/* Nominal humidity 55% */
#define NOMINAL_HUMIDITY            (550)
#define HUMIDITY_0_PERCENT          (0)
#define HUMIDITY_100_PERCENT        (1000)
#define HUMIDITY_50                 (500)   

/* Function Prototypes */
void init_humidity_resources(void);
__inline uint16 CalculateCapacitance(uint16 rawCounts, uint16 refSensorCounts);
__inline uint16 CalculateHumidity(uint16 capacitance);

/*******************************************************************************
* Function Name: init_humidity
********************************************************************************
*
* Summary:
* This function initializes all the resources required to measure humidity.
*
* Parameters:
*  None
*
* Return:
* int
*
* Side Effects:
* None
*******************************************************************************/
void init_humidity(void)
{   
    /* Initialize hardware resources */
    init_humidity_resources();
    
}
void process_humidity(void)
{
    
    /* sensor values                                    */

    uint16 humidityRawCounts;	/* Raw count from CapSense Component for the humidity sensor */
	uint16 capacitance;			/* Capacitance of the humidity sensor */
	uint16 humidity;			/* Measured humidity */
	uint16 rawCountsRefCap;     /* Raw count from CapSense Component for the Reference capacitor */
    float float_humidity;
    
    /* Check if CapSense scan is complete */
    if(!(CSD_IsBusy())) {
        humidityRawCounts = CSD_BUTTON0_SNS0_RAW0_VALUE;
        rawCountsRefCap = CSD_BUTTON0_SNS1_RAW0_VALUE;
        /* Convert raw counts to capacitance */
        capacitance = CalculateCapacitance(humidityRawCounts, rawCountsRefCap);
        /* Calculate humidity */
        humidity = CalculateHumidity(capacitance);
        
        float_humidity = (float) ( humidity ) / 10.0;
        send_AT_sensor( IMATRIX_FLOAT, AT_SENSOR_5, &float_humidity );

        /* Scan sensors */
        CSD_ScanAllWidgets();
    }
    
}

/*******************************************************************************
* Function Name: void init_humidity_resources(void)
********************************************************************************
*
* Summary:
*  This function initializes all the hardware resources
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
void init_humidity_resources(void)
{
    
    /* Start CapSense Component */
    CSD_Start();
}

/*******************************************************************************
* Function Name: __inline uint16 CalculateCapacitance(uint16 RawCounts, uint16 RefsensorCounts)
********************************************************************************
*
* Summary:
*  This function calculates capacitance from raw count.
*
* Parameters:
*  uint16 RawCounts - Raw count corresponding to Humidity sensor
*  uint16 RefsensorCounts - Raw count corresponding to Reference capacitor
*
* Return:
*  Capacitance of the Humidity sensor
*
* Side Effects:
*   None
*******************************************************************************/
__inline uint16 CalculateCapacitance(uint16 rawCounts, uint16 refsensorCounts)
{
    return (uint16)((float)(rawCounts - OFFSETCOUNT) * (CREF - COFFSET) / (float)(refsensorCounts - OFFSETCOUNT));
}

/*******************************************************************************
* Function Name: __inline uint16 CalculateHumidity(uint16 Capacitance)
********************************************************************************
*
* Summary:
*  This function calculates humidity from capacitance

* Parameters:
*  uint16 Capacitance - Capacitance of the humidity sensor
*
* Return:
*  Calculated Humidity value
*
* Side Effects:
*   None
*******************************************************************************/
__inline uint16 CalculateHumidity(uint16 capacitance)
{
    int16 humidity;
    int16 delta;
    
    /* Find capacitance difference from nominal capacitance at 55% RH */
    delta = capacitance - CAPACITANCE_AT_55_RH;
    
    /* Calculate humidity from capacitance difference and sensor sensitivity */
    humidity = ((delta * SENSITIVITY_DENOMINATOR) / SENSITIVITY_NUMERATOR) + NOMINAL_HUMIDITY;
    
    /* If humidity is less than zero, limit it to 0; If humidity is greater than 1000 (100%), limit to 1000 */
    humidity = (humidity < HUMIDITY_0_PERCENT) ? HUMIDITY_0_PERCENT : (humidity > HUMIDITY_100_PERCENT) ? HUMIDITY_100_PERCENT : humidity;
    /* Return Humidity value */
    return humidity;
}

/* [] END OF FILE */
