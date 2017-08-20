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

#include "switch_sensor.h"
#include "temp.h"
#include "humidity.h"
#include "lux.h"
#include "pir.h"
#include "thermostat.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define LED_ON						(0u)
#define LED_OFF						(1u)
#define LED_DELAY                   (100)
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

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize any sensors
  * @param  None
  * @retval : None
  */
void init_sensors(void)
{
        
    /* Enable global interrupts */
    CyGlobalIntEnable;
    init_hardware();
    init_switch();
    init_temp();
    init_humidity();
    init_lux();
    init_pir();
    init_thermostat();

}

/**
  * @brief Process sensors
  * @param  None
  * @retval : None
  */

void process_sensors(void)
{
    process_switch();
    process_temp();
    process_humidity();
    process_lux();
    process_pir();
    process_thermostat();
}
/**
  * @brief initialize hardware for sensors and controls
  * @param  None
  * @retval : None
  */

void init_hardware(void)
{
    uint16_t i;
    	/* Start the Scanning SAR ADC Component and start conversion */
	ADC_Start();
	ADC_StartConvert();

	/* Start the trans-impedance amplifier (TIA) */
	Opamp_TIA_1_Start();
    
    /* Start Reference buffer */
	RefBuffer_Start();
    
	/* Start Programmable Voltage Reference */
	PVref_1_Start();
    
    /* Enable Programmable Voltage Reference */
    PVref_1_Enable();
    /*
     * PIR Related
     */
       
    /* Start the Reference Buffer */
    RefBuffer_Start();
    
    /* Start the first stage amplifier */
    PIRAmplifierStage1_Start();
    
    /* Start the second stage amplifier (PGA) */
    PIRAmplifierStage2_Start();    
    /*
     * LEDS - pretty little light show on H/W Init
     */
    for( i = 0; i < 3; i++ ) {
        Pin_LED_Red_Write(LED_ON);
        Pin_LED_Blue_Write(LED_OFF);
        CyDelay( LED_DELAY );
        Pin_LED_Green_Write(LED_ON);
        Pin_LED_Red_Write(LED_OFF);
        CyDelay( LED_DELAY );
        Pin_LED_Blue_Write(LED_ON);
        Pin_LED_Green_Write(LED_OFF);
        CyDelay( LED_DELAY );
    }
    /*
     * All Off
     */
    Pin_LED_Red_Write(LED_OFF);
    Pin_LED_Green_Write(LED_OFF);
    Pin_LED_Blue_Write(LED_OFF);
}
/* [] END OF FILE */

