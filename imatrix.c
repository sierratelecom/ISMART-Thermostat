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
/** @file imatrix.c
 *
 *  Created on: April 26, 2017
 *      Author: greg.phillips
 */



#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <CyLib.h>
#include <math.h>

#include "imatrix.h"
#include "ascii.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define ISMART_TIMEOUT  5   // 5 Seconds for expected output
/******************************************************
 *                    Constants
 ******************************************************/
#define AT_BUFFER_LENGTH   32

/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    GET_UINT32,
    GET_INT32,
    GET_FLOAT,
    GET_OK,
};
enum {
    LOOK_FOR_CR,
    LOOK_FOR_LF,
    GATHER_DATA,
    LOOK_FOR_2LF,
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
void ftoa(float f, char *str, uint8_t precision);
void flush_rx_uart(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern uint32_t rtc_time;
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Set up the serial port for communication to the ISMART module
  * @param  None
  * @retval : None
  */

void init_iMatrix_interface(void)
{
    uint32_t return_value;
    
    UART_1_Enable();
    UART_1_Init();
    UART_1_Start();
    UART_1_UartDisableCts();
    /*
     * Set up the right mode for future transactions
     */
    UART_1_UartPutString( "AT V1\r" );
    
    get_AT_response( false, &return_value );
    
}
/**
  * @brief Send an AT command 
  * @param  command
  * @retval : None
  */
void send_AT_command( uint16_t command )
{
    char tx_buffer[ AT_BUFFER_LENGTH ];
    uint32_t return_value;
    
    switch( command ) {
        case AT_PROVISION :
            strcpy( tx_buffer, "AT &IP\r" );
            break;
        default :
            /*
             * Ignore
             */
             return;
    }
    
    flush_rx_uart();    // Get all / any data pending

    UART_1_UartPutString( tx_buffer );
    
    get_AT_response( RESPONSE_NONE, &return_value );
}
/**
  * @brief Send and AT command to set a value of a uint32 sensor
  * @param  Sensor Register, value
  * @retval : None
  */
void send_AT_control( uint16_t data_type, uint16_t s_reg, void *value )
{
    char tx_buffer[ AT_BUFFER_LENGTH ];
    uint32_t return_value, foo;
    
    memset( tx_buffer, 0x00, AT_BUFFER_LENGTH );       // Initialize to 0
    strcpy( tx_buffer, "AT &IC" );
    itoa( s_reg, &tx_buffer[ strlen( tx_buffer ) ], 10 );
    strcat( tx_buffer, "=" );
    /*
     * Select data to output and add data
     */
    switch( data_type ) {
        case IMATRIX_UINT32 :
            //sprintf( tx_buffer, "AT &IS%u=%lu\r\n", s_reg, *(uint32_t *) value );
            itoa( *( uint32_t *) value, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            break;
        case IMATRIX_INT32 :
            //sprintf( tx_buffer, "AT &IS%u=%ld\r\n", s_reg, *(int32_t *) value );
            itoa( *( int32_t *) value, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            break;
        case IMATRIX_FLOAT : 
            foo = (uint32_t) *(float *) value;
            //ftoa( *(float *) value, &tx_buffer[ strlen( tx_buffer ) ], 2 );
            itoa( foo, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            //sprintf( tx_buffer, "AT &IS%u=%f\r\n", s_reg, *(float *) value );
            // strcpy( tx_buffer, "AT &IS1=15.9876\r\n" );
            break;
    }
    strcat( tx_buffer, "\r" );
    
    UART_1_UartPutString( tx_buffer );
    
    get_AT_response( RESPONSE_NONE, &return_value );

}
/**
  * @brief Send and AT command to set a value of a uint32 sensor
  * @param  Sensor Register, value
  * @retval : None
  */
void send_AT_sensor( uint16_t data_type, uint16_t s_reg, void *value )
{
    char tx_buffer[ AT_BUFFER_LENGTH ];
    uint32_t return_value, foo;
    
    memset( tx_buffer, 0x00, AT_BUFFER_LENGTH );       // Initialize to 0
    strcpy( tx_buffer, "AT &IS" );
    itoa( s_reg, &tx_buffer[ strlen( tx_buffer ) ], 10 );
    strcat( tx_buffer, "=" );
    /*
     * Select data to output and add data
     */
    switch( data_type ) {
        case IMATRIX_UINT32 :
            //sprintf( tx_buffer, "AT &IS%u=%lu\r\n", s_reg, *(uint32_t *) value );
            itoa( *( uint32_t *) value, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            break;
        case IMATRIX_INT32 :
            //sprintf( tx_buffer, "AT &IS%u=%ld\r\n", s_reg, *(int32_t *) value );
            itoa( *( int32_t *) value, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            break;
        case IMATRIX_FLOAT : 
            foo = (uint32_t) *(float *) value;
            //ftoa( *(float *) value, &tx_buffer[ strlen( tx_buffer ) ], 2 );
            itoa( foo, &tx_buffer[ strlen( tx_buffer ) ], 10 );
            //sprintf( tx_buffer, "AT &IS%u=%f\r\n", s_reg, *(float *) value );
            // strcpy( tx_buffer, "AT &IS1=15.9876\r\n" );
            break;
    }
    strcat( tx_buffer, "\r" );
    
    UART_1_UartPutString( tx_buffer );
    
    get_AT_response( RESPONSE_NONE, &return_value );

}
/**
  * @brief Send and AT command to set a value of a uint32 sensor
  * @param  Sensor Register, value
  * @retval : None
  */
bool get_AT_control( uint16_t data_type, uint16_t s_reg, void *value )
{
    char tx_buffer[ AT_BUFFER_LENGTH ];
    
    memset( tx_buffer, 0x00, AT_BUFFER_LENGTH );       // Initialize to 0
    strcpy( tx_buffer, "AT &IC" );
    itoa( s_reg, &tx_buffer[ strlen( tx_buffer ) ], 10 );
    strcat( tx_buffer, "?\r" );
    flush_rx_uart();    // Get all / any data pending
    UART_1_UartPutString( tx_buffer );
    
    return( get_AT_response( data_type, value ) );

}
/**
  * @brief Get a response for an AT command
  * @param  Sensor Register, value
  * @retval : None
  */
bool get_AT_response( uint16_t response_type, void *value )
{
    bool found;
    volatile char ch;
    char *ptr, rx_buffer[ AT_BUFFER_LENGTH ];
    uint16_t response_state, i;
    volatile uint32_t uart_data, count, start_time;
    
    /*
     * Look for response type and /r/nOK/r/n
     *
     * Return Response value
     *
     * Repsonse types:  RESPONSE_NONE - Just look for OK
     *                  RESPONSE_UINT32 - A unsigned Integer
     *                  RESPONSE_INT32 -  A signed Integer
     *                  RESPONSE_FLOAT - A float
     */
    /*
     * Look first for a CR/LF from the entered request, then get the value and then the 2nd CR LF
     */
    start_time = 0; // RTC_1_GetUnixTime();
    count = 0;
    found = false;
    response_state = LOOK_FOR_CR;
    i = 0;
    
    do {
        count += 1;     // Manual overide until clock working
        uart_data = UART_1_UartGetByte();
        if( ( uart_data & 0xFFFFFF00 ) == 0x00 ) {  // Character recevied is valid
            ch = (char) uart_data & 0xFF;
            switch( response_state ) {
                case LOOK_FOR_CR :
                    if( ch == CR )
                        response_state = LOOK_FOR_LF;
                    break;
                case LOOK_FOR_LF :
                    if( ch == LF ) {
                        response_state = GATHER_DATA;
                        i = 0;
                    }
                    break;
                case GATHER_DATA :
                    if( ch == CR )
                        response_state = LOOK_FOR_2LF;
                    else {
                        rx_buffer[ i++ ] = ch;
                        if( i >= ( AT_BUFFER_LENGTH ) )    // Go looking again
                            response_state = LOOK_FOR_CR;
                    }
                    break;
                case LOOK_FOR_2LF :
                    if( ch == LF ) {
                        switch( response_type ) {
                            case RESPONSE_UINT32 :
                                /*
                                 * Convert buffer to an uint_32_t
                                 */
                                *( (uint32_t *) value) = (uint32_t) strtoul( rx_buffer, &ptr, 10 );
                                break;
                            case RESPONSE_INT32 :
                                /*
                                 * Convert buffer to an uint_32_t
                                 */
                                *( (int32_t *) value) = atoi( rx_buffer );
                                break;
                            case RESPONSE_FLOAT :
                                //*( (float *) value) = atof( rx_buffer );
                                break;
                            case RESPONSE_NONE :
                            default :
                                break;
                        }
                        found = true;
                    }
                break;
            default :
                break;
            }
        }
        /*
         * Check for Time out
         */
        if( ( count > 10000 ) /* || ( rtc_time  > ( start_time + ISMART_TIMEOUT ) ) */ )
            return( false );

    } while( found == false );
    return true;
}
void flush_rx_uart(void)
{
    uint32_t uart_data, count;
    
    count = 0;
    do {
        uart_data = UART_1_UartGetByte();
        if( ( uart_data & UART_1_UART_RX_UNDERFLOW ) != 0x00 ) {  // No more data
            count += 1;
        } else {    // Something in UART
            count = 0;
        }
    } while( count < 10000 );
}
/* [] END OF FILE */
