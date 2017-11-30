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
#define IMATRIX_UINT32      0x00
#define IMATRIX_INT32       0x01
#define IMATRIX_FLOAT       0x02
#define IMATRIX_VARLENGTH   0x03

#define AT_SENSOR_0         0    // On Board Temp
#define AT_SENSOR_1         1    // Wi Fi BSSID
#define AT_SENSOR_2         2    // Wi Fi CHannel
#define AT_SENSOR_3         3    // Wi Fi RF Noise
#define AT_SENSOR_4         4    // Wi Fi RSSI
#define AT_SENSOR_5         5    // Humidity
#define AT_SENSOR_6         6    // Inductive Prox
#define AT_SENSOR_7         7    // Light
#define AT_SENSOR_8         8    // Occupancy
#define AT_SENSOR_9         9    // Room Temperature
#define AT_SENSOR_10        10   // Switch

#define AT_CONTROL_0        0    // LED Green
#define AT_CONTROL_1        1    // LED Red
#define AT_CONTROL_2        2    // Cool/Heat 0 - off / 1 - Cool / 2 - Heat
#define AT_CONTROL_3        3    // Fan On
#define AT_CONTROL_4        4    // Mode
#define AT_CONTROL_5        5    // Set Point Away High
#define AT_CONTROL_6        6    // Set Point Away Low
#define AT_CONTROL_7        7    // Set Point Home

/*
 * AT Commands Supported
 */
enum {
    AT_PROVISION = 0,
    NO_AT_COMMANDS
};
/*
 *  Response Types
 */
#define RESPONSE_NONE       0x00
#define RESPONSE_UINT32     0x01
#define RESPONSE_INT32      0x02
#define RESPONSE_FLOAT      0x03

void init_iMatrix_interface(void);
void send_AT_command( uint16_t command );
void send_AT_control( uint16_t data_type, uint16_t s_reg, void *value );
void send_AT_sensor( uint16_t data_type, uint16_t s_reg, void *value );
bool get_AT_control( uint16_t data_type, uint16_t s_reg, void *value );
bool get_AT_response( uint16_t response_type, void *value );


/* [] END OF FILE */
