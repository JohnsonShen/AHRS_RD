/*================================================================================*
 *                                                                                *
 *            _    _ _____   _____   ______         _                             *
 *      /\   | |  | |  __ \ / ____| |  ____|       (_)                            *
 *     /  \  | |__| | |__) | (___   | |__ _   _ ___ _  ___  _ __                  *
 *    / /\ \ |  __  |  _  / \___ \  |  __| | | / __| |/ _ \| '_ \                 *
 *   / ____ \| |  | | | \ \ ____) | | |  | |_| \__ \ | (_) | | | |                *
 *  /_/    \_\_|  |_|_|  \_\_____/  |_|   \__,_|___/_|\___/|_| |_|                *
 *                                                                                *
 *                                                                                *
 * Nuvoton A.H.R.S Library for Cortex M4 Series                                   *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#ifndef CONFIG_H_
#define CONFIG_H_
#include "AHRSLib.h"
#define HW__VERSION_CODE 1001
#define USE_I2C_PORT1
#define UART_BAUD_RATE 115200
#define OUTPUT_DATA_INTERVAL 20  //milliseconds
#define DEBUG_PRINT printf
#define DISPLAY_LOOP_TIME 0
#define DISPLAY_SSV_TIME 0
/************************/
/*      ACC/GYRO        */
/************************/
#define MPU6050
//#define LSM6DS3
/************************/
/*        MAG           */
/************************/
//#define HMC5883
//#define AK8975
//#define IST8310
#endif //CONFIG_H_

