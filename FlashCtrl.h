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
#ifndef _FLASH_CTRL_H
#define _FLASH_CTRL_H
#include "Def.h"
//Calibration Section
#define CAL_BASE	16
#define FIELD_VALID_SIZE 1
#define QUALITY_FACTOR_SIZE 1
#define FIELD_VALID      0x77
#define FIELD_INVALID    0x88
#define CAL_BASE_GYRO	 CAL_BASE
#define CAL_BASE_ACC	(CAL_BASE_GYRO + GYRO_CAL_DATA_SIZE + FIELD_VALID_SIZE)
#define CAL_BASE_MAG	(CAL_BASE_ACC + ACC_CAL_DATA_SIZE + FIELD_VALID_SIZE)



void FlashInit(void);
void UpdateFlashCounter(void);
uint32_t GetFlashCounter(void);
void UpdateFlashCal(int8_t sensorType, bool erase);
bool GetFlashCal(int8_t sensorType, float* Cal);
void TestFloat(void);
float GetFloatCounter(void);
int32_t float2dw(float f);
float dw2float(int32_t dw);
void FlashControl(void);
#endif
