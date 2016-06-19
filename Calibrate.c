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
#include <stdio.h>
#include <stdlib.h>
#include "retarget.h"
#include "AHRSLib.h"
#include "Timer_Ctrl.h"
#include "FlashCtrl.h"
#include "Sensors.h"
#include "Report.h"
#include "LED.h"
void CalibrationFail()
{
	const char fail = 'f';
	Serial_write((char*)&fail, 1);
}
void AccCalibration()
{
	const char direction = GetChar();
	const char calibration_done = 'd';
	char side = atoi(&direction);
	signed char status;
	
	if (direction == 'z')
		side = 0;
	else
		side = atoi(&direction);
	
	if ((direction == '0')||(direction == 'z'))
		nvtCalACCInit();
	
	do {
		DelayMsec(1);
		SensorsRead(SENSOR_ACC,1);
		status = nvtCalACCBufferFill(side);
	}while(status==STATUS_BUFFER_NOT_FILLED);
	
	if(status==STATUS_BUFFER_FILLED) {
		if (direction == 'z')
			UpdateFlashCal(SENSOR_ACC, false);
		Serial_write((char*)&direction, 1);
	}
	else {
	Serial_write((char*)&calibration_done, 1);
		UpdateFlashCal(SENSOR_ACC, false);
	}
}
void GyroCalibration()
{
	const char axis_done = GetChar();
	const char axis = axis_done - 0x78;
	
	signed char status;
	
	nvtCalGyroInit(axis);
	
	do {
		SensorsRead(SENSOR_GYRO,1);
		DelayMsec(16);
		status=nvtGyroScaleCalibrate(axis);
    led_arm_state(LED_STATE_TOGGLE);
UpdateLED();	
	} while(status==STATUS_GYRO_CAL_RUNNING);	
	
	if(status==STATUS_GYRO_AXIS_CAL_DONE) {
		UpdateFlashCal(SENSOR_GYRO, false);
		Serial_write((char*)&axis_done, 1);
	}
}
void MagCalibration()
{
	const char calibration_done = 'd';
	signed char status;
	uint8_t CalQFactor;
	int16_t RawMAG[3];
	
	nvtCalMAGInit();
	do {
		DelayMsec(320);
		SensorsRead(SENSOR_MAG,1);
		status = nvtCalMAGBufferFill();
		nvtGetSensorRawMAG(RawMAG);
		if (report_format == REPORT_FORMAT_BINARY) {
			Serial_write((char*)RawMAG, 6);
		}
		else if (report_format == REPORT_FORMAT_TEXT) {
			printf("@rM:%d,%d,%d\n",RawMAG[0],RawMAG[1],RawMAG[2]);
		}
	}while(status==STATUS_BUFFER_NOT_FILLED);
	
	if(status==STATUS_CAL_DONE) {
		CalQFactor = nvtGetMagCalQFactor();
		if (report_format == REPORT_FORMAT_BINARY) {
			Serial_write((char*)&calibration_done, 1);
			Serial_write((char*)&CalQFactor, 1);
		}
		else if (report_format == REPORT_FORMAT_TEXT) {
			printf("%c,%d\n",calibration_done,CalQFactor);
		}
	}
}
void SensorCalibration()
{
	char InitState = GetSensorInitState();
	char calibration_sensor = GetChar();
	TIMER_Enable(false); 
	if((calibration_sensor=='a')&&(InitState&SENSOR_ACC)) {       // Do 'a'cc calibration
		AccCalibration();
	}
	else if((calibration_sensor=='g')&&(InitState&SENSOR_GYRO)) {// Do 'g'yro calibration 
		GyroCalibration();
	}
	else if((calibration_sensor=='m')&&(InitState&SENSOR_MAG)) { // Do 'g'yro calibration 
		MagCalibration();
		UpdateFlashCal(SENSOR_MAG, false);
	}
	else                                                        // Fail doing calibration 
		CalibrationFail();
	TIMER_Enable(true);
}
