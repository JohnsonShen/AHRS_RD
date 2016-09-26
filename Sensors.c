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
#include <math.h>
#ifdef M451
#include "M451Series.h"
#else
#include "NUC1xx.h"
#endif
#include "MPU6050.h"
#include "SparkFunLSM6DS3.h"
#include "hmc5883l.h"
#include "ak8975.h"
#include "Sensors.h"
#include "FlashCtrl.h"
#include "Timer_Ctrl.h"
#include "LED.h"
#include "Report.h"
SensorInit_T SensorInitState = {false,false,false};
SensorInit_T SensorCalState  = {false,false,false};
CAL_FLASH_STATE_T CalFlashState =  {false,false,false,0xff};
Sensor_T Sensor;
float GyroScale[3];
float AccScale[3];
float GyroOffset[3];
float AccOffset[3];
float MagCalMatrix[10];
#ifdef HMC5883
float magCal[3];
#endif
void temperatureRead(float *temperatureOut)
{
#if STACK_ACC
#if defined(MPU6050) || defined(MPU6500)
	*temperatureOut = (MPU6050_getTemperature()-512)/340+34;
	*temperatureOut = *temperatureOut*25/20;
#endif
#endif
}

#if STACK_GYRO
#if defined(MPU6050) || defined(MPU6500)
#endif
#endif
void DisplayCalACC()
{
  printf("ACC Offset: %f  %f  %f\n", AccOffset[0], AccOffset[1], AccOffset[2]);
  printf("ACC Scale: %f  %f  %f\n", AccScale[0], AccScale[1], AccScale[2]);
}
void DisplayCalGYRO()
{
  printf("GYRO Offset: %f  %f  %f\n", GyroOffset[0], GyroOffset[1], GyroOffset[2]);
  printf("GYRO Scale: %f  %f  %f\n", GyroScale[0], GyroScale[1], GyroScale[2]);
}
/* Sensors Init */
void SensorInitACC()
{
	float Cal[ACC_CAL_DATA_SIZE];
	bool FlashValid;
#if defined(LSM6DS3)
  status_t status;
#endif
	
	if(!SensorInitState.ACC_Done) {
#if defined(MPU6050) || defined(MPU6500)
		SensorInitState.ACC_Done = MPU6050_initialize();
		SensorInitState.GYRO_Done = SensorInitState.ACC_Done;
#else
    LSM6DS3_init();
    status = begin();
    if(status==0)
      SensorInitState.ACC_Done = true;
    else
      SensorInitState.ACC_Done = false;
		SensorInitState.GYRO_Done = SensorInitState.ACC_Done;
#endif
	}
	if(SensorInitState.ACC_Done) {
		printf("ACC connect      - [OK]\n");
		FlashValid = GetFlashCal(SENSOR_ACC, Cal);
		if(FlashValid) {
			CalFlashState.ACC_FLASH = true;
			AccOffset[0] = Cal[0];
			AccOffset[1] = Cal[1];
			AccOffset[2] = Cal[2];
			AccScale[0]  = Cal[3];
			AccScale[1]  = Cal[4];
			AccScale[2]  = Cal[5];
			printf("ACC calibration from - [FLASH]\n");
			
		}
		else {
			AccOffset[0] = 0;
			AccOffset[1] = 0;
			AccOffset[2] = 0;
			AccScale[0] = IMU_G_PER_LSB_CFG;
			AccScale[1] = IMU_G_PER_LSB_CFG;
			AccScale[2] = IMU_G_PER_LSB_CFG;
			printf("ACC calibration from - [DEFAULT]\n");
		}
	printf("Offset: %f  %f  %f\n", AccOffset[0], AccOffset[1], AccOffset[2]);
	printf("Scale: %f  %f  %f\n", AccScale[0], AccScale[1], AccScale[2]);
	nvtSetAccScale(AccScale);
	nvtSetAccOffset(AccOffset);
	nvtSetAccG_PER_LSB(calcAccel(1)/*IMU_G_PER_LSB_CFG*/);
}
	else {
    __disable_irq();
    SYS_UnlockReg();
    SYS_ResetChip();
    printf("ACC connect      - [FAIL]\n");
  }
}
void SensorInitGYRO()
{
	float Cal[GYRO_CAL_DATA_SIZE];
	bool FlashValid;
	if(!SensorInitState.GYRO_Done) {
#if defined(MPU6050) || defined(MPU6500)
		SensorInitState.GYRO_Done = MPU6050_initialize();
		SensorInitState.ACC_Done = SensorInitState.GYRO_Done;
#else

#endif
	}

	if(SensorInitState.GYRO_Done) {
		printf("GYRO connect     - [OK]\n");
		FlashValid = GetFlashCal(SENSOR_GYRO, Cal);
		
		if(FlashValid) {
			CalFlashState.GYRO_FLASH = true;
			GyroOffset[0] = Cal[0];
			GyroOffset[1] = Cal[1];
			GyroOffset[2] = Cal[2];
			GyroScale[0]  = Cal[3];
			GyroScale[1]  = Cal[4];
			GyroScale[2]  = Cal[5];
			printf("GYRO calibration from [FLASH]\n");
			
		}
		else {
			GyroOffset[0] = 0;
			GyroOffset[1] = 0;
			GyroOffset[2] = 0;
			GyroScale[0] = 1.0;
			GyroScale[1] = 1.0;
			GyroScale[2] = 1.0;
			printf("GYRO calibration from - [DEFAULT]\n");
		}
		printf("Offset: %f  %f  %f\n", GyroOffset[0], GyroOffset[1], GyroOffset[2]);
		printf("Scale: %f  %f  %f\n", GyroScale[0], GyroScale[1], GyroScale[2]);
		nvtSetGyroScale(GyroScale);
		nvtSetGyroOffset(GyroOffset);

#if defined(MPU6050) || defined(MPU6500)
		nvtSetGYRODegPLSB(IMU_DEG_PER_LSB_CFG);
#else
    nvtSetGYRODegPLSB(calcGyro(1));
#endif
	}
	else
		printf("GYRO connect     - [FAIL]\n");
}
void SensorInitMAG()
{
	float Cal[MAG_CAL_DATA_SIZE + QUALITY_FACTOR_SIZE];
	bool FlashValid;
	int i;
	
	if(!SensorInitState.MAG_Done) {
#ifdef HMC5883
		SensorInitState.MAG_Done = hmc5883lInit();
		hmc5883lSelfTest();
		hmc5883lGetRatioFactor(&magCal[0],&magCal[1],&magCal[2]);
#endif
#ifdef AK8975
		SensorInitState.MAG_Done = AK8975_initialize();
#endif
	}
	
	if(SensorInitState.MAG_Done) {
		if (report_format == REPORT_FORMAT_TEXT) 
		printf("MAG connect      - [OK]\n");
		FlashValid = GetFlashCal(SENSOR_MAG, Cal);
		
		if(FlashValid) {
			CalFlashState.MAG_FLASH = true;
			for(i=0;i<MAG_CAL_DATA_SIZE;i++)
				MagCalMatrix[i] = Cal[i];
			CalFlashState.MAG_QFACTOR = Cal[i];
			if (report_format == REPORT_FORMAT_TEXT) 
			printf("MAG calibration from - [FLASH], Q:%d\n",CalFlashState.MAG_QFACTOR);
		}
		else {
			/*MagCalMatrix[0] = MAG_CAL0;
			MagCalMatrix[1] = MAG_CAL1;
			MagCalMatrix[2] = MAG_CAL2;
			MagCalMatrix[3] = MAG_CAL3;
			MagCalMatrix[4] = MAG_CAL4;
			MagCalMatrix[5] = MAG_CAL5;
			MagCalMatrix[6] = MAG_CAL6;
			MagCalMatrix[7] = MAG_CAL7;
			MagCalMatrix[8] = MAG_CAL8;
			MagCalMatrix[9] = MAG_CAL9;*/
			for(i=0;i<MAG_CAL_DATA_SIZE;i++)
				MagCalMatrix[i] = 0;
#ifdef HMC5883
			MagCalMatrix[3] = magCal[0];//MAG_GAUSS_PER_LSB;
			MagCalMatrix[4] = magCal[1];//MAG_GAUSS_PER_LSB;
			MagCalMatrix[5] = magCal[2];//MAG_GAUSS_PER_LSB;
#else
			MagCalMatrix[3] = MAG_GAUSS_PER_LSB;
			MagCalMatrix[4] = MAG_GAUSS_PER_LSB;
			MagCalMatrix[5] = MAG_GAUSS_PER_LSB;
#endif
			if (report_format == REPORT_FORMAT_TEXT) 
			printf("MAG calibration from - [DEFAULT], Q:%d\n",CalFlashState.MAG_QFACTOR);
		}
		if (report_format == REPORT_FORMAT_TEXT) {
		printf("M[0][1][2]: %f %f %f\n", MagCalMatrix[0], MagCalMatrix[1], MagCalMatrix[2]);
		printf("M[3][4][5]: %f %f %f\n", MagCalMatrix[3], MagCalMatrix[4], MagCalMatrix[5]);
		printf("M[6][7][8]: %f %f %f\n", MagCalMatrix[6], MagCalMatrix[7], MagCalMatrix[8]);
		printf("M[9]: %f\n", MagCalMatrix[9]);
		}
		nvtSetMagCalMatrix(MagCalMatrix);
		nvtSetMagGaussPLSB(MAG_GAUSS_PER_LSB);
	}
	else {
		if (report_format == REPORT_FORMAT_TEXT)
		printf("MAG connect      - [FAIL]\n");
  }
}
void SensorsInit()
{
#if STACK_ACC
	SensorInitACC();
#endif
#if STACK_GYRO
	SensorInitGYRO();
#endif
#if STACK_MAG
	SensorInitMAG();
#endif
}

/* Sensors Read */
void SensorReadACC()
{
#if STACK_ACC
	int16_t rawACC[3];//,rawGYRO[3];
#if defined(MPU6050) || defined(MPU6500)
	MPU6050_getAcceleration(&rawACC[0],&rawACC[1], &rawACC[2]);
  //MPU6050_getMotion6(&rawACC[0],&rawACC[1], &rawACC[2],&rawGYRO[0],&rawGYRO[1], &rawGYRO[2]);
#else
  rawACC[0] = readRawAccelX();
  rawACC[1] = readRawAccelY();
  rawACC[2] = readRawAccelZ();
#endif
	ACC_ORIENTATION(rawACC[0],rawACC[1],rawACC[2]);
  //GYRO_ORIENTATION(rawGYRO[0],rawGYRO[1],rawGYRO[2]);
#endif
}

void SensorReadGYRO()
{
#if STACK_GYRO
	int16_t rawGYRO[3];
#if defined(MPU6050) || defined(MPU6500)
	MPU6050_getRotation(&rawGYRO[0],&rawGYRO[1], &rawGYRO[2]);
#else
  rawGYRO[0] = readRawGyroX();
  rawGYRO[1] = readRawGyroY();
  rawGYRO[2] = readRawGyroZ();
#endif
	GYRO_ORIENTATION(rawGYRO[0],rawGYRO[1],rawGYRO[2]);
	//printf("Raw GYRO:%d %d %d\n",Sensor.rawGYRO[0], Sensor.rawGYRO[1], Sensor.rawGYRO[2]);
#endif
}

void SensorReadMAG()
{
	int16_t rawMAG[3];
#ifdef HMC5883
	hmc5883lGetHeading(&rawMAG[0],&rawMAG[1], &rawMAG[2]);
#endif
#ifdef AK8975
	AK8975_getHeading(&rawMAG[0],&rawMAG[1], &rawMAG[2]);
#endif
	MAG_ORIENTATION(rawMAG[0],rawMAG[1],rawMAG[2]);
	//printf("Raw Mag:%d %d %d\n",Sensor.rawMAG[0], Sensor.rawMAG[1], Sensor.rawMAG[2]);
}

void SensorsRead(char SensorType, char interval)
{
#if STACK_ACC
	if(SensorType&SENSOR_ACC&&SensorInitState.ACC_Done) {
		SensorReadACC();
		nvtInputSensorRawACC(&Sensor.rawACC[0]);
	}
#endif
#if STACK_MAG
	if(SensorType&SENSOR_MAG&&SensorInitState.MAG_Done) {
		if((GetFrameCount()%interval)==0) {
			SensorReadMAG();
			nvtInputSensorRawMAG(&Sensor.rawMAG[0]);
		}
	}
	else {
		Sensor.rawMAG[0] = 0;
		Sensor.rawMAG[1] = 0;
		Sensor.rawMAG[2] = 0;
		nvtInputSensorRawMAG(&Sensor.rawMAG[0]);
	}
#endif
#if STACK_GYRO
	if(SensorType&SENSOR_GYRO&&SensorInitState.GYRO_Done) {
		SensorReadGYRO();
		nvtInputSensorRawGYRO(&Sensor.rawGYRO[0]);
	}
#endif
}
void SensorsDynamicCalibrate(char SensorType)
{
#if STACK_ACC
	if(SensorType&SENSOR_ACC&&SensorInitState.ACC_Done) {
		/* TBD */
	}
#endif
#if STACK_GYRO
	if(SensorType&SENSOR_GYRO&&SensorInitState.GYRO_Done) {
		if(1/*!SensorCalState.GYRO_Done*/) {
			if(nvtGyroCenterCalibrate()!=STATUS_GYRO_CAL_DONE)
				led_arm_state(LED_STATE_TOGGLE);
			else {
				SensorCalState.GYRO_Done = true;
				led_arm_state(LED_STATE_OFF);
				nvtGetGyroOffset(GyroOffset);
			}
		}
	}
#endif
#if STACK_MAG
	if(SensorType&SENSOR_MAG&&SensorInitState.MAG_Done) {
		if(!SensorCalState.MAG_Done) {
			static float rpy[3],lastY,diff;
			nvtGetEulerRPY(rpy);
			diff = fabsf(rpy[2] - lastY);
			if((diff>0.01f)||(diff==0))
				led_mag_state(LED_STATE_TOGGLE);
			else {
				led_arm_state(LED_STATE_OFF);
				SensorCalState.MAG_Done = true;
			}
			
			lastY = rpy[2];
		}
	}
#endif
}

char GetSensorInitState()
{
	char InitState = 0;
	
	InitState = (((SensorInitState.ACC_Done<<ACC))|((SensorInitState.GYRO_Done<<GYRO))|((SensorInitState.MAG_Done<<MAG)));
	return InitState;
}

char GetSensorCalState()
{
	char CalState = 0;
	CalState = (((SensorCalState.ACC_Done<<ACC))|((SensorCalState.GYRO_Done<<GYRO))|((SensorCalState.MAG_Done<<MAG)));
	return CalState;
}
