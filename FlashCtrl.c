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
#include <string.h>
#include "retarget.h"
#include "Report.h"
#ifdef M451
#include "M451Series.h"
#else
#include "Driver\DrvUART.h"
#include "Driver\DrvFMC.h"
#endif
#include "AHRSLib.h"
#include "FlashCtrl.h"
#ifdef M451
#define PAGE_SIZE 512 //2K Bytes
uint32_t data_buff[PAGE_SIZE];
#else
#define PAGE_SIZE 128 //512 Bytes
#endif
uint32_t DATA_Flash_Start_ADD;
uint32_t FlashCounter;
float tfloat;

//============================================================================
// DATA FLASH OPERATION
// u32addr : 0-1024 (For 4KBytes Data Flash)
// u32data : 0-0xFFFFFFFF (4Bytes)
//============================================================================
void DATA_FLASH_Write(uint32_t u32addr,uint32_t u32data)
{
	uint32_t i=0;
#ifdef M451
	SYS_UnlockReg();
	FMC_Open();
	
	for(i=0;i<PAGE_SIZE;i++)
		data_buff[i] = FMC_Read(DATA_Flash_Start_ADD+i*4+ u32addr/PAGE_SIZE*2048);
	
	FMC_Erase(DATA_Flash_Start_ADD+u32addr/PAGE_SIZE*2048);
	data_buff[u32addr%PAGE_SIZE]=u32data;
	
	for(i=0; i<PAGE_SIZE; i++) 
		FMC_Write(DATA_Flash_Start_ADD+i*4+ u32addr/PAGE_SIZE*2048, data_buff[i]);
	
	FMC_Close();
	SYS_LockReg();
#else
	uint32_t data_buff[PAGE_SIZE];
	__set_PRIMASK(1);//Avoid interrupt

	UNLOCKREG();
	DrvFMC_EnableISP();

	for(i=0;i<PAGE_SIZE;i++)
		DrvFMC_Read(DATA_Flash_Start_ADD+i*4+ u32addr/128*512, &data_buff[i]);

	DrvFMC_Erase(DATA_Flash_Start_ADD+u32addr/128*512);

	data_buff[u32addr%128]=u32data; 

	for(i=0; i<PAGE_SIZE; i++) 
		DrvFMC_Write(DATA_Flash_Start_ADD+i*4+ u32addr/128*512, data_buff[i]);

	DrvFMC_DisableISP();
	LOCKREG();
	__set_PRIMASK(0);
#endif

}
//============================================================================
// u32addr : 0-1024  
//============================================================================
uint32_t DATA_FLASH_Read(uint32_t u32add)
{
	uint32_t u32data;
#ifdef M451
	SYS_UnlockReg();
	FMC_Open();
	u32data = FMC_Read(u32add*4+DATA_Flash_Start_ADD);
	FMC_Close();
	SYS_LockReg();
#else
	__set_PRIMASK(1);
	UNLOCKREG();
	DrvFMC_EnableISP();
	DrvFMC_Read(u32add*4+DATA_Flash_Start_ADD, &u32data);
	DrvFMC_DisableISP(); 
	LOCKREG();
	__set_PRIMASK(0);
#endif
	return u32data;
}
#ifdef M451
static int  SetDataFlashBase(uint32_t u32DFBA)
{
	uint32_t au32Config[2];

	/* Read current User Configuration */
	FMC_ReadConfig(au32Config, 1);

	/* Just return when Data Flash has been enabled */
	if(!(au32Config[0] & 0x1))
		return 0;

	/* Enable User Configuration Update */
	FMC_EnableConfigUpdate();

	/* Erase User Configuration */
	FMC_Erase(FMC_CONFIG_BASE);

	/* Write User Configuration to Enable Data Flash */
	au32Config[0] &= ~0x1;
	au32Config[1] = u32DFBA;

	if(FMC_WriteConfig(au32Config, 2))
		return -1;

	printf("\nSet Data Flash base as 0x%x.\n", FMC_ReadDataFlashBaseAddr());

	/* Perform chip reset to make new User Config take effect */
	SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;

	return 0;
}
#endif

void FlashInit()
{
#ifdef M451
	SYS_UnlockReg();
	FMC_Open();
	SetDataFlashBase(DATA_FLASH_TEST_BASE);
	DATA_Flash_Start_ADD = FMC_ReadDataFlashBaseAddr();
	FMC_Close();
	SYS_LockReg();
#else
	DrvFMC_EnableISP();
	/* Read Data Flash base address */
	DATA_Flash_Start_ADD = DrvFMC_ReadDataFlashBaseAddr();
	DrvFMC_DisableISP();
#endif
	printf("Flash initilize - [OK]\n");
}
void UpdateFlashCounter()
{
	/* Enable ISP function */
#ifdef M451
	SYS_UnlockReg();
	FMC_Open();
#else
	DrvFMC_EnableISP();
#endif
	//printf("  Data Flash Base Address .................... [0x%08x]\n", DATA_Flash_Start_ADD);
	FlashCounter = DATA_FLASH_Read(0);

	FlashCounter++;
	DATA_FLASH_Write(0,FlashCounter);
#ifdef M451
	FMC_Close();
	SYS_LockReg();
#else
	DrvFMC_DisableISP();
#endif
}
int32_t float2dw(float f)
{
	int32_t* pdw;

	pdw = (int32_t*)&f;
	return *pdw;
}
float dw2float(int32_t dw)
{
	float* pf;

	pf = (float*)&dw;
	return *pf;
}
int32_t i162dw(int16_t i16)
{
	int32_t* pdw;
	int32_t i32;
	i32 = i16;

	pdw = (int32_t*)&i32;
	return *pdw;
}
int16_t dw2i16(int32_t dw)
{
	int16_t* pi16;
	int16_t i16;
	
	i16 = dw;
	pi16 = (int16_t*)&i16;
	return *pi16;
}
void TestFloat()
{
#ifndef M451
	/* Enable ISP function */
DrvFMC_EnableISP();
//printf("  Data Flash Base Address .................... [0x%08x]\n", DATA_Flash_Start_ADD);
	tfloat = dw2float(DATA_FLASH_Read(0));

	tfloat+=0.1f;
	DATA_FLASH_Write(0,float2dw(tfloat));
	DrvFMC_DisableISP();
#endif
}
uint32_t GetFlashCounter(void)
{
	return FlashCounter;
}
float GetFloatCounter(void)
{
	return tfloat;
}
void UpdateFlashCal(int8_t sensorType, bool erase)
{
	uint8_t CalBase, i, QualityFactor;
	float mean[3], scale[3], rotate[9], matrix[MAG_CAL_DATA_SIZE];
	
	if(sensorType&SENSOR_GYRO) {
		CalBase=CAL_BASE_GYRO;
		nvtGetGyroOffset(mean);
		nvtGetGyroScale(scale);
		if(erase) {
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_INVALID));
			return;
		}
		else
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_VALID));
		DATA_FLASH_Write(CalBase++,float2dw(mean[0]));
		DATA_FLASH_Write(CalBase++,float2dw(mean[1]));
		DATA_FLASH_Write(CalBase++,float2dw(mean[2]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[0]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[1]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[2]));
		
		printf("GyroMean.x:%f\n", mean[0]);
		printf("GyroMean.y:%f\n", mean[1]);
		printf("GyroMean.z:%f\n", mean[2]);
		printf("GyroScale.x:%f\n", scale[0]);
		printf("GyroScale.y:%f\n", scale[1]);
		printf("GyroScale.z:%f\n", scale[2]);
}
	if(sensorType&SENSOR_ACC) {
		CalBase=CAL_BASE_ACC;
		nvtGetAccOffset(mean);
		nvtGetAccScale(scale);
		nvtGetAccRotate(rotate);
		if(erase) {
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_INVALID));
			return;
		}
		else
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_VALID));
		DATA_FLASH_Write(CalBase++,float2dw(mean[0]));
		DATA_FLASH_Write(CalBase++,float2dw(mean[1]));
		DATA_FLASH_Write(CalBase++,float2dw(mean[2]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[0]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[1]));
		DATA_FLASH_Write(CalBase++,float2dw(scale[2]));
		for(i=0;i<9;i++)
		  DATA_FLASH_Write(CalBase++,float2dw(rotate[i]));

		printf("AccMean.x:%f\n", mean[0]);
		printf("AccMean.y:%f\n", mean[1]);
		printf("AccMean.z:%f\n", mean[2]);
		printf("AccScale.x:%f\n", scale[0]);
		printf("AccScale.y:%f\n", scale[1]);
		printf("AccScale.z:%f\n", scale[2]);
		printf("M[0][1][2]: %f %f %f\n", rotate[0], rotate[1], rotate[2]);
		printf("M[3][4][5]: %f %f %f\n", rotate[3], rotate[4], rotate[5]);
		printf("M[6][7][8]: %f %f %f\n", rotate[6], rotate[7], rotate[8]);
	}
	if(sensorType&SENSOR_MAG) {
		CalBase=CAL_BASE_MAG;
		nvtGetMagCalMatrix(matrix);
		QualityFactor = nvtGetMagCalQFactor();
		if(erase) {
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_INVALID));
			return;
		}
		else
			DATA_FLASH_Write(CalBase++,i162dw((int16_t)FIELD_VALID));
		for(i=0;i<MAG_CAL_DATA_SIZE;i++) {
			DATA_FLASH_Write(CalBase++,float2dw(matrix[i]));
			printf("MagInvW[%d]:%f\n", i, matrix[i]);
		}
		DATA_FLASH_Write(CalBase++,i162dw((int16_t)QualityFactor));
		printf("Quality Factor:%d\n", QualityFactor);
	}
}
bool GetFlashCal(int8_t sensorType, float* Cal)
{
	uint8_t CalBase, i;
	int16_t Valid;
	bool FlashValid;

	if(sensorType&SENSOR_GYRO) {
		CalBase=CAL_BASE_GYRO;
		Valid = dw2i16(DATA_FLASH_Read(CalBase++));
		if(Valid==FIELD_VALID) {
			for(i = 0; i< GYRO_CAL_DATA_SIZE; i++) {
				Cal[i] = dw2float(DATA_FLASH_Read(CalBase++));
			}
			FlashValid = true;
		}
		else
			FlashValid = false;
	}
	else if(sensorType&SENSOR_ACC) {
		CalBase=CAL_BASE_ACC;
		Valid = dw2i16(DATA_FLASH_Read(CalBase++));
		if(Valid==FIELD_VALID) {
			for(i = 0; i< (ACC_CAL_DATA_SIZE/* + FIELD_VALID_SIZE*/); i++) {
				Cal[i] = dw2float(DATA_FLASH_Read(CalBase++));
			}
			FlashValid = true;
		}
		else
			FlashValid = false;
	}
	else if(sensorType&SENSOR_MAG) {
		CalBase=CAL_BASE_MAG;
		Valid = dw2i16(DATA_FLASH_Read(CalBase++));
		if(Valid==FIELD_VALID) {
			for(i = 0; i< (MAG_CAL_DATA_SIZE/* + FIELD_VALID_SIZE*/); i++) {
				Cal[i] = dw2float(DATA_FLASH_Read(CalBase++));
			}
			FlashValid = true;
			Cal[i] = dw2i16(DATA_FLASH_Read(CalBase++));
		}
		else
			FlashValid = false;
	}
	else
		FlashValid = false;
	
	return FlashValid;
}

void FlashControl()
{
	char Action = GetChar();

	if(Action=='e') {
		char Part = GetChar();
		if(Part=='a') {
			UpdateFlashCal(SENSOR_ACC, true);
			if (report_format == REPORT_FORMAT_TEXT) {
			printf("ACC CalData Erased.\n");
		}
		}
		else if(Part=='g') {
			UpdateFlashCal(SENSOR_GYRO, true);
			if (report_format == REPORT_FORMAT_TEXT) {
			printf("GYRO CalData Erased.\n");
		}
		}
		else if(Part=='m') {
			UpdateFlashCal(SENSOR_MAG, true);
			if (report_format == REPORT_FORMAT_TEXT) {
			printf("MAG CalData Erased.\n");
		}
		}
		else if(Part=='x') {
			UpdateFlashCal(SENSOR_ACC, true);
			UpdateFlashCal(SENSOR_GYRO, true);
			UpdateFlashCal(SENSOR_MAG, true);
			if (report_format == REPORT_FORMAT_TEXT) {
			printf("Flash Data Erased.\n");
		}
	}
}
}
