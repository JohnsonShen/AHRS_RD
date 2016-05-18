/*================================================================================*
 * O     O          __             ______  __   __  ____     __  ___          __  *
 *  \   /      /\  / /_      _    / /___/ / /  / / / __ \   / / /   \    /\  / /  *
 *   [+]      /  \/ / \\    //   / /____ / /  / /  \ \_    / / | | | |  /  \/ /   *
 *  /   \    / /\  /   \\__//   / /----// /__/ /  \ \__ \ / /  | | | | / /\  /    *
 * O     O  /_/  \/     \__/   /_/      \_ ___/    \___ //_/    \___/ /_/  \/     *
 *                                                                                *
 *                                                                                *
 * Nuvoton Sensor Fusion Application Firmware for Cortex M4 Series                *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#include <stdio.h>
#include <stdlib.h>
#ifdef M451
#include "M451Series.h"
#else
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvUSB.h"
#include "Driver\DrvI2C.h"
#endif
#include "Def.h"
#include "FlashCtrl.h"
#include "NVT_I2C.h"
#include "Timer_Ctrl.h"
#include "I2CDev.h"
#include "retarget.h"
#include "AHRSLib.h"
#include "Sensors.h"
#include "Report.h"
#include "Calibrate.h"

#define MAG_INTERVAL 4
void setupSystemClock()
{
#ifdef M451
	SYS_UnlockReg();
	/* Enable HIRC clock */
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

	/* Waiting for HIRC clock ready */
	CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

	/* Switch HCLK clock source to HIRC */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

	/* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
	CLK_SetCoreClock(SYSTEM_CLOCK);
	CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);
	SYS_LockReg();
#else
	uint32_t u32PllCr;
	uint16_t i;
	UNLOCKREG();	
	DrvSYS_SetOscCtrl(E_SYS_OSC22M, 1);
	while (DrvSYS_GetChipClockSourceStatus(E_SYS_OSC22M) != 1);
	DrvSYS_SelectPLLSource(E_SYS_INTERNAL_22M);
	u32PllCr = DrvSYS_GetPLLContent(E_SYS_INTERNAL_22M, SYSTEM_CLOCK);	
	/*Delay for 12M or 22M stable*/
	for (i=0;i<10000;i++);		

	DrvSYS_SetPLLContent(u32PllCr);
	SYSCLK->PLLCON.OE     = 0;
	SYSCLK->PLLCON.PD 	  = 0;

	/*Delay for PLL stable*/
	for (i=0;i<10000;i++);
	/* Change HCLK clock source to be PLL. */
	DrvSYS_SelectHCLKSource(2);
	LOCKREG();	// Lock the protected registers
#endif
}

void setupUART()
{
#ifdef M451
	/* Enable peripheral clock */
	CLK_EnableModuleClock(UART0_MODULE);
	/* Peripheral clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
	/* Set PD multi-function pins for UART0 RXD, TXD */
	SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;
	/* Reset UART module */
	SYS_ResetModule(UART0_RST);

	/* Configure UART0 and set UART0 Baudrate */
	UART_Open(UART0, 115200);
#else
	STR_UART_T param;
	DrvGPIO_InitFunction(E_FUNC_UART1);
	DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC,3);
	param.u32BaudRate        = UART_BAUD_RATE;
	param.u8cDataBits        = DRVUART_DATABITS_8;
	param.u8cStopBits        = DRVUART_STOPBITS_1;
	param.u8cParity          = DRVUART_PARITY_NONE;
	param.u8cRxTriggerLevel  = DRVUART_FIFO_1BYTES;
	param.u8TimeOut        	 = 0;
	DrvUART_Open(UART_PORT1, &param);
#endif
	//printf("Uart Init - [OK]\n");
}

void setup()
{
	setupSystemClock();
	setupUART();
	printf("*Welcom to Nuvoton Design\n");
	printf("*Nuvoton Sensor Fusion Reference Design Version 1.0\n");
	printf("*Bring Up System..\n");
	setup_system_tick(1000);
	I2C_Init();
	FlashInit();
	nvtAHRSInit();
	SensorsInit();
	printf("*System Ready\n");
	printf("*Command List:\n");
	printf("@ss - Stream Start, the default is Euler angle\n");
	printf("@st - Stream Toggle\n");
	printf("@me - Display mode: euler angle\n");
	printf("@mr - Display mode: raw data of sensors\n");
}
void CommandProcess()
{
	// Read incoming control messages
	if (Serial_available() >= 2)
	{
		char start=Serial_read();
		if (start == '@') {// Start of new control message
			int command = Serial_read(); // Commands
			if (command == 'h') {//Hook AHRS Stack Device
				// Read ID
				char id[2];
				id[0] = GetChar();
				id[1] = GetChar();
				// Reply with synch message
				printf("@HOOK");
				Serial_write(id, 2);
			}
			else if (command == 'c') {// A 'c'calibration command
				SensorCalibration();
			}
			else if (command == 'm') {// Set report 'm'ode
				char mode = GetChar();
				if (mode == 'e') {// Report AHRS by 'e'uler angle
					report_mode = REPORT_AHRS_EULER;
				}
				else if (mode == 'q') {// Report // Report AHRS by 'q'quaternion
					report_mode = REPORT_AHRS_QUATERNION;
				}
				else if (mode == 'r') {// Report sensor 'r'aw data
					report_mode = REPORT_SENSORS_RAW;
				}
			}
			else if (command == 'f') {// Set report 'f'ormat
				char format = GetChar();
				if (format == 'b') {// Report 'b'inary format
					report_format = REPORT_FORMAT_BINARY;
				}
				else if (format == 't') {// Report 't'ext format
					report_format = REPORT_FORMAT_TEXT;
				}
			}
			else if (command == 's') {// 's'tream output control
				char mode = GetChar();
				if (mode == 's') {// 's'tart stream
					stream_mode = STREAM_START;
				}
				else if (mode == 'p') {// 'p'ause stream
					stream_mode = STREAM_PAUSE;
				}
				else if (mode == 't') {// 't'oggle stream
					if(stream_mode==STREAM_START)
						stream_mode = STREAM_PAUSE;
					else
						stream_mode = STREAM_START;
				}
			}
		}
		else { 
			printf("Unknown command.\n");
		} // Skip character
	}
}
// Main Control loop
void loop()
{
	CommandProcess();
	SensorsRead(SENSOR_ACC|SENSOR_GYRO|SENSOR_MAG/*|SENSOR_BARO*/,1);

  while((GetSensorCalState()&&GYRO)==false) {
		SensorsDynamicCalibrate(SENSOR_GYRO);
	}
		nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);

	if((GetFrameCount()%6)==0)
		report_sensors();
	
	IncFrameCount(1);
}

/*-----------------------------------------------------------------------------------*/
/*  Fly Controller Main Function                                                     */
/*-----------------------------------------------------------------------------------*/
int32_t main (void)
{
	setup();
	while(true) loop();
}




