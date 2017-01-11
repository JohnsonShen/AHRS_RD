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
#include "LED.h"
#define MAG_INTERVAL 4
int update_time;
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
void DisplayCommandList()
{
  printf("*======================================================================*\n"); DelayMsec(10);
  printf("*            _    _ _____   _____   ______         _                   *\n"); DelayMsec(10);
  printf("*      /+   | |  | |  __ + / ____| |  ____|       (_)                  *\n"); DelayMsec(10);
  printf("*     /  +  | |__| | |__) | (___   | |__ _   _ ___ _  ___  _ __        *\n"); DelayMsec(10);
  printf("*    / /+ + |  __  |  _  / +___ +  |  __| | | / __| |/ _ +| '_ +       *\n"); DelayMsec(10);
  printf("*   / ____ +| |  | | | + + ____) | | |  | |_| +__ + | (_) | | | |      *\n"); DelayMsec(10);
  printf("*  /_/    +_+_|  |_|_|  +_+_____/  |_|   +__,_|___/_|+___/|_| |_|      *\n"); DelayMsec(10);
  printf("*======================================================================*\n"); DelayMsec(10);
	printf("*Command List:\n"); DelayMsec(10);
  printf("@dc  - Display Commands\n"); DelayMsec(10);
	printf("@ss  - Stream Start, start displaying euler angle\n"); DelayMsec(10);
	printf("@sp  - Stream Stop\n"); DelayMsec(10);
	printf("@me  - Mode Euler: switch display mode to euler angle\n"); DelayMsec(10);
	printf("@mr  - Mode Raw: switch display mode to sensor raw data\n"); DelayMsec(10);
  printf("@mc  - Mode Calibrated: switch display mode to sensor calibrated data\n"); DelayMsec(10);
  printf("@caz - Calibrate Acc Z, calibrate Acc in Z axis:\n"); DelayMsec(10);
  printf("     1. Put module on a horizontal plan\n"); DelayMsec(10);
  printf("     2. Apply command: @caz (calibration should be done in 1 second)\n"); DelayMsec(10);
  printf("@cgz - Calibrate Gyro Z:calibrate Gyro in Z axis:\n"); DelayMsec(10);
  printf("     1. Put module steady\n"); DelayMsec(10);
  printf("     2. Apply command: @cgz\n"); DelayMsec(10);
  printf("     3. Rotate module right 3 times(1080 degree) around z axis\n"); DelayMsec(10);
  printf("     4. Leave module staedy (calibration shound be done in 2 seconds)\n"); DelayMsec(10);
  printf("@bea - Block Erase Acc, reset ACC calibration\n"); DelayMsec(10);
  printf("@beg - Block Erase Gyro, reset Gyro calibration\n"); DelayMsec(10);
  printf("@fb  - Format Binary, switch display format to binary (GUI protocol)\n"); DelayMsec(10);
  printf("@ft  - Format Text, switch display format to text\n"); DelayMsec(10);
  printf("@da  - Display ACC, display ACC calibration parameters\n"); DelayMsec(10);
  printf("@dg  - Display GYRO, display GYRO calibration parameters\n"); DelayMsec(10);
  printf("@dln - Display Loop On, display loop spped on\n"); DelayMsec(10);
  printf("@dlf - Display Loop Off, display loop off on\n"); DelayMsec(10);
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
  nvtSetGyroDeviationTH(50);
	SensorsInit();
  TIMER_Init();
	DisplayCommandList();
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
      else if (command == 'b') {// 'b'lock erase flash
				FlashControl();
			}
      else if (command == 'd') {// 'd'isplay 'c'ommand list
        char token = GetChar();
        if (token == 'c')
          DisplayCommandList();
        else if (token == 'a')
          DisplayCalACC();
        else if (token == 'g')
          DisplayCalGYRO();
        else if (token == 'l') {
          token = GetChar();
          if (token == 'n')
            SetDisplayLoopTime(true);
          else if (token == 'f')
            SetDisplayLoopTime(false);
        }
			}
      else if (command == 'g') {// 'g'yro dynamic
        char token = GetChar();
        if (token == 'd')
          ToggleGyroDynamicCalibrate();
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
                                else if (mode == 'c') {// Report sensor 'c'alibrated data
					report_mode = REPORT_SENSORS_CALIBRATED;
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
				else if (mode == 'g') {
					char value = GetChar();
					float mean[3];
					nvtGetGyroOffset(mean);
					if (value == 'i') {
						mean[2]+=0.05f;
					}
					else if (value == 'd') {
						mean[2]-=0.05f;
					}
					nvtSetGyroOffset(mean);
					printf("offsetZ:%f\n",mean[2]);
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
  int current_time;
	CommandProcess();
#if STACK_MAG
	SensorsRead(SENSOR_ACC|SENSOR_GYRO|SENSOR_MAG/*|SENSOR_BARO*/,1);
#else
  SensorsRead(SENSOR_ACC|SENSOR_GYRO,1);
#endif
  	if(ChronographRead(ChronRC)>= OUTPUT_RC_INTERVAL) {
		SensorsDynamicCalibrate(SENSOR_GYRO|SENSOR_MAG);
		ChronographSet(ChronRC);
	}
 current_time = micros();
  if(GetSensorCalState()&(1<<GYRO))
#if STACK_MAG
		nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO|SENSOR_MAG);
#else
    nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);
#endif
 update_time = micros() - current_time;

	if((GetFrameCount()%50)==0)
		report_sensors();
	
	IncFrameCount(1);
if((GetFrameCount()%12)==0)
		UpdateLED();
}

/*-----------------------------------------------------------------------------------*/
/*  Fly Controller Main Function                                                     */
/*-----------------------------------------------------------------------------------*/
int32_t main (void)
{
	setup();
	while(true) loop();
}




