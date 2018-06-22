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
#ifdef M451
#include "M451Series.h"
#else
#include "DrvGPIO.h"
#endif
#include "LED.h"
#include "Def.h"
#include "Sensors.h"
#ifdef M451
#if (BOARD_CODE == 200)
#define IO_STATE_ARM        PB, BIT0	//PA, BIT0
#define IO_ARM              PB0				//PA0
#define IO_STATE_HEAD_MODE  PB, BIT8	//PA, BIT1
#define IO_MAG              PB8
#else
#define IO_STATE_ARM        PA, BIT3
#define IO_ARM              PA3
#define IO_STATE_HEAD_MODE  PA, BIT1
#define IO_MAG              PA1
#endif
#define IO_STATE_HFREE_MODE PA, BIT2
#define IO_HFREE_MODE       PA2
#else
#define IO_STATE_ARM        E_GPB, 8
#endif  
static char ledState = 0;
char GetLedState()
{
	return ledState;
}
void LED_Init(void)
{
	#ifdef M451
	GPIO_SetMode(IO_STATE_ARM, GPIO_MODE_OUTPUT);
	GPIO_SetMode(IO_STATE_HEAD_MODE, GPIO_MODE_OUTPUT);
	GPIO_SetMode(IO_STATE_HFREE_MODE, GPIO_MODE_OUTPUT);
	IO_ARM = 1;
	IO_MAG = 1;
	IO_HFREE_MODE = 1;
#else
	DrvGPIO_Open(IO_STATE_ARM, E_IO_OUTPUT);
	DrvGPIO_SetBit(IO_STATE_ARM);
#endif	
}

void led_arm_state(char state)
{
	if(state==LED_STATE_TOGGLE)
		ledState=ledState^(1<<LED_ARM);	
	else if(state==LED_STATE_ON)
		ledState|=(1<<LED_ARM);
	else if(state==LED_STATE_OFF)
		ledState&=~(1<<LED_ARM);	
}
void led_mag_state(char state)
{
	if(state==LED_STATE_TOGGLE)
		ledState=ledState^(1<<LED_MAG);	
	else if(state==LED_STATE_ON)
		ledState|=(1<<LED_MAG);
	else if(state==LED_STATE_OFF)
		ledState&=~(1<<LED_MAG);	
}

void UpdateLED()
{
	if((ledState&(1<<LED_ARM))==0)
		IO_ARM = 1;
	else
		IO_ARM = 0;
}
