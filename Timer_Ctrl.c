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
#include "math.h"
#include "Def.h"
#ifdef M451
#include "M451Series.h"
#else
#include "Driver\DrvSYS.h"
#include "DrvTIMER.h"
#endif
#include "Timer_Ctrl.h"
#include "AHRSLib.h"
#include "Sensors.h"
extern int update_time;
typedef struct {
	int lastTime;
	int interval0;
	int currentTime; 
}Chronograph_T;
Chronograph_T Chronograph[NumCron];

float UPDATE_DT;
volatile uint32_t tick_counter = 0;
uint32_t frame_counter = 0;
uint32_t GPABCallback_Counter=0;

volatile uint32_t u32Timer0Cnt=0, u32Timer1Cnt=0, u32Timer2Cnt=0, u32Timer3Cnt=0;
bool beDisplayLoopTime = 0;
void SetDisplayLoopTime(bool display) 
{
  beDisplayLoopTime = display;
}
void Delay(uint32_t delayCnt)
{
	while(delayCnt--) {
		__NOP();
		__NOP();
	}
}
void DelayLoop(unsigned short delay){
	while(--delay);//About 200 psec per run
}
void DelayUsec(unsigned int usec){
	while(usec--)
		DelayLoop(5);
}
void DelayMsec(unsigned short msec){
	int tick_count= getTickCount();
	while((getTickCount()-tick_count)<msec);
}
void setup_system_tick(uint32_t sampleRate)
{
	uint32_t tickPeriod = SystemCoreClock/sampleRate;
	
	SysTick_Config(tickPeriod);
	ChronographStart(ChronMain);
	printf("SystemCoreClock:%d\n",SystemCoreClock);
	printf("Tick Time: %d ms\n",1000/sampleRate);
}
void SysTick_Handler(void)
{
	static int FC_Last;
	int freqCount=0;

	if((tick_counter%1000)==0) {
		freqCount=frame_counter-FC_Last;
		UPDATE_DT = (float)(1.0f/freqCount);
    if(beDisplayLoopTime) {
      printf("FC:%d\n",freqCount);
    }
		FC_Last = frame_counter;
	}
	#if STACK_BARO
		if((tick_counter==3000))
			SetCalibratingB(10);
	#endif
  //if((tick_counter%1000)==0)
  //  printf("update time:%d\n",update_time);
	tick_counter++;
	nvtMillisecondTick();
}

void IncFrameCount(int inc)
{
	frame_counter+=inc;
}

uint32_t GetFrameCount()
{
	return frame_counter;
}

void ChronographSet(char Chron)
{
	Chronograph[Chron].lastTime = Chronograph[Chron].currentTime;
	Chronograph[Chron].currentTime = tick_counter;  
}

void ChronographStart(char Chron)
{
	Chronograph[Chron].currentTime = Chronograph[Chron].lastTime = tick_counter;
}

int32_t ChronographRead(char Chron)
{
	int32_t chron_diff = (tick_counter - Chronograph[Chron].currentTime);
	
	if(Chron==ChronMain)
		UPDATE_DT = (float)chron_diff/1000;
	
	return chron_diff;
}

float getUpdateDT()
{
	return UPDATE_DT;
}

int getTickCount()
{
	return tick_counter;
}

uint32_t micros()
{
	return u32Timer2Cnt;
}

uint32_t millis()
{
	return (u32Timer2Cnt/1000);
}

#ifdef M451
void TMR2_IRQHandler(void)
{
	if(TIMER_GetIntFlag(TIMER2) == 1) {
		/* Clear Timer2 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER2);
		u32Timer2Cnt+=10;
	}
}

void TIMER_Enable(char enable) 
{
	if(enable)
		TIMER_EnableInt(TIMER2);
	else
		TIMER_DisableInt(TIMER2);
}
#else
/*---------------------------------------------------------------------------------------------------------*/
/*  Callback funtion                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/* 1 micro second period */
void Timer2_Callback(uint32_t u32Param)
{
	if (u32Param == 1) {
		printf(" Enter Timer 2 callback #%dth, Current INT ticks is %d\n", ++u32Timer2Cnt, DrvTIMER_GetIntTicks(E_TMR2));
	}else {
		u32Timer2Cnt+=10;
	}
}

/* 100ms period */
void Timer3_Callback(uint32_t u32Param)
{
	if (u32Param == 1) {
		printf(" Enter Timer 3 callback #%dth, Current INT ticks is %d\n", ++u32Timer0Cnt, DrvTIMER_GetIntTicks(E_TMR3));
	}
	else {
		++u32Timer0Cnt;
	}
}

void TIMER_Enable(char enable) 
{
	if(enable)
		DrvTIMER_EnableInt(E_TMR2);  // Enable TIMER0 Intettupt  
	else
		DrvTIMER_DisableInt(E_TMR2);  // Enable TIMER0 Intettupt  
}
#endif

void TIMER_Init()
{
#ifdef M451
	/* Enable peripheral clock */
	CLK_EnableModuleClock(TMR2_MODULE);
	CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);

	TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 100000);
	TIMER_EnableInt(TIMER2);
	NVIC_SetPriority (TMR2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	/* Enable Timer2 NVIC */
	NVIC_EnableIRQ(TMR2_IRQn);
	/* Start Time2 counting */
	TIMER_Start(TIMER2);
#else
	DrvTIMER_Init();
	/* Setup Timer 2 */
	DrvSYS_SelectIPClockSource(E_SYS_TMR2_CLKSRC, 0x07);
	DrvTIMER_Open(E_TMR2, 100000, E_PERIODIC_MODE); 
	DrvTIMER_SetTimerEvent(E_TMR2, 1, (TIMER_CALLBACK)Timer2_Callback, 0);
	NVIC_SetPriority (TMR2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	TIMER_Enable(TRUE);
	DrvTIMER_Start(E_TMR2);
#endif

#ifdef USE_TIMER3
	/* Setup Timer 3 */
	DrvSYS_SelectIPClockSource(E_SYS_TMR3_CLKSRC, 0x07);
	DrvTIMER_Open(E_TMR3, 50, E_PERIODIC_MODE); 
	DrvTIMER_SetTimerEvent(E_TMR3, 1, (TIMER_CALLBACK)Timer3_Callback, 0);
	NVIC_SetPriority (TMR3_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
	DrvTIMER_EnableInt(E_TMR3); 
	DrvTIMER_Start(E_TMR3);
#endif
}
