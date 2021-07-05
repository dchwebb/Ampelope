#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms

void SystemClock_Config();
void InitSysTick();
void InitDAC();
void InitIO();
void InitEnvTimer();

/*


void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitMidiUART();
*/
