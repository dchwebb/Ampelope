#pragma once

#include "stm32g4xx.h"
//#include <algorithm>

extern volatile uint32_t SysTickVal;

void SystemClock_Config(void);
/*
void InitIO(void);
void InitSysTick();
void InitSampleAcquisition();
void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitMidiUART();
void InitDAC();
*/
