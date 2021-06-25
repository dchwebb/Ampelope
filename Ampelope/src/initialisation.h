#pragma once

#include "stm32g4xx.h"
//#include <algorithm>

extern volatile uint32_t SysTickVal;

void SystemClock_Config(void);
void InitSysTick();
void InitDAC();

/*
void InitIO(void);
void InitSampleAcquisition();
void InitCoverageTimer();
void InitDebounceTimer();
void InitEncoders();
void InitMidiUART();
*/
