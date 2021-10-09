#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f


#define ADC_BUFFER_LENGTH 9
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];
enum ADC_Controls {
	ADC_Attack_1   = 0,		// PC0
	ADC_Decay_1    = 1,		// PC1
	ADC_Sustain_1  = 2,		// PC2
	ADC_Release_1  = 3,		// PC3

	ADC_Attack_2   = 4,		// PA0
	ADC_Decay_2    = 5,		// PA1
	ADC_Sustain_2  = 6,		// PA3
	ADC_Release_2  = 7,		// PB0

	ADC_LFO_Speed  = 8,		// PB11
};


void SystemClock_Config();
void InitSysTick();
void InitDAC();
void InitIO();
void InitEnvTimer();
void InitADC();
void InitUart();
void InitCordic();
/*
void InitCoverageTimer();
void InitDebounceTimer();
*/
