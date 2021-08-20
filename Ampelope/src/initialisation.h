#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f
#define EXP_LOOKUP_SIZE 4096						// Size of lookup table
#define EXP_LOOKUP_MIN -1.75f
#define EXP_LOOKUP_MAX 0.0f
#define EXP_LOOKUP_INC (EXP_LOOKUP_MAX - EXP_LOOKUP_MIN) / static_cast<float>(EXP_LOOKUP_SIZE)

#define ADC_BUFFER_LENGTH 4
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];
enum ADC_Controls {
	ADC_Attack   = 0,	// PC0 ADC12_IN6
	ADC_Decay    = 1,	// PC1 ADC12_IN7
	ADC_Sustain  = 2,	// PC2 ADC12_IN8
	ADC_Release  = 3,	// PC3 ADC12_IN9

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
