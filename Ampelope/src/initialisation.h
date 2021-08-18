#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f

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
