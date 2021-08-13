#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f

#define ADC_BUFFER_LENGTH 2
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];
enum ADC_Controls {
	ADC_Attack   = 0,	// PB0 ADC12_IN8   Pin 27
	ADC_Decay    = 5,	// PC0 ADC123_IN10 Pin 8
};


void SystemClock_Config();
void InitSysTick();
void InitDAC();
void InitIO();
void InitEnvTimer();
void InitADC();
void InitUart();

/*
void InitCoverageTimer();
void InitDebounceTimer();
*/
