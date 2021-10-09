#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f
#define ADC_BUFFER_LENGTH 9

struct ADSR {
	uint16_t attack;
	uint16_t sustain;
	uint16_t decay;
	uint16_t release;
};

struct ADCValues {
	ADSR EnvA;
	ADSR EnvB;
	uint16_t Tremolo;
};

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

extern volatile ADCValues ADC_array;

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
