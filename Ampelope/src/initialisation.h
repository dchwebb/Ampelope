#pragma once

#include "stm32g4xx.h"
#include <algorithm>

extern volatile uint32_t SysTickVal;

#define SYSTICK 1000						// 1ms
#define SAMPLERATE 48000.0f
#define ADC_BUFFER_LENGTH 9

#define DEBUG_ON  GPIOB->ODR |= GPIO_ODR_OD9;
#define DEBUG_OFF GPIOB->ODR &= ~GPIO_ODR_OD9;

void SystemClock_Config();
void InitSysTick();
void InitDAC();
void InitIO();
void InitEnvTimer();
void InitADC(volatile uint16_t* ADC_array);
void InitUart();
void InitCordic();
/*
void InitCoverageTimer();
void InitDebounceTimer();
*/
