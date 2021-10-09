#pragma once

#include "initialisation.h"


struct Envelope {
public:
	Envelope(volatile ADSR& adsr, volatile uint32_t* outputDAC, GPIO_TypeDef* gatePort, uint8_t gatePin, GPIO_TypeDef* shortPort, uint8_t shortPin, GPIO_TypeDef* tremPort, uint8_t tremPin)
	 : adsr(adsr), outputDAC{outputDAC}, gatePort{gatePort}, gatePin{gatePin}, shortPort{shortPort}, shortPin{shortPin}, tremPort{tremPort}, tremPin{tremPin} {}

	void calcEnvelope();
	float CordicExp(float x);
	float CordicLn(float x);

	float attack = 800.0f;
	uint16_t decay = 0;
	float sustain = 4095.0f;
	uint16_t release = 300;
	float currentLevel = 0.0f;

	int32_t cordic_inc = 0;
	float cordic_sin = 0;
	bool longTimes = true;
	bool tremolo = false;

	uint32_t clockCounter;					// Counter used to calculate clock times in sample time
	uint32_t lastClock;						// Time last clock signal received in sample time
	uint32_t clockInterval;					// Clock interval in sample time
	bool clockValid = false;
	bool clockHigh = false;


	const float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

private:
	volatile ADSR&     adsr;
	volatile uint32_t* outputDAC;
	GPIO_TypeDef*      gatePort;
	uint8_t            gatePin;
	GPIO_TypeDef*      shortPort;
	uint8_t            shortPin = 5;
	GPIO_TypeDef*      tremPort;
	uint8_t            tremPin;
};


struct Envelopes {
	void calcEnvelopes();

	Envelope envelope[4] = {
			{ADC_array.EnvA, &(DAC1->DHR12R1), GPIOB, 13, GPIOB,  5, GPIOB,  6},		// PA4 Env1
			{ADC_array.EnvA, &(DAC1->DHR12R2), GPIOB, 14, GPIOB,  3, GPIOB,  4},		// PA5 Env2
			{ADC_array.EnvB, &(DAC3->DHR12R2), GPIOB, 15, GPIOC, 10, GPIOC, 12},		// PA2 Env4
			{ADC_array.EnvB, &(DAC3->DHR12R1), GPIOC,  6, GPIOB, 12, GPIOA, 15} 		// PB1 Env3
	};
};

