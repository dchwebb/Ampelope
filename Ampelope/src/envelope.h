#pragma once

#include "initialisation.h"
#include "SerialHandler.h"

struct ADSR {
	uint16_t attack;
	uint16_t decay;
	uint16_t sustain;
	uint16_t release;
};

struct ADCValues {
	ADSR EnvA;
	ADSR EnvB;
	uint16_t Tremolo;
};

extern volatile ADCValues ADC_array;

struct Envelope {
	friend class SerialHandler;					// Allow the serial handler access to private data for debug printing
public:
	Envelope(volatile ADSR& adsr, volatile uint32_t* outputDAC, GPIO_TypeDef* gatePort, uint8_t gatePin, GPIO_TypeDef* shortPort, uint8_t shortPin, GPIO_TypeDef* tremPort, uint8_t tremPin)
	 : adsr(adsr), outputDAC{outputDAC}, gatePort{gatePort}, gatePin{gatePin}, shortPort{shortPort}, shortPin{shortPin}, tremPort{tremPort}, tremPin{tremPin} {}

	void calcEnvelope();						// Sets the DAC level for envelope
	float CordicExp(float x);					// Uses the CORDIC co-processor to calculate an exponential e^x
	float CordicLn(float x);					// Uses the CORDIC co-processor to calculate a natural log ln(x)

private:
	const float     timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	bool            longADSR = true;			// True if using long ADSR settings (also tremolo)
	float           attack = 800.0f;			// Store the ADSR values based on the pot values (mainly for debugging)
	uint16_t        decay = 0;
	float           sustain = 4095.0f;
	uint16_t        release = 300;
	float           currentLevel = 0.0f;		// The current level of the envelope (held as a float for accuracy of calulculation)

	int32_t         tremCosinePos = 0;			// Position of cordic cosine wave in q1.31 format
	float           tremCosineVal = 0.0f;		// Value of cosine scaled from 0.0 to 1.0
	bool            tremolo = false;			// True if the tremolo is activated

	enum class      gateStates {off, attack, decay, sustain, release};
	gateStates      gateState = gateStates::off;

	// Hardware settings for each envelope (which ADSR bank, GPIO gate input and switch positions)
	volatile ADSR&     adsr;
	volatile uint32_t* outputDAC;
	GPIO_TypeDef*      gatePort;
	uint8_t            gatePin;
	GPIO_TypeDef*      shortPort;
	uint8_t            shortPin;
	GPIO_TypeDef*      tremPort;
	uint8_t            tremPin;
};


struct Envelopes {
	friend class SerialHandler;				// Allow the serial handler access to private data for debug printing

public:
	void calcEnvelopes();					// Calls calculation on all contained envelopes

	static uint32_t tremSpeed;				// Tremolo interval in sample time after scaling (public for visibility in the envelope class)

private:
	bool     clockValid;					// True if a clock pulse has been received within a second
	uint32_t clockInterval;					// Clock interval in sample time
	uint32_t clockCounter;					// Counter used to calculate clock times in sample time
	uint32_t lastClock;						// Time last clock signal received in sample time
	bool     clockHigh;						// Record clock high state to detect clock transitions
	uint32_t tremHysteresis;				// Hysteresis to prevent jumping between multipliers
	float    tremMult;						// Multiplier set by tremolo potentiometer when clock applied

	Envelope envelope[4] = {
			{ADC_array.EnvA, &(DAC1->DHR12R1), GPIOB, 13, GPIOB,  5, GPIOB,  6},		// PA4 Env1
			{ADC_array.EnvA, &(DAC1->DHR12R2), GPIOB, 14, GPIOB,  3, GPIOB,  4},		// PA5 Env2
			{ADC_array.EnvB, &(DAC3->DHR12R2), GPIOB, 15, GPIOC, 10, GPIOC, 12},		// PB1 Env3
			{ADC_array.EnvB, &(DAC3->DHR12R1), GPIOC,  6, GPIOB, 12, GPIOA, 15} 		// PA2 Env4
	};
};

