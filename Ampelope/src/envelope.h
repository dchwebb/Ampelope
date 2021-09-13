#pragma once

#include "initialisation.h"


struct Envelope {
public:
	float attack = 800.0f;
	uint16_t decay = 0;
	float sustain = 4095.0f;
	uint16_t release = 300;
	float currentLevel = 0.0f;

	int32_t cordic_inc = 0;
	float cordic_sin = 0;
	bool longTimes = true;
	bool lfo = false;

	uint32_t clockCounter;					// Counter used to calculate clock times in sample time
	uint32_t lastClock;						// Time last clock signal received in sample time
	uint32_t clockInterval;					// Clock interval in sample time
	bool clockValid = false;
	bool clockHigh = false;


	const float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	void calcEnvelope();
	float CordicExp(float x);
	float CordicLn(float x);
};




