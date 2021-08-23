#pragma once

#include "initialisation.h"
#include <cmath>

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

	const float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	void calcEnvelope();
	float CordicExp(float x);
	float CordicLn(float x);
};




