#pragma once

#include "initialisation.h"

struct Envelope {
public:
	uint16_t attack = 800;
	uint16_t decay = 0;
	float sustain = 4095.0f;
	uint16_t release = 2400;
	float currentLevel = 0.0f;
	float exponent = 0.0f;	// breakout variables for debug
	float powErr = 0.0f;
	float rc = 0.0f;

	bool longTimes = true;

	const float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	void calcEnvelope();
};
