#pragma once

#include "initialisation.h"

struct Envelope {
public:
	uint16_t attack = 800;
	uint16_t decay = 600;
	uint16_t sustain = 2048;
	uint16_t release = 2400;
	float currentLevel = 0.0f;
	float exponent = 0.0f;	// breakout variables for debug
	float powErr = 0.0f;
	float rc = 0.0f;

	bool longTimes = true;

	uint16_t attackCount;
	uint16_t decayCount;
	uint16_t releaseCount;

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	void calcEnvelope();
};
