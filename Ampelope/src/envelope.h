#pragma once

#include "initialisation.h"
#include <cmath>

struct Envelope {
public:
	uint16_t attack = 800;
	uint16_t decay = 0;
	float sustain = 4095.0f;
	uint16_t release = 300;
	float currentLevel = 0.0f;
	float exponent = 0.0f;	// breakout variables for debug
	float powErr = 0.0f;
	float rc = 0.0f;
	int32_t cordic_inc = 0;
	float cordic_sin = 0;
	bool longTimes = true;
	bool lfo = false;

	const float timeStep = 1.0f / SAMPLERATE;	// one time unit - corresponding to sample time

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;


	static const uint16_t ExpLookupSize = 6000;						// Size of lookup table
	static constexpr const float ExpLookupMin = -1.75f;
	static constexpr const float ExpLookupMax = 0.0f;
	static constexpr const float ExpLookupInc = (ExpLookupMax - ExpLookupMin) / static_cast<float>(ExpLookupSize);
	static float expArray[ExpLookupSize];

	void calcEnvelope();

	void CreateExpLookup();
	float ExpApprox(float p);
};




