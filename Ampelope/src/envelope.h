#pragma once

#include "initialisation.h"

struct Envelope {
public:
	uint16_t attack = 800;
	uint16_t decay = 600;
	uint16_t sustain = 2000;
	uint16_t release = 2400;
	float currentLevel = 0;

	uint16_t attackCount;
	uint16_t decayCount;
	uint16_t releaseCount;

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	uint16_t calcEnvelope();
};
