#pragma once

#include "initialisation.h"

struct Envelope {
public:
	uint16_t attack = 100;
	uint16_t decay = 30;
	uint16_t sustain = 2000;
	uint16_t release = 40;
	int32_t currentLevel = 0;

	uint16_t attackCount;
	uint16_t decayCount;
	uint16_t releaseCount;

	enum class gateStates {off, attack, decay, sustain, release};
	gateStates gateState = gateStates::off;

	uint16_t calcEnvelope();
};
