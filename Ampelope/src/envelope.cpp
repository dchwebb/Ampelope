#include "envelope.h"
#include <cmath>

uint16_t Envelope::calcEnvelope() {
	// Gate on
	if ((GPIOC->IDR & GPIO_IDR_ID8) == 0) {

		switch (gateState) {
		case gateStates::off:
			//currentLevel = 0;
			gateState = gateStates::attack;
			break;

		case gateStates::attack: {
			// Normalise to a linear scale between 0 and 1
			float xPos = std::pow(static_cast<float>(currentLevel) / 4096.0f, 2.f);
			float xStep = 1.f / std::max(static_cast<float>(attack), 1.f);
			float newXPos = xPos + xStep;
			float newYPos = std::sqrt(newXPos);
			currentLevel = newYPos * 4096.f;
			if (currentLevel >= 4096) {
				currentLevel = 4095;
				gateState = gateStates::decay;
			}
			break;
		}
		case gateStates::decay: {
			float yHeight = 4096.f - sustain;			// Height of decay curve
			float yPos = (static_cast<float>(currentLevel) - sustain) / yHeight;		// position in decay curve normalised to 0-1 range
			//float xPos = 1.f - std::pow(yPos, 0.2f);			// Calculate normalised x position based on y position
			float xStep = 1.f / std::max(static_cast<float>(decay), 1.f);
			//float newXPos = xPos + xStep;
			//float newYPos = std::pow(1.f - newXPos, 5.f);

			float newYPos = std::pow(std::pow(yPos, 0.2f) - xStep, 5.f);

			//decayCount = 4096 / std::max<uint16_t>(decay, 1);
			//currentLevel -= decayCount;
			currentLevel = (newYPos * yHeight) + sustain;
			if (currentLevel < sustain) {
				currentLevel = sustain;
				gateState = gateStates::sustain;
			}
			break;
		}
		case gateStates::sustain:
			currentLevel = sustain;
			break;
		case gateStates::release:
			break;
		}

	} else {
		if (currentLevel > 0) {
			releaseCount = 4096 / std::max<uint16_t>(release, 1);
			currentLevel = std::max<int32_t>(currentLevel - releaseCount, 0);
		}
		gateState = gateStates::off;
	}
	DAC->DHR12R1 = currentLevel;
}
