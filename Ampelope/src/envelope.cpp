#include "envelope.h"
#include <cmath>

uint16_t Envelope::calcEnvelope() {
	// Gate on
	if ((GPIOC->IDR & GPIO_IDR_ID8) == 0) {
		attack = ADC_array[0];
		decay = ADC_array[1];

		switch (gateState) {
		case gateStates::off:
			//currentLevel = 0;
			gateState = gateStates::attack;
			break;

		case gateStates::attack: {
			// Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)
//			const float rc = .9f;		// Reducing rc gives a steeper curve
//			float xPos = -rc * std::log(1.f - (currentLevel / 6096.0f));		// Use a higher multiple than 4096 so that end of attack stage is not when cap is fully charged
//			float scaledMult = std::pow((float)attack / 4096.f, 6.f) * 48000.f;		// Potentiometer values scale to exponentially (^6 for CEM, ^2.8 for SysX Short) longer attack times
//			float xStep = 1.f / std::max(scaledMult, 1.f);
//			float newXPos = xPos + xStep;
//			float newYPos = 1.0f - std::pow(M_E, -newXPos / rc);

			// Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)

			// fullRange = value representing fully charged capacitor; comparitor value is 4096 where cap is charged enough to trigger decay phase
			const float fullRange = 5000.f;

			// scales attack pot to allow more range at low end of pot, exponentially longer times at upper end
			// higher values give shorter attack times at lower pot values
			const float attackScale = 2.4f;			// 2.4 = approx setting for long times
			const float maxDurationMult = 1.25f;	// to scale maximum delay time - 1.25 for long times
			const float timeStep = 1.f / 48000.f;	// one time unit - corresponding to sample time

			// RC value - scale attack value to represent R component; maxDurationMult represents capacitor size
			float rc = std::pow((float)attack / 4096.f, attackScale) * maxDurationMult;		// Reduce rc for a steeper curve
			float xPos = -rc * std::log(1.f - (currentLevel / fullRange));		// Invert capacitor equation to calculate current 'time' based on y/voltage value
			float newXPos = xPos + timeStep;
			float newYPos = 1.0f - std::pow(M_E, -newXPos / rc);		// Capacitor charging equation

			currentLevel = newYPos * fullRange;
			if (currentLevel >= 4095.f) {
				currentLevel = 4095.f;
				gateState = gateStates::decay;
			}
			break;
		}
		case gateStates::decay: {
			float yHeight = 4096.f - sustain;			// Height of decay curve
			float yPos = (currentLevel - sustain) / yHeight;		// position in decay curve normalised to 0-1 range
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
//			releaseCount = 4096 / std::max<uint16_t>(release, 1);
//			currentLevel = std::max<int32_t>(currentLevel - releaseCount, 0);

			float yPos = currentLevel / 4096.f;		// position in decay curve normalised to 0-1 range
			float xStep = 1.f / std::max(static_cast<float>(release), 1.f);

			float newYPos = std::pow(std::pow(yPos, 0.2f) - xStep, 5.f);

			currentLevel = (newYPos * 4096.f);

		}
		gateState = gateStates::off;
	}
	DAC1->DHR12R1 = static_cast<uint32_t>(currentLevel);
}
