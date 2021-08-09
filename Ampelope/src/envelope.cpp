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

			// fullRange = value of fully charged capacitor; comparitor value is 4096 where cap is charged enough to trigger decay phase
			const float fullRange = 5000.0f;

			// scales attack pot to allow more range at low end of pot, exponentially longer times at upper end
			// higher values give shorter attack times at lower pot values
			const float attackScale = 2.4f;			// 2.4 = approx setting for long times
			const float maxDurationMult = 1.25f;	// to scale maximum delay time - 1.25 for long times
			//const float maxDurationMult = 0.15f;	// to scale maximum delay time - 0.15 for short times
			const float timeStep = 1.0f / 48000.0f;	// one time unit - corresponding to sample time

			// RC value - attackScale represents R component; maxDurationMult represents capacitor size
			float rc = std::pow((float)attack / 4096.f, attackScale) * maxDurationMult;		// Reduce rc for a steeper curve
			float xPos = -rc * std::log(1.f - (currentLevel / fullRange));		// Invert capacitor equation to calculate current 'time' based on y/voltage value
			float newXPos = xPos + timeStep;
			float newYPos = 1.0f - std::pow(M_E, -newXPos / rc);		// Capacitor charging equation

			currentLevel = newYPos * fullRange;
			if (currentLevel >= 4095.0f) {
				currentLevel = 4095.0f;
				gateState = gateStates::decay;
			}
			break;
		}
		case gateStates::decay: {
			// Capacitor discharge equation: Vc = Vo * e ^ -t/RC

			// scales decay pot to allow more range at low end of pot, exponentially longer times at upper end
			// higher values give shorter attack times at lower pot values
			const float decayScale = 2.4f;			// 2.4 = approx setting for long times
			const float maxDurationMult = 5.0f;		// to scale maximum delay time
			//const float maxDurationMult = 0.3f;	// to scale maximum delay time - 0.3 for short times
			const float timeStep = 1.0f / 48000.0f;	// one time unit - corresponding to sample time

			float yHeight = 4096.0f - sustain;		// Height of decay curve

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			float rc = std::pow((float)decay / 4096.0f, decayScale) * maxDurationMult;
			float xPos = -rc * std::log((currentLevel - sustain) / yHeight);		// Invert capacitor discharge equation to calculate current 'time' based on y/voltage value
			float newXPos = xPos + timeStep;
			float newYPos = std::pow(M_E, -newXPos / rc);		// Capacitor discharging equation

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
			float yPos = currentLevel / 4096.f;		// position in decay curve normalised to 0-1 range
			float xStep = 1.f / std::max(static_cast<float>(release), 1.f);

			float newYPos = std::pow(std::pow(yPos, 0.2f) - xStep, 5.f);

			currentLevel = (newYPos * 4096.f);

		}
		gateState = gateStates::off;
	}
	DAC1->DHR12R1 = static_cast<uint32_t>(currentLevel);
}
