#include "envelope.h"
#include <cmath>
#include <ctgmath>

// Gives maximum error of 0.1 / 1.24 (ie 8%)
float fastPow(float a, float b)
{
    union {
        float    f;
        uint32_t u;
    } temp;
    temp.f = a;
    temp.u = (uint32_t)(b * (float)(temp.u - 1064866808) + 1064866808);
    return temp.f;
}

// fixme
// Decay to sustain

void Envelope::calcEnvelope() {
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
			const float attackScale = 2.9f;			// higher values give shorter attack times at lower pot values
			float maxDurationMult = (longTimes ? 7.7f : 0.9f) / 1.73;		// 1.73 allows duration to be set in seconds
			//const float maxDurationMult = longTimes ? 4.45f : 0.52;	// to scale maximum delay time
			const float timeStep = 1.0f / 48000.0f;	// one time unit - corresponding to sample time

			// RC value - attackScale represents R component; maxDurationMult represents capacitor size

			//GPIOC->ODR |= GPIO_IDR_ID6;
			rc = std::pow(static_cast<float>(attack) / 4096.f, attackScale) * maxDurationMult;		// Reduce rc for a steeper curve
			//GPIOC->ODR &= ~GPIO_ODR_ODR_6;

			if (rc != 0.0f) {
				float xPos = -rc * std::log(1.f - (currentLevel / fullRange));		// Invert capacitor equation to calculate current 'time' based on y/voltage value
				float newXPos = xPos + timeStep;
				exponent = -newXPos / rc;
				float newYPos = 1.0f - std::exp(exponent);		// Capacitor charging equation
				currentLevel = newYPos * fullRange;
			} else {
				currentLevel = fullRange;
			}

			if (currentLevel >= 4095.0f) {
				currentLevel = 4095.0f;
				gateState = gateStates::decay;
			}
			break;

		}
		case gateStates::decay: {
			// Capacitor discharge equation: Vc = Vo * e ^ -t/RC
			float newYPos = 0.0f;
			// scales decay pot to allow more range at low end of pot, exponentially longer times at upper end
			const float decayScale = 2.4f;			// higher values give shorter attack times at lower pot values
			const float maxDurationMult = longTimes ? 5.0f : 0.3f;		// to scale maximum delay time
			const float timeStep = 1.0f / 48000.0f;	// one time unit - corresponding to sample time

			float yHeight = 4096.0f - sustain;		// Height of decay curve

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			rc = std::pow((float)decay / 4096.0f, decayScale) * maxDurationMult;
			if (rc != 0.0f) {
				float xPos = -rc * std::log((currentLevel - sustain) / yHeight);		// Invert capacitor discharge equation to calculate current 'time' based on y/voltage value
				float newXPos = xPos + timeStep;
				newYPos = std::exp(-newXPos / rc);		// Capacitor discharging equation
			}
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
