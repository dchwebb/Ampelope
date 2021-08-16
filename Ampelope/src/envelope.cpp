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

// Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)
// Capacitor discharge equation: Vc = Vo * e ^ -t/RC



void Envelope::calcEnvelope() {
	// Gate on
	if ((GPIOC->IDR & GPIO_IDR_ID8) == 0) {

		switch (gateState) {
		case gateStates::off:
			//currentLevel = 0;
			gateState = gateStates::attack;
			break;

		case gateStates::attack: {

			attack = std::max((int)ADC_array[ADC_Attack], 200);

			// fullRange = value of fully charged capacitor; comparitor value is 4096 where cap is charged enough to trigger decay phase
			const float fullRange = 5000.0f;

			// scales attack pot to allow more range at low end of pot, exponentially longer times at upper end
			const float attackScale = 2.9f;			// higher values give shorter attack times at lower pot values
			float maxDurationMult = (longTimes ? 7.7f : 0.9f) / 1.73;		// 1.73 allows duration to be set in seconds

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
			decay = ADC_array[ADC_Decay];

			// scales decay pot to allow more range at low end of pot, exponentially longer times at upper end
			const float decayScale = 2.4f;			// higher values give shorter attack times at lower pot values
			float maxDurationMult = (longTimes ? 44.0f : 5.28f) / 4.4;		// to scale maximum delay time

			float yHeight = 4096.0f - sustain;		// Height of decay curve

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			rc = std::pow(static_cast<float>(decay) / 4096.0f, decayScale) * maxDurationMult;
			if (rc != 0.0f) {
				float xPos = -rc * std::log((currentLevel - sustain) / yHeight);		// Invert capacitor discharge equation to calculate current 'time' based on y/voltage
				float newXPos = xPos + timeStep;
				float newYPos = std::exp(-newXPos / rc);		// Capacitor discharging equation
				currentLevel = (newYPos * yHeight) + sustain;
			} else {
				currentLevel = 0.0f;
			}

			if (currentLevel <= sustain + 1.5f) {				// add a little extra to avoid getting stuck in infinitely small decrease
				sustain = ADC_array[ADC_Sustain];
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
			release = ADC_array[ADC_Release];

			const float releaseScale = 2.4f;			// higher values give shorter attack times at lower pot values
			float maxDurationMult = (longTimes ? 44.0f : 5.2f) / 1.3;		// to scale maximum delay time

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			rc = std::pow(static_cast<float>(release) / 4096.0f, releaseScale) * maxDurationMult;
			if (rc != 0.0f) {
				float xPos = -rc * std::log(currentLevel / 4096.0f);
				float newXPos = xPos + timeStep;
				float newYPos = std::exp(-newXPos / rc);
				currentLevel = newYPos * 4096.0f;
			} else {
				currentLevel = 0.0f;
			}

		}
		gateState = gateStates::off;
	}
	DAC1->DHR12R1 = static_cast<uint32_t>(currentLevel);
}
