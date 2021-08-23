#include "envelope.h"
#include <cmath>
#include <ctgmath>

float Envelope::expArray[Envelope::ExpLookupSize];

void Envelope::CreateExpLookup() {
	for (int i = 0; i < ExpLookupSize; ++i) {
		expArray[i] = std::exp(ExpLookupMin + (ExpLookupInc * static_cast<float>(i)));
	}
}

#define q31_float_to_int(x) ((int)((float)(x) * (float)0x7FFFFFFF))


float CordicExp(float x) {
	// use CORDIC sinh function and generate e^x = sinh(x) + cosh(x) - only values from  -1.118 to +1.118
	CORDIC->CSR = (6 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// Must be 1 for sinh
			CORDIC_CSR_NRES |						// 2 Results as we need both sinh and cosh
			(5 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 5 (gives 5 * 4 = 20 iterations in 5 clock cycles)

	// convert float to q1_31 format scaling x by 1/2 at the same time
	int q31;
	if (x < -1.118f) {
		q31 = (int)((x + 1.0f) * 1073741824.0f);		// as range of x is limited to -1.118 to +1.118 reduce exponent by constant (note that only values from around -1.75 to 0 used in this mechanism)
	} else {
		q31 = (int)(x * 1073741824.0f);
	}

	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 2 at the same time
	float sinh = (float)((int)CORDIC->RDATA) / 1073741824.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
	float cosh = (float)((int)CORDIC->RDATA) / 1073741824.0f;
	float res = sinh + cosh;
	if (x < -1.118f) {
		return res * 0.3678794411714f;					// multiply by e^-1 to correct range offset
	} else {
		return res;
	}
}

float CordicLn(float x) {
	CORDIC->CSR = (8 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cosine, 1: Sine, 2: Phase, 3: Modulus, 4: Arctangent, 5: Hyperbolic cosine, 6: Hyperbolic sine, 7: Arctanh, 8: Natural logarithm, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// 1: 0.107 ≤ x < 1
			(5 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 5 (gives 5 * 4 = 20 iterations in 5 clock cycles)

	int q31 = (int)(x * 1073741824.0f);				// convert float to q1_31 format scaling x by 1/2 at the same time
	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 4 at the same time
	return static_cast<float>((int)CORDIC->RDATA) / 536870912.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
}

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



float Envelope::ExpApprox(float p) {

	//const float inc = (EXP_LOOKUP_MAX - ExpLookupMin) / static_cast<float>(ExpLookupSize);
	float pos = (p - ExpLookupMin) / ExpLookupInc;

	if (pos >= ExpLookupSize - 1) {
		//return 1.0f;
		return expArray[ExpLookupSize - 1];
	}
	if (pos < 0.0) {
		return expArray[0];
	}

	float low = expArray[static_cast<uint16_t>(pos)];
	float high = expArray[static_cast<uint16_t>(pos + 1)];
	float fraction = pos - std::floor(pos);
	return low + fraction * (high - low);
}


// Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)
// Capacitor discharge equation: Vc = Vo * e ^ -t/RC

float minexp = 999999.99f, maxexp = -999999.99;

void Envelope::calcEnvelope() {

	//CORDIC->WDATA = cordic_inc;		// This should be a value between -1 and 1 in q1.31 format, relating to -pi to +pi
	cordic_inc += ADC_array[ADC_Release] * 200;

	// Gate on
	if ((GPIOC->IDR & GPIO_IDR_ID8) == 0) {
		//GPIOC->ODR |= GPIO_IDR_ID6;
		sustain = ADC_array[ADC_Sustain];

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
			float maxDurationMult = (longTimes ? 7.7f : 0.9f) / 1.73f;		// 1.73 allows duration to be set in seconds

			// RC value - attackScale represents R component; maxDurationMult represents capacitor size
			GPIOC->ODR |= GPIO_IDR_ID6;
			rc = std::pow(static_cast<float>(attack) / 4096.f, attackScale) * maxDurationMult;		// Reduce rc for a steeper curve
			GPIOC->ODR &= ~GPIO_IDR_ID6;

			if (rc != 0.0f) {
				/*
				 * Long hand calculations:
				 * 1. Invert capacitor equation to calculate current 'time' based on y/voltage value
				 * float ln = std::log(1.0f - (currentLevel / fullRange));
				 * float xPos = -rc * ln;
				 * float newXPos = xPos + timeStep;		// Add timeStep (based on sample rate) to current X position
				 *
				 * 2. Calculate exponential of time for capacitor charging equation
				 * float exponent = -newXPos / rc;
				 * float newYPos = 1.0f - std::exp(exponent);
				 */
				float newYPos = 1.0f - (1.0f - (currentLevel / fullRange)) * CordicExp(- timeStep / rc);

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
			GPIOC->ODR |= GPIO_IDR_ID6;
			rc = std::pow(static_cast<float>(decay) / 4096.0f, decayScale) * maxDurationMult;
			GPIOC->ODR &= ~GPIO_IDR_ID6;

			if (rc != 0.0f && currentLevel > sustain) {
				/*
				 * Long hand calculations:
				 * 1. Invert capacitor discharge equation to calculate current 'time' based on y/voltage
				 * float xPos = -rc * std::log((currentLevel - sustain) / yHeight);
				 * float newXPos = xPos + timeStep;
				 *
				 * 2. Calculate exponential of time for capacitor discharging equation
				 * float exponent = -newXPos / rc;
				 * float newYPos = std::exp(exponent);		// Capacitor discharging equation
				 */

				float newYPos = ((currentLevel - sustain) / yHeight) * CordicExp(- timeStep / rc);		// Capacitor discharging equation
				currentLevel = (newYPos * yHeight) + sustain;

			} else {
				currentLevel = 0.0f;
			}

			if (currentLevel <= sustain + 1.5f) {				// add a little extra to avoid getting stuck in infinitely small decrease
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
			//release = ADC_array[ADC_Release];

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

	if (lfo) {
		if (CORDIC->CSR & CORDIC_CSR_RRDY) {
			cordic_sin = static_cast<float>(static_cast<int32_t>(CORDIC->RDATA)) / 4294967295.0f + 0.5f;
			DAC1->DHR12R1 = static_cast<uint32_t>((4095.0f - currentLevel * cordic_sin));
		}
	} else {
		DAC1->DHR12R1 = static_cast<uint32_t>(4095.0f - currentLevel);
	}

	GPIOC->ODR &= ~GPIO_ODR_ODR_6;
}
