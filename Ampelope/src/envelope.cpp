#include "envelope.h"
#include <cmath>


uint32_t Envelopes::tremSpeed = 0;

void Envelopes::calcEnvelopes()
{
	DEBUG_ON

	// Check if clock received
	if ((GPIOA->IDR & GPIO_IDR_IDR_9) == 0) {		// Clock signal high
		if (!clockHigh) {
			clockInterval = clockCounter - lastClock;
			lastClock = clockCounter;
			clockHigh = true;
		}
	} else {
		clockHigh = false;
	}
	clockValid = (clockCounter - lastClock < (SAMPLERATE * 2));					// Valid clock interval is within a second
	++clockCounter;

	if (clockValid) {
		if (ADC_array.Tremolo > tremHysteresis + 20 || ADC_array.Tremolo < tremHysteresis - 20) {
			tremHysteresis = ADC_array.Tremolo;

			if (tremHysteresis < 682)				tremMult = 8.0f;
			else if (tremHysteresis < 1365) 		tremMult = 4.0f;
			else if (tremHysteresis < 2048) 		tremMult = 2.0f;
			else if (tremHysteresis < 2731) 		tremMult = 1.0f;
			else if (tremHysteresis < 3413) 		tremMult = 0.5f;
			else 									tremMult = 0.25f;
		}
		tremSpeed = static_cast<uint32_t>(tremMult * static_cast<float>(clockInterval));

	} else {
		tremSpeed = 0;
	}


	for (Envelope& env : envelope) {
		env.calcEnvelope();
	}

	DEBUG_OFF
}

float Envelope::CordicExp(float x)
{
	// use CORDIC sinh function and generate e^x = sinh(x) + cosh(x)
	CORDIC->CSR = (6 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cos, 1: Sin, 2: Phase, 3: Modulus, 4: Arctan, 5: cosh, 6: sinh, 7: Arctanh, 8: ln, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// Must be 1 for sinh
			CORDIC_CSR_NRES |						// 2 Results as we need both sinh and cosh
			(6 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 6 (gives 6 * 4 = 24 iterations in 6 clock cycles)

	// convert float to q1_31 format scaling x by 1/2 at the same time
	int q31;
	if (x < -1.118f) {
		q31 = (int)((x + 1.0f) * 1073741824.0f);	// as range of x is limited to -1.118 to +1.118 reduce exponent by e^-1 (note that only values from around -1.75 to 0 used in this mechanism)
	} else {
		q31 = (int)(x * 1073741824.0f);
	}

	//volatile float etest = std::exp(x);

	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 2 at the same time
	float sinh = (float)((int)CORDIC->RDATA) / 1073741824.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
	float cosh = (float)((int)CORDIC->RDATA) / 1073741824.0f;
	float res = sinh + cosh;
	if (x < -1.118f) {
		return res * 0.3678794411714f;				// multiply by e^-1 to correct range offset
	} else {
		return res;
	}
}


float Envelope::CordicLn(float x)
{
	CORDIC->CSR = (8 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cosine, 1: Sine, 2: Phase, 3: Modulus, 4: Arctangent, 5: Hyperbolic cosine, 6: Hyperbolic sine, 7: Arctanh, 8: Natural logarithm, 9: Square Root
			CORDIC_CSR_SCALE_0 |					// 1: 0.107 ≤ x < 1
			(5 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 5 (gives 5 * 4 = 20 iterations in 5 clock cycles)

	int q31 = (int)(x * 1073741824.0f);				// convert float to q1_31 format scaling x by 1/2 at the same time
	CORDIC->WDATA = q31;

	// convert values back to floats scaling by * 4 at the same time
	return static_cast<float>((int)CORDIC->RDATA) / 536870912.0f;	// command will block until RDATA is ready - no need to poll RRDY flag
}



void Envelope::calcEnvelope()
{
	// Gate on
	if ((gatePort->IDR & (1 << gatePin)) == 0) {

		longADSR = (shortPort->IDR & (1 << shortPin)) != 0;
		tremolo = (tremPort->IDR & (1 << tremPin)) == 0;

		sustain = adsr.sustain;

		switch (gateState) {
		case gateStates::off:
			gateState = gateStates::attack;
			break;

		case gateStates::release:
			gateState = gateStates::attack;
			break;

		case gateStates::attack: {

			attack = std::round(((attack * 31.0f) + static_cast<float>(adsr.attack)) / 32.0f);

			// fullRange = value of fully charged capacitor; comparitor value is 4096 where cap is charged enough to trigger decay phase
			const float fullRange = 5000.0f;

			// scales attack pot to allow more range at low end of pot, exponentially longer times at upper end
			float maxDurationMult = (longADSR ? 7.7f : 0.9f) / 1.73f;		// 1.73 allows duration to be set in seconds

			// RC value - attackScale represents R component; maxDurationMult represents capacitor size (Reduce rc for a steeper curve)
			float rc = std::pow(static_cast<float>(attack) / 4096.f, 3.0f) * maxDurationMult;		// Using a^3 for fast approximation for measured charging rate (^2.9)

			if (rc != 0.0f) {
				/*
				 * Long hand calculations:
				 * Capacitor charging equation: Vc = Vs(1 - e ^ -t/RC)
				 * 1. Invert capacitor equation to calculate current 'time' based on y/voltage value
				 * float ln = std::log(1.0f - (currentLevel / fullRange));
				 * float xPos = -rc * ln;
				 * float newXPos = xPos + timeStep;		// Add timeStep (based on sample rate) to current X position
				 *
				 * 2. Calculate exponential of time for capacitor charging equation
				 * float exponent = -newXPos / rc;
				 * float newYPos = 1.0f - std::exp(exponent);
				 * currentLevel = newYPos * fullRange;
				 */

				currentLevel = fullRange - (fullRange - currentLevel) * CordicExp(-timeStep / rc);

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
			decay = adsr.decay;

			// scales decay pot to allow more range at low end of pot, exponentially longer times at upper end
			float maxDurationMult = (longADSR ? 44.0f : 5.28f) / 4.4;		// to scale maximum delay time

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			float rc = std::pow((float)decay / 4096.0f, 2.0f) * maxDurationMult;		// Use x^2 as approximation for measured x^2.4

			if (rc != 0.0f && currentLevel > sustain) {
				/*
				 * Long hand calculations:
				 * Capacitor discharge equation: Vc = Vo * e ^ -t/RC
				 * 1. Invert capacitor discharge equation to calculate current 'time' based on y/voltage
				 * float yHeight = 4096.0f - sustain;		// Height of decay curve
				 * float xPos = -rc * std::log((currentLevel - sustain) / yHeight);
				 * float newXPos = xPos + timeStep;
				 *
				 * 2. Calculate exponential of time for capacitor discharging equation
				 * float exponent = -newXPos / rc;
				 * float newYPos = std::exp(exponent);		// Capacitor discharging equation
				 * currentLevel = (newYPos * yHeight) + sustain;
				 */

				currentLevel = sustain + (currentLevel - sustain) * CordicExp(-timeStep / rc);

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
		}

	} else {
		if (currentLevel > 0.0f) {
			gateState = gateStates::release;
			release = adsr.release;

			float maxDurationMult = (longADSR ? 14.585f : 1.15f);		// to scale maximum delay time

			// RC value - decayScale represents R component; maxDurationMult represents capacitor size
			float rc = std::pow(static_cast<float>(release) / 4096.0f, 2.0f) * maxDurationMult;
			if (rc != 0.0f && currentLevel > 1.0f) {
				/*
				 * Long hand calculations:
				 * float xPos = -rc * std::log(currentLevel / 4096.0f);
				 * float newXPos = xPos + timeStep;
				 * float newYPos = std::exp(-newXPos / rc);
				 * currentLevel = newYPos * 4096.0f;
				 */

				currentLevel = currentLevel * CordicExp(-timeStep / rc);
			} else {
				currentLevel = 0.0f;
			}

		} else {
			gateState = gateStates::off;
		}
	}

	if (tremolo) {
		CORDIC->CSR = (0 << CORDIC_CSR_FUNC_Pos) | 		// 0: Cosine, 1: Sine, 2: Phase, 3: Modulus, 4: Arctangent, 5: Hyperbolic cosine, 6: Hyperbolic sine, 7: Arctanh, 8: Natural logarithm, 9: Square Root
				(5 << CORDIC_CSR_PRECISION_Pos);		// Set precision to 5 (gives 5 * 4 = 20 iterations in 5 clock cycles)

		CORDIC->WDATA = tremCosinePos;		// This should be a value between -1 and 1 in q1.31 format, relating to -pi to +pi

		if (Envelopes::tremSpeed != 0) {
			tremCosinePos += 4294967295 / Envelopes::tremSpeed;
		} else {
			tremCosinePos += (ADC_array.Tremolo + 50) * 400;
		}

		tremCosineVal = static_cast<float>(static_cast<int32_t>(CORDIC->RDATA)) / 4294967295.0f + 0.5f;
		*outputDAC = static_cast<uint32_t>(currentLevel * tremCosineVal);
	} else {
		tremCosinePos = 0;								// Reset cosine position to 0 so that it starts at full level when turned on
		*outputDAC = static_cast<uint32_t>(currentLevel);
	}



}
