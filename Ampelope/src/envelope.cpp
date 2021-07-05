#include "envelope.h"

uint16_t Envelope::calcEnvelope() {
	// Gate on
	if ((GPIOC->IDR & GPIO_IDR_ID8) == 0) {

		switch (gateState) {
		case gateStates::off:
			//currentLevel = 0;
			gateState = gateStates::attack;
			break;

		case gateStates::attack:
			attackCount = 4096 / std::max<uint16_t>(attack, 1);
			currentLevel += attackCount;
			if (currentLevel >= 4096) {
				currentLevel = 4095;
				gateState = gateStates::decay;
			}
			break;

		case gateStates::decay:
			decayCount = 4096 / std::max<uint16_t>(decay, 1);
			currentLevel -= decayCount;
			if (currentLevel < sustain) {
				currentLevel = sustain;
				gateState = gateStates::sustain;
			}
			break;
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
