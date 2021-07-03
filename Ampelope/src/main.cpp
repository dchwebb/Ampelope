#include "initialisation.h"

volatile uint32_t SysTickVal;

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitSysTick();
	InitDAC();

	uint16_t x = 0;
	uint16_t attack, decay, sustain, release;
	uint16_t attackCount;
	uint16_t decayCount;
	uint16_t releaseCount;

	while (1)
	{
		x++;
		if (x == 4096) {
			x = 0;
		}
		DAC->DHR12R1 = x;
		DAC->DHR12R2 = 4095 - x;


	}
}

