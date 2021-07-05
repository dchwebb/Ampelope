#include "initialisation.h"
#include "envelope.h"

volatile uint32_t SysTickVal;

uint16_t x = 0;
Envelope envelope;


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
	InitIO();
	InitEnvTimer();


	while (1)
	{

		x++;
		if (x == 4096) {
			x = 0;
		}

		DAC->DHR12R2 = 4095 - x;


	}
}

