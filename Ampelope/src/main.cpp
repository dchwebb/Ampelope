#include "initialisation.h"
#include "envelope.h"
#include "usb.h"

volatile uint32_t SysTickVal;
volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

uint16_t x = 0;
Envelope envelope;
USBHandler usb;

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
	InitADC();
	InitUart();

	usb.InitUSB();

	while (1)
	{

		x++;
		if (x == 4096) {
			x = 0;
		}

		DAC->DHR12R2 = 4095 - x;


	}
}

