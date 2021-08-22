#include "initialisation.h"
#include "envelope.h"
#include "usb.h"
#include "uartHandler.h"
#include "SerialHandler.h"
#include <cmath>
// FIXME - add transistor to start VCA in off position

volatile uint32_t SysTickVal;
volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

uint32_t buttonDebounce;
uint16_t x = 0;
Envelope envelope;
USBHandler usb;
SerialHandler serial(usb);

extern "C" {
#include "interrupts.h"
}


//float expArray[EXP_LOOKUP_SIZE];
//void expLookup() {
//
//	const float inc = (EXP_LOOKUP_MAX - EXP_LOOKUP_MIN) / static_cast<float>(EXP_LOOKUP_SIZE);
//	for (int i = 0; i < EXP_LOOKUP_SIZE; ++i) {
//		expArray[i] = std::exp(EXP_LOOKUP_MIN + (inc * static_cast<float>(i)));
//		//expArray[i] = std::exp(EXP_LOOKUP_MIN + (EXP_LOOKUP_INC * static_cast<float>(i)));
//	}
//}

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
	InitCordic();

	usb.InitUSB();
	envelope.CreateExpLookup();

	while (1)
	{

		x++;
		if (x == 4096) {
			x = 0;
		}

		DAC->DHR12R2 = 4095 - x;

		if ((GPIOC->IDR & GPIO_IDR_ID13) != 0 && SysTickVal > buttonDebounce + 1000) {
			buttonDebounce = SysTickVal;
#if (USB_DEBUG)
			usb.OutputDebug();
#endif
			//uartSendString("hello");

		}
		serial.Command();			// Check for incoming CDC commands

	}
}

