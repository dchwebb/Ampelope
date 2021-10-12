#include "initialisation.h"
#include "envelope.h"
#include "usb.h"
#include "uartHandler.h"
#include "SerialHandler.h"
#include <cmath>
// FIXME - add transistor to start VCA in off position

volatile uint32_t SysTickVal;
volatile ADCValues ADC_array;

uint32_t buttonDebounce;
Envelopes envelopes;
USBHandler usb;
SerialHandler serial(usb);

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
	InitADC(reinterpret_cast<volatile uint16_t*>(&ADC_array));
	InitUart();
	InitCordic();

	usb.InitUSB();

	while (1) {
		if ((GPIOC->IDR & GPIO_IDR_ID13) != 0 && SysTickVal > buttonDebounce + 1000) {
			buttonDebounce = SysTickVal;
#if (USB_DEBUG)
			usb.OutputDebug();
#endif
		}
		serial.Command();			// Check for incoming CDC commands
	}
}

