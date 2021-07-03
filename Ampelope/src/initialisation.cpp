#include "initialisation.h"

#define PLL_M 0b101		// 0101: PLLM = 6
#define PLL_N 85
#define PLL_R 0			//  00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8
#define PLL_P 2


void SystemClock_Config(void) {
	// See page 236 for clock configuration
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		// SYSCFG + COMP + VREFBUF + OPAMP clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;		// Enable Power Control clock
	PWR->CR5 &= ~PWR_CR5_R1MODE;				// Select the Range 1 boost mode

	RCC->CR |= RCC_CR_HSEON;					// HSE ON
	while ((RCC->CR & RCC_CR_HSERDY) == 0);		// Wait till HSE is ready

	// Configure PLL
	RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos) | (PLL_N << RCC_PLLCFGR_PLLN_Pos) | (PLL_P << RCC_PLLCFGR_PLLPDIV_Pos) | (PLL_R << RCC_PLLCFGR_PLLR_Pos) | (RCC_PLLCFGR_PLLSRC_HSE);
	RCC->CR |= RCC_CR_PLLON;					// Enable the main PLL
	RCC->PLLCFGR = RCC_PLLCFGR_PLLREN;			// Enable PLL R (drives AHB clock)
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait till the main PLL is ready

	// Configure Flash prefetch and wait state. NB STM32G431 is a category 2 device (128KB flash in 1 bank)
	FLASH->ACR |= FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN;
	FLASH->ACR &= ~FLASH_ACR_LATENCY_1WS;

	// The system clock must be divided by 2 using the AHB prescaler before switching to a higher system frequency.
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2;			// HCLK = SYSCLK / 2
	RCC->CFGR |= RCC_CFGR_SW_PLL;				// Select the main PLL as system clock source

	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);

	// Reset the AHB clock (previously divided by 2) and set APB clocks
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;			// PCLK1 = HCLK / 1 (APB1)
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;			// PCLK2 = HCLK / 1 (APB2)
}

void InitSysTick()
{
	// Register macros found in core_cm4.h
	SysTick->CTRL = 0;								// Disable SysTick
	SysTick->LOAD = 0xFFFF - 1;						// Set reload register to maximum 2^24 - each tick is around 400us

	// Set priority of Systick interrupt to least urgency (ie largest priority value)
	NVIC_SetPriority (SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	SysTick->VAL = 0;								// Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;	// Select processor clock: 1 = processor clock; 0 = external clock
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;		// Enable SysTick interrupt
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;		// Enable SysTick
}

void InitDAC()
{
	// p789 - opamp - use follower mode? p792
	//	OPAMP1_VINP		DAC3_CH1	PA1 (VINP0)  / PA3 (VINP1)  / PA7 (VINP2)
	//	OPAMP3_VINP		DAC3_CH2	PB0 (VINP0)  / PB13 (VINP1) / PA1 (VINP2)
	//	x OPAMP4_VINP		DAC4_CH1	PB13 (VINP0) / [PB11 (VINP2)]
	//	x OPAMP5_VINP		DAC4_CH2	PB14 (VINP0) / [PC3 (VINP2)]
	//	x OPAMP6_VINP		DAC3_CH1	PB13 (VINP2) / [PB12 (VINP0)]

	// Once the DAC channelx is enabled, the corresponding GPIO pin (PA4 DAC1_OUT1 or PA5 DAC1_OUT2) is automatically connected to the analog converter output (DAC_OUTx).
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;			// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;				// Enable DAC Clock

	DAC->MCR &= ~DAC_MCR_MODE1_Msk;					// Set to normal mode: DAC channel1 is connected to external pin with Buffer enabled
	DAC->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)

	DAC->MCR &= ~DAC_MCR_MODE2_Msk;					// Set to normal mode: DAC channel1 is connected to external pin with Buffer enabled
	DAC->CR |= DAC_CR_EN2;							// Enable DAC using PA4 (DAC_OUT1)

	// output triggered with DAC->DHR12R1 = x;
}

/*
void InitIO()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// reset and clock control - advanced high performance bus - GPIO port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;			// reset and clock control - advanced high performance bus - GPIO port C

	// configure PC13 blue button
	//GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved

	//GPIOA->MODER |= GPIO_MODER_MODER7_0;			// Set to output

}



//	Setup Timer 3 on an interrupt to trigger sample acquisition
void InitSampleAcquisition() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;				// Enable Timer 3
	TIM3->PSC = 50;									// Set prescaler
	TIM3->ARR = 140; 								// Set auto reload register

	TIM3->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 0);					// Lower is higher priority

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->EGR |= TIM_EGR_UG;						//  Re-initializes counter and generates update of registers
}

//	Setup Timer 9 to count clock cycles for coverage profiling
void InitCoverageTimer() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;				// Enable Timer
	TIM9->PSC = 100;
	TIM9->ARR = 65535;

	TIM9->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2);		// Lower is higher priority

}

//	Setup Timer 5 to count time between bounces
void InitDebounceTimer() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;				// Enable Timer
	TIM5->PSC = 10000;
	TIM5->ARR = 65535;
}


void InitEncoders() {
	// L Encoder: button on PA10, up/down on PB6 and PB7; R Encoder: Button on PB13, up/down on PC6 and PC7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// reset and clock control - advanced high performance bus - GPIO port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;			// reset and clock control - advanced high performance bus - GPIO port C
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;			// Enable system configuration clock: used to manage external interrupt line connection to GPIOs

	// configure PA10 button to fire on an interrupt
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;	// Select Pin PA10 which uses External interrupt 2
	EXTI->RTSR |= EXTI_RTSR_TR10;					// Enable rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR10;					// Enable falling edge trigger
	EXTI->IMR |= EXTI_IMR_MR10;						// Activate interrupt using mask register

	// configure PB13 button to fire on an interrupt
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PB;	// Select Pin PB4 which uses External interrupt 2
	EXTI->RTSR |= EXTI_RTSR_TR13;					// Enable rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR13;					// Enable falling edge trigger
	EXTI->IMR |= EXTI_IMR_MR13;						// Activate interrupt using mask register

	// L Encoder using timer functionality - PB6 and PB7
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOB->MODER |= GPIO_MODER_MODER6_1;			// Set alternate function
	GPIOB->AFR[0] |= 2 << 24;						// Alternate function 2 is TIM4_CH1

	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOB->MODER |= GPIO_MODER_MODER7_1;			// Set alternate function
	GPIOB->AFR[0] |= 2 << 28;						// Alternate function 2 is TIM4_CH2

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;				// Enable Timer 4
	TIM4->PSC = 0;									// Set prescaler
	TIM4->ARR = 0xFFFF; 							// Set auto reload register to max
	TIM4->SMCR |= TIM_SMCR_SMS_0 |TIM_SMCR_SMS_1;	// SMS=011 for counting on both TI1 and TI2 edges
	TIM4->SMCR |= TIM_SMCR_ETF;						// Enable digital filter
	TIM4->CNT = 32000;								// Start counter at mid way point
	TIM4->CR1 |= TIM_CR1_CEN;

	// R Encoder using timer functionality - PC6 and PC7
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR6_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOC->MODER |= GPIO_MODER_MODER6_1;			// Set alternate function
	GPIOC->AFR[0] |= 3 << 24;						// Alternate function 3 is TIM8_CH1

	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR7_0;			// Set pin to pull up:  01 Pull-up; 10 Pull-down; 11 Reserved
	GPIOC->MODER |= GPIO_MODER_MODER7_1;			// Set alternate function
	GPIOC->AFR[0] |= 3 << 28;						// Alternate function 3 is TIM8_CH2

	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;				// Enable Timer 8
	TIM8->PSC = 0;									// Set prescaler
	TIM8->ARR = 0xFFFF; 							// Set auto reload register to max
	TIM8->SMCR |= TIM_SMCR_SMS_0 |TIM_SMCR_SMS_1;	// SMS=011 for counting on both TI1 and TI2 edges
	TIM8->SMCR |= TIM_SMCR_ETF;						// Enable digital filter
	TIM8->CNT = 32000;								// Start counter at mid way point
	TIM8->CR1 |= TIM_CR1_CEN;


	NVIC_SetPriority(EXTI15_10_IRQn, 4);			// Lower is higher priority
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void InitMidiUART() {
	// PC11 UART4_RX 79
	// [PA1  UART4_RX 24 (AF8) ** NB Dev board seems to have something pulling this pin to ground so can't use]

	RCC->APB1ENR |= RCC_APB1ENR_UART4EN;			// UART4 clock enable

	GPIOC->MODER |= GPIO_MODER_MODER11_1;			// Set alternate function on PC11
	GPIOC->AFR[1] |= 0b1000 << 12;					// Alternate function on PC11 for UART4_RX is 1000: AF8

	int Baud = (SystemCoreClock / 4) / (16 * 31250);
	UART4->BRR |= Baud << 4;						// Baud Rate (called USART_BRR_DIV_Mantissa) = (Sys Clock: 180MHz / APB1 Prescaler DIV4: 45MHz) / (16 * 31250) = 90
	UART4->CR1 &= ~USART_CR1_M;						// Clear bit to set 8 bit word length
	UART4->CR1 |= USART_CR1_RE;						// Receive enable

	// Set up interrupts
	UART4->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(UART4_IRQn, 3);				// Lower is higher priority
	NVIC_EnableIRQ(UART4_IRQn);

	UART4->CR1 |= USART_CR1_UE;						// USART Enable

}



*/



