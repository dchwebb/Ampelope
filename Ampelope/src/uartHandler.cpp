#include "uartHandler.h"

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[100];
volatile bool uartCmdRdy = false;

// Manages communication to ST Link debugger UART
void InitUart() {
	// MODER 00: Input mode, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode (reset state)

	// PC4 USART1_TX, PC5 USART1_RX [PA3 RX USART2_RX]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			// reset and clock control - advanced high performance bus - GPIO port C
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

//	GPIOC->MODER &= ~GPIO_MODER_MODER4_1;			// configure PC4 test out
//	GPIOC->MODER &= ~GPIO_MODER_MODER5_1;			// configure PC5 test out

	GPIOC->MODER &= ~GPIO_MODER_MODE4_0;			// Set alternate function on PC4
	GPIOC->AFR[0] |= 7 << GPIO_AFRL_AFSEL4_Pos;		// Alternate function for USART1_TX is AF7
	GPIOC->MODER &= ~GPIO_MODER_MODE5_0;			// Set alternate function on PC5
	GPIOC->AFR[0] |= 7 << GPIO_AFRL_AFSEL5_Pos;		// Alternate function for USART1_RX is AF7

	// By default clock source is muxed to peripheral clock 2 which is system clock (change clock source in RCC->CCIPR1)
	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16
	int USARTDIV = (SystemCoreClock) / 230400;		//clk / desired_baud
	USART1->BRR = USARTDIV & ~8;					// BRR[3] must not be set
	USART1->CR1 &= ~USART_CR1_M;					// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	USART1->CR1 |= USART_CR1_RE;					// Receive enable
	USART1->CR1 |= USART_CR1_TE;					// Transmitter enable

	// Set up interrupts
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 3);				// Lower is higher priority
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1 |= USART_CR1_UE;					// USART Enable

	// configure PC13 user button on nucleo board
	GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;

}


std::string IntToString(const int32_t& v) {
	return std::to_string(v);
}

std::string HexToString(const uint32_t& v, const bool& spaces) {
	char buf[50];
	sprintf(buf, "%X", v);
	return std::string(buf);
//	std::string(buf).append("\r\n")

	/*
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << v;
	if (spaces) {
		//std::string s = ss.str();
		return ss.str().insert(2, " ").insert(5, " ").insert(8, " ");
	}
	return ss.str();
	*/
}

std::string HexByte(const uint16_t& v) {
	char buf[50];
	sprintf(buf, "%X", v);
	return std::string(buf);

//	std::stringstream ss;
//	ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << v;
//	return ss.str();
}

void uartSendChar(char c) {
	while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0);
	USART1->TDR = c;
}

void uartSendString(const char* s) {
	char c = s[0];
	uint8_t i = 0;
	while (c) {
		while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART1->TDR = c;
		c = s[++i];
	}
}

void uartSendString(const std::string& s) {
	for (char c : s) {
		while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART1->TDR = c;
	}
}

extern "C" {

// USART Decoder
void USART1_IRQHandler() {
	if (!uartCmdRdy) {
		uartCmd[uartCmdPos] = USART1->RDR; 				// accessing RDR automatically resets the receive flag
		if (uartCmd[uartCmdPos] == 10) {
			uartCmdRdy = true;
			uartCmdPos = 0;
		} else {
			uartCmdPos++;
		}
	}
}
}
