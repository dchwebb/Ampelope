void SysTick_Handler(void) {

	SysTickVal++;
}

void TIM3_IRQHandler() {
	TIM3->SR &= ~TIM_SR_UIF;
	GPIOC->ODR |= GPIO_IDR_ID6;
	envelope.calcEnvelope();
	GPIOC->ODR &= ~GPIO_ODR_ODR_6;
}

void USB_LP_IRQHandler() {
	usb.USBInterruptHandler();
}

void NMI_Handler(void) {}

void HardFault_Handler(void) {
	while (1) {}
}

void MemManage_Handler(void) {
	while (1) {}
}

void BusFault_Handler(void) {
	while (1) {}
}

void UsageFault_Handler(void) {
	while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

