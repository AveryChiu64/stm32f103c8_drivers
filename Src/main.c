#include "stm32f103c8.h"

int main(void) {

	return 0;
}

void EXTI0_IRQHandler() {
	// Handle the interrupt
	gpio_irq_handling(0);
}
