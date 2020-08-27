#include "stm32f103c8.h"

void delay(void) {
	for (uint32_t i = 0; i < 5000; i++)
		;
}

GpioAddress led = { .pin = 13, .port = GPIOC, };
GpioSettings settings_led = { .mode = OUTPUT_MODE_10MHZ, .type =
		PUSH_PULL_OUTPUT, .pupd = INPUT_PULL_UP };

GpioAddress btn = { .pin = 0, .port = GPIOA, };
GpioSettings settings_btn = { .mode = INPUT_MODE, .type = PUPD_INPUT, .pupd =
		INPUT_PULL_UP, .edge = INTERRUPT_EDGE_FALLING };

int main(void) {
	gpio_peri_clock_ctrl(led.port, ENABLE);
	gpio_peri_clock_ctrl(btn.port, ENABLE);
	gpio_init(&led, &settings_led);
	gpio_init(&btn, &settings_btn);
	gpio_irq__interrupt_config(IRQ_NO_EXTI0, ENABLE);
	gpio_irq_priority_config(IRQ_NO_EXTI0, NVIC_IRQ_PRIORITY0);
	while (1) {
	}
}

void EXTI0_IRQHandler() {
	// Handle the interrupt
	gpio_irq_handling(btn.pin);
	gpio_toggle_pin(&led);
	delay();
}
