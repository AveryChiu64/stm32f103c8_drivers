#include "stm32f103c8.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GpioAddress led = { .pin = 13, .port = GPIOC, };
	GpioSettings settings = { .mode = OUTPUT_MODE_10MHZ, .type =
			PUSH_PULL_OUTPUT, };
	gpio_peri_clock_ctrl(led.port, ENABLE);
	gpio_init(&led, &settings);
	gpio_write_pin(&led,GPIO_STATE_HIGH);
	while (1) {
	}
}
