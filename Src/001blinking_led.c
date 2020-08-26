#include "stm32f103c8.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GpioAddress led = { .pin = 0, .port = GPIOA, };
	GpioSettings settings = { .mode = OUTPUT_MODE_10MHZ, .type =
			PUSH_PULL_OUTPUT, .pupd = INPUT_PULL_UP };
	gpio_peri_clock_ctrl(led.port, ENABLE);
	gpio_init(&led, &settings);
	while (1) {
		gpio_toggle_pin(&led);
		delay();
	}
}
