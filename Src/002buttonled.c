#include "stm32f103c8.h"

void delay(void) {
	for (uint32_t i = 0; i < 5000; i++);
}

int main(void) {
	GpioAddress led = { .pin = 13, .port = GPIOC, };
	GpioSettings settings_led = { .mode = OUTPUT_MODE_10MHZ, .type =
					PUSH_PULL_OUTPUT, .pupd = INPUT_PULL_UP };

	GpioAddress btn = { .pin = 0, .port = GPIOA, };
	GpioSettings settings_btn = { .mode = INPUT_MODE, .type =
				PUPD_INPUT, .pupd = INPUT_PULL_UP };
	gpio_peri_clock_ctrl(led.port, ENABLE);
	gpio_peri_clock_ctrl(btn.port, ENABLE);
	gpio_init(&led, &settings_led);
	gpio_init(&btn, &settings_btn);
	while (1) {
		if(gpio_read_pin(&btn) == GPIO_STATE_LOW) {
			gpio_toggle_pin(&led);
			delay();
		}
	}
}
