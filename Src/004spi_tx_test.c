#include "stm32f103c8.h"
#include <string.h>

void spi1_gpio_init() {
	gpio_peri_clock_ctrl(GPIOA,ENABLE);
	GpioSettings gpio_settings = { .mode = OUTPUT_MODE_10MHZ, .type =
			ALTFN_PUSH_PULL_OUTPUT, .pupd = INPUT_PULL_UP };
	GpioAddress mosi = { .pin = 7, .port = GPIOA, };
	GpioAddress miso = { .pin = 6, .port = GPIOA, };
	GpioAddress sck = { .pin = 5, .port = GPIOA, };
	GpioAddress nss = { .pin = 4, .port = GPIOA, };

	gpio_init(&mosi, &gpio_settings);
	gpio_init(&miso, &gpio_settings);
	gpio_init(&sck, &gpio_settings);
	gpio_init(&nss, &gpio_settings);
}

void spi1_init() {
	spi_peri_clock_ctrl(SPI1, ENABLE);
	SpiSettings spi_settings = {
			.device_mode = MASTER,
			.bus_config = FULL_DUPLEX,
			.br = SPI_SCLK_SPEED_DIV2,
			.dff= SPI_DFF_8_BIT,
			.mode = SPI_MODE_0,
			.ssm = SSM_DI
	};

	SpiHandler handler = { .address = SPI1,
			.settings = spi_settings };

	spi_init(&handler);
	};

	int main(void) {
		char user_data[] = "Hello World";
		spi1_gpio_init();
		spi1_init();
		spi_peripheral_control(SPI1, ENABLE);
		spi_ssoe_config(SPI1, ENABLE);
		spi_tx(SPI1,(uint8_t*)user_data,strlen(user_data));
		spi_peripheral_control(SPI1, DISABLE);
		while(1) {

		}
	}

