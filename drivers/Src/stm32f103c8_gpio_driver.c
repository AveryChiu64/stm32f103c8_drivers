#include "stm32f103c8_gpio_driver.h"

/*******************************************************************
 * NAME : gpio_peri_clock_ctrl
 *
 * DESCRIPTION : Enables or dsiables the peripheral clock
 *
 *PARAMETERS:	GpioRegDef 		*port			Address of GPIO Port
 * 			  	uint8_t		 	 en_or_di		ENABLE or DISABLE macros
 *
 * OUTPUTS : void
 */
void gpio_peri_clock_ctrl(GpioRegDef *port, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (port == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (port == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (port == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (port == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (port == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (port == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (port == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (port == GPIOG) {
			GPIOG_PCLK_EN();
		}
	} else {
		if (port == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (port == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (port == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (port == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (port == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (port == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (port == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (port == GPIOG) {
			GPIOG_PCLK_DI();
		}
	}
}

void gpio_init(GpioAddress *address, GpioSettings *settings) {
	if (GpioAddress.pin > MAX_PIN_HIGH_REG) {
		printf("Pin number out of range\n");
	}

	if(!settings->pupd) {
		settings->pupd = 0;
	}

	address->port->CRL &= ~( 0x3 << (4 * address->pin));
	address->port->CRL |= (settings->mode << (4 * address->pin));

	address->port->CRL &= ~( 0x3 << (4 * address->pin + 2));
	address->port->CRL |= (settings->type << (4 * address->pin + 2));

	address->port->ODR &= ~( 0x3 << (address->pin));
	address->port->ODR |= (settings->pupd << (address->pin));
}

void gpio_deinit(const GpioAddress *address);
uint8_t gpio_read_pin(GpioAddress *address);
uint16_t gpio_read_port(GpioRegDef *port);
void gpio_write_pin(GpioAddress *address, GpioState state);
void gpio_write_port(GpioRegDef *port, GpioState state);
void gpio_toggle_pin(GpioAddress *address);

// IRQ Configuration and ISR Handling
void gpio_irq_config(uint8_t irq_numbeer, uint8_t irq_priority,
		uint8_t en_or_di);
void gpio_irq_handling(GpioAddress address);
