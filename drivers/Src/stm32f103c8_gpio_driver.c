#include "stm32f103c8_gpio_driver.h"
#include "stdio.h"

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
	if (address->pin > NUM_PINS) {
		printf("Pin number out of range\n");
	}

	if (!settings->pupd) {
		settings->pupd = 0;
	}

	address->port->CRL &= ~(0x3 << (4 * address->pin));
	address->port->CRL |= (settings->mode << (4 * address->pin));

	address->port->CRL &= ~(0x3 << (4 * address->pin + 2));
	address->port->CRL |= (settings->type << (4 * address->pin + 2));

	address->port->ODR &= ~(0x3 << (address->pin));
	address->port->ODR |= (settings->pupd << (address->pin));
}

void gpio_deinit(GpioRegDef *port) {
	if (port == GPIOA) {
		GPIOA_REG_RESET();
	} else if (port == GPIOB) {
		GPIOB_REG_RESET();
	} else if (port == GPIOC) {
		GPIOC_REG_RESET();
	} else if (port == GPIOD) {
		GPIOD_REG_RESET();
	} else if (port == GPIOD) {
		GPIOD_REG_RESET();
	} else if (port == GPIOE) {
		GPIOE_REG_RESET();
	} else if (port == GPIOF) {
		GPIOF_REG_RESET();
	} else if (port == GPIOG) {
		GPIOG_REG_RESET();
	}
}
GpioState gpio_read_pin(GpioAddress *address) {
	// We shift the bit we want to the LSB and mask the other bits
	return (GpioState)((address->port->IDR >> address->pin) & 0x00000001);
}

uint16_t gpio_read_port(GpioRegDef *port) {
	return (uint16_t)(port->IDR);
}

void gpio_write_pin(GpioAddress *address, GpioState state) {
	if(state == GPIO_STATE_HIGH) {
		address->port->ODR |= (1 << address->pin);
	}
	else {
		address->port->ODR &= ~(1 << address->pin);
	}
}

void gpio_write_port(GpioRegDef *port, uint16_t value) {
	port-> ODR = value;
}

void gpio_toggle_pin(GpioAddress *address);

// IRQ Configuration and ISR Handling
void gpio_irq_config(uint8_t irq_numbeer, uint8_t irq_priority,
		uint8_t en_or_di);
void gpio_irq_handling(GpioAddress *address);
