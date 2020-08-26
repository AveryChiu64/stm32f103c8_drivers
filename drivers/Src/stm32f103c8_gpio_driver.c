#include "stm32f103c8_gpio_driver.h"
#include "stdio.h"

/*******************************************************************
 * NAME : gpio_peri_clock_ctrl
 *
 * DESCRIPTION : Enables or disables the peripheral clock
 *
 *PARAMETERS:	GpioRegDef 		*port			Address of GPIO Port
 * 			  	uint8_t		 	 en_or_di		ENABLE or DISABLE macros
 *
 * OUTPUTS : 	void
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

/*******************************************************************
 * NAME : gpio_init
 *
 * DESCRIPTION : Initializes the GPIO settings
 *
 *PARAMETERS:	GpioAddress 	*address		Address of GPIO Port
 * 			  	GpioSettings 	*settings 	 	GPIO settings
 *
 * OUTPUTS : 	void
 */
void gpio_init(GpioAddress *address, GpioSettings *settings) {
	if (address->pin > NUM_PINS) {
		printf("Pin number out of range\n");
	}
	if (!settings->pupd) {
		settings->pupd = 0;
	}

	if (address->pin <= MAX_PIN_NUM_CRL) {
		address->port->CRL &= ~(0x3 << (4 * (address->pin)));
		address->port->CRL |= (settings->mode << (4 * (address->pin)));

		address->port->CRL &= ~(0x3 << (4 * (address->pin) + 2));
		address->port->CRL |= (settings->type << (4 * (address->pin) + 2));
	} else {
		address->port->CRH &= ~(0x3 << (4 * ((address->pin) % 8)));
		address->port->CRH |= (settings->mode << (4 * ((address->pin) % 8)));

		address->port->CRH &= ~(0x3 << (4 * (address->pin) + 2));
		address->port->CRH |=
				(settings->type << (4 * ((address->pin) % 8) + 2));
	}

	address->port->ODR &= ~(0x3 << (address->pin));
	address->port->ODR |= (settings->pupd << (address->pin));

	if (settings->edge == INTERRUPT_EDGE_RISING) {
		EXTI->RTSR |= (1 << (address->pin));
		EXTI->FTSR &= ~(1 << (address->pin));
	} else if (settings->edge == INTERRUPT_EDGE_FALLING) {
		EXTI->FTSR |= (1 << (address->pin));
		EXTI->RTSR &= ~(1 << (address->pin));
	} else {
		EXTI->FTSR |= (1 << (address->pin));
		EXTI->RTSR &= ~(1 << (address->pin));
	}
}

/*******************************************************************
 * NAME : gpio_deinit
 *
 * DESCRIPTION : Resets a port
 *
 * PARAMETERS:	GpioRegDef 		*port		The GPIO port
 *
 * OUTPUTS : 	void
 */
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

/*******************************************************************
 * NAME : gpio_read_pin
 *
 * DESCRIPTION : Reads the value from a specific port and pin
 *
 * PARAMETERS:	GpioAddress 	*address		Address of GPIO Port
 *
 * OUTPUTS : 	GpioState						State of the GPIO pin
 */

GpioState gpio_read_pin(GpioAddress *address) {
// We shift the bit we want to the LSB and mask the other bits
	return (GpioState) ((address->port->IDR >> address->pin) & 0x00000001);
}

/*******************************************************************
 * NAME : gpio_read_port
 *
 * DESCRIPTION : Reads the register for the port
 *
 * PARAMETERS:	GpioRegDef 		*port			The GPIO port
 *
 * OUTPUTS : 	uint16_t						The values in the register
 */
uint16_t gpio_read_port(GpioRegDef *port) {
	return (uint16_t) (port->IDR);
}

/*******************************************************************
 * NAME : gpio_write_pin
 *
 * DESCRIPTION : Writes a value to a pin
 *
 * PARAMETERS:	GpioAddress 	*address		Address of GPIO Port
 * 				GpioState 		state			State of the GPIO pin
 *
 * OUTPUTS : 	void
 */

void gpio_write_pin(GpioAddress *address, GpioState state) {
	if (state == GPIO_STATE_HIGH) {
		address->port->ODR |= (1 << address->pin);
	} else {
		address->port->ODR &= ~(1 << address->pin);
	}
}

/*******************************************************************
 * NAME : gpio_write_port
 *
 * DESCRIPTION : Writes a value to the whole register for the port
 *
 * PARAMETERS:	GpioRegDef 		*port			Address of GPIO Port
 * 				uint16_t 		 value			Value to write to the register
 *
 * OUTPUTS : 	void
 */

void gpio_write_port(GpioRegDef *port, uint16_t value) {
	port->ODR = value;
}

/*******************************************************************
 * NAME : gpio_toggle_pin
 *
 * DESCRIPTION : Toggles an output pin's state
 *
 * PARAMETERS:	GpioAddress 	*address		Address of GPIO Port
 *
 * OUTPUTS : 	void
 */

void gpio_toggle_pin(GpioAddress *address) {
	address->port->ODR ^= (1 << address->pin);
}

// IRQ Configuration and ISR Handling
void gpio_irq_config(uint8_t irq_numbeer, uint8_t irq_priority,
		uint8_t en_or_di);
void gpio_irq_handling(GpioAddress *address);
