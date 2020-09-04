#include "stm32f103c8_i2c_driver.h"

uint16_t ahb_prescaler[8] = { 2, 4, 8, 16, 32, 64, 128, 256 };
uint16_t apb1_prescaler[4] = { 2, 4, 8, 16 };

void i2c_peri_clock_ctrl(I2CRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (address == I2C1) {
			I2C1_PCLK_EN();
		} else if (address == I2C2) {
			I2C2_PCLK_EN();
		}
	} else {
		if (address == I2C1) {
			I2C1_PCLK_DI();
		} else if (address == I2C2) {
			I2C2_PCLK_DI();
		}
	}
}

uint32_t rcc_get_pclk1_value() {
	uint8_t hpre, ppre, ahb, apb1;
	uint32_t sysclk;

	uint8_t clksrc = ((RCC->CFGR >> 2) & 0x3);

	// HSI
	if (clksrc == 0) {
		sysclk = 1600000;
	}
	// HSE
	else if (clksrc == 1) {
		sysclk = 800000;
	}
	// AHB
	hpre = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	if (hpre < 8) {
		ahb = 1;
	} else {
		ahb = ahb_prescaler[hpre - 8];
	}

	// APB1
	ppre = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0xF);
	if (ppre < 4) {
		apb1 = 1;
	} else {
		apb1 = apb1_prescaler[ppre - 4];
	}
	return (sysclk / ahb) / apb1;
}

void i2c_init(I2CHandler *handler) {
	// Configure mode

	// Configure speed
	// Configure device address
	// Configure Ack
	handler->address->CR1 |= ((handler->settings.ack) << I2C_CR1_ACK);
	// Configure rise time
}
void i2c_peripheral_control(I2CRegDef *address, uint8_t en_or_di);
uint8_t i2c_get_flag_status(I2CRegDef *address, uint32_t flag_name);

void i2c_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_di) {
	uint8_t index = irq_number / 32;
	uint8_t section = irq_number % 32;
	if (en_or_di == ENABLE) {
		*(NVIC_ISER_BASEADDR + index * 4) |= (1 << section);
	} else {
		*(NVIC_ICER_BASEADDR + index * 4) |= (1 << section);
	}
}

void i2c_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority) {
	uint8_t index = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;
	uint8_t shift = ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*(NVIC_IPR_BASEADDR + (index * 4)) |= (irq_priority << shift);

}

__weak void i2c_application_event_callback(I2CHandler *handler,
		uint8_t application_event) {

}
