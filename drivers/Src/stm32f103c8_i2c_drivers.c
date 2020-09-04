#include "stm32f103c8_i2c_driver.h"
#include "stm32f103c8_rcc_driver.h"

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

void i2c_init(I2CHandler *handler) {

	// Configure Ack
	handler->address->CR1 |= ((handler->settings.ack) << I2C_CR1_ACK);

	// Configure FREQ
	handler->address->CR2 |= (rcc_get_pclk1_value() & 0x3F);

	// Configure device address (we only use 7 bit)
	handler->address->OAR1 |= (handler->settings.device_address << 1);

	// CCR calculations (speed)
	uint16_t ccr_value = 0;
	if (handler->settings.scl_speed <= I2C_SCL_SPEED_SM) {
		// Standard
		ccr_value = rcc_get_pclk1_value() / (2 * handler->settings.scl_speed);
	} else {
		// Fast
		handler->address->CCR |= (1 << I2C_CCR_DUTY);
		handler->address->CCR |= (handler->settings.duty_cycle << I2C_CCR_FS);
		if (handler->settings.duty_cycle == FM_DUTY_2) {
			ccr_value = rcc_get_pclk1_value()
					/ (3 * handler->settings.scl_speed);
		} else {
			ccr_value = rcc_get_pclk1_value()
					/ (25 * handler->settings.scl_speed);
		}
	}
	handler->address->CCR |= ccr_value & 0xFFF;

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
