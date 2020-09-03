#include "stm32f103c8_i2c_driver.h"

void i2c_peri_clock_ctrl(I2CRegDef *address, uint8_t en_or_di);
void i2c_init(I2CHandler *handler);
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

__weak void i2c_application_event_callback(I2CHandler *handler,uint8_t application_event) {

}
