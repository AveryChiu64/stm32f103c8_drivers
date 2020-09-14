#include "stm32f103c8_usart_driver.h"

void usart_peri_clock_ctrl(UsartRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (address == USART1) {
			USART1_PCLK_EN();
		} else if (address == USART2) {
			USART2_PCLK_EN();
		} else if (address == USART3) {
			USART3_PCLK_EN();
		} else if (address == UART4) {
			UART4_PCLK_EN();
		} else if (address == UART5) {
			UART5_PCLK_EN();
		}
	} else {
		if (address == USART1) {
			USART1_PCLK_DI();
		} else if (address == USART2) {
			USART2_PCLK_DI();
		} else if (address == USART3) {
			USART3_PCLK_DI();
		} else if (address == UART4) {
			UART4_PCLK_DI();
		} else if (address == UART5) {
			UART5_PCLK_DI();
		}
	}
}

// Init and De-init
void usart_init(UsartHandler *handler);
void usart_deinit(UsartRegDef *address);

// Data Send and Receive
void usart_tx(UsartRegDef *address, uint8_t *tx_buffer, uint32_t len);
void usrat_rx(UsartRegDef *address, uint8_t *rx_buffer, uint32_t len);
uint8_t usart_tx_it(UsartHandler *handler, uint8_t *tx_buffer, uint32_t len);
uint8_t usart_rx_it(UsartHandler *handler, uint8_t *rx_buffer, uint32_t len);

void usart_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_di) {
	uint8_t index = irq_number / 32;
	uint8_t section = irq_number % 32;
	if (en_or_di == ENABLE) {
		*(NVIC_ISER_BASEADDR + index * 4) |= (1 << section);
	} else {
		*(NVIC_ICER_BASEADDR + index * 4) |= (1 << section);
	}
}

void usart_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority) {
	// Find IPR register
	uint8_t index = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;
	uint8_t shift = ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*(NVIC_IPR_BASEADDR + (index * 4)) |= (irq_priority << shift);

}

void usart_irq_handling(UsartHandler *handler);

void usart_peripheral_control(UsartRegDef *address, uint8_t en_or_di);
uint8_t usart_get_flag_status(UsartRegDef *address, uint32_t flag_name);
void usart_clear_flag(UsartRegDef *address, uint16_t status_flag_name);
void usart_application_event_callback(UsartHandler *handler, uint8_t app_event);
