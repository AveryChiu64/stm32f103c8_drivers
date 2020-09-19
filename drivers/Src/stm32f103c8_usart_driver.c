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

void usart_init(UsartHandler *handler) {
	usart_peri_clock_ctrl(handler->address, ENABLE);
	// mode
	handler->address->CR1 |= (handler->settings.mode << USART_RE);

	// parity
	if (handler->settings.parity == USART_PARITY_DISABLE) {
		handler->address->CR1 &= ~(1 << USART_PCE);
	} else {
		handler->address->CR1 |= (1 << USART_PCE);
		if (handler->settings.parity == USART_PARITY_EN_EVEN) {
			handler->address->CR1 &= ~(1 << USART_PS);
		} else {
			handler->address->CR1 |= (1 << USART_PS);
		}
	}
	// word length
	address->address->CR1 |= (handler->settings.word_length << USART_M);

	// stop bits
	handler->address->CR2 |= (handler->settings.stop_bits << USART_STOP);

	// hw flow ctrl
	handler->address->CR3 |= (handler->settings.flow_ctrl << USART_RTSE);
}

// Data Send and Receive
void usart_tx(UsartRegDef *address, uint8_t *tx_buffer, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		// TXE Flag
		while(!usart_get_flag_status(handler->address,FLAG));
		handler->address->DR =
		tx_buffer++;
		// TC flag
		while(!usart_get_flag_status(handler->address,FLAG));
	}
}
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

void usart_peripheral_control(UsartRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		address->CR1 |= (1 << USART_UE);
	} else {
		address->CR1 &= ~(1 << USART_UE);
	}
}

uint8_t usart_get_flag_status(UsartRegDef *address, uint32_t flag_name);
void usart_clear_flag(UsartRegDef *address, uint16_t status_flag_name);
void usart_application_event_callback(UsartHandler *handler, uint8_t app_event);
