#include "stm32f103c8_spi_driver.h"

void spi_peri_clock_ctrl(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (address == SPI1) {
			SPI1_PCLK_EN();
		} else if (address == SPI2) {
			SPI2_PCLK_EN();
		} else if (address == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (address == SPI1) {
			SPI1_PCLK_DI();
		} else if (address == SPI2) {
			SPI2_PCLK_DI();
		} else if (address == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

void spi_init(SpiHandler *handler) {

	// Configure device mode
	handler->address->CR[0] |= ((handler->settings.device_mode) << SPI_CR1_MSTR);

	// Bus configuration
	switch (handler->settings.bus_config) {
	case FULL_DUPLEX:
		handler->address->CR[0] &= ~(1 << SPI_CR1_BIDI_MODE);
		break;
	case HALF_DUPLEX:
		handler->address->CR[0] |= (1 << SPI_CR1_BIDI_MODE);
		break;
	case SIMPLEX_RX:
		handler->address->CR[0] &= ~(1 << SPI_CR1_BIDI_MODE);
		handler->address->CR[0] |= (1 << SPI_CR1_RX_ONLY);
		break;
	}

	// Configure data frame format
	handler->address->CR[0] |= ((handler->settings.dff) << SPI_CR1_DFF);

	// Configure SPI mode
	// Note that the bit for CPOL is right beside CPHA
	handler->address->CR[0] |= ((handler->settings.mode) << SPI_CR1_CPHA);

	// Configure ssm
	handler->address->CR[0] |= ((handler->settings.ssm) << SPI_CR1_SSM);

	// Configure baud rate
	handler->address->CR[0] |= ((handler->settings.br) << SPI_CR1_BR1);
}

void spi_tx(SpiRegDef *address, uint8_t *tx_buffer, uint32_t len);
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len);

void spi_irq__interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void spi_irq_handling(SpiHandler *address);
