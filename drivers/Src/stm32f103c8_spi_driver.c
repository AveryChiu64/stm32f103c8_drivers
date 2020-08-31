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
	handler->address->CR1 |= ((handler->settings.device_mode) << SPI_CR1_MSTR);

	// Bus configuration
	switch (handler->settings.bus_config) {
	case FULL_DUPLEX:
		handler->address->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);
		break;
	case HALF_DUPLEX:
		handler->address->CR1 |= (1 << SPI_CR1_BIDI_MODE);
		break;
	case SIMPLEX_RX:
		handler->address->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);
		handler->address->CR1 |= (1 << SPI_CR1_RX_ONLY);
		break;
	}

	// Configure data frame format
	handler->address->CR1 |= ((handler->settings.dff) << SPI_CR1_DFF);

	// Configure SPI mode
	// Note that the bit for CPOL is right beside CPHA
	handler->address->CR1 |= ((handler->settings.mode) << SPI_CR1_CPHA);

	// Configure ssm
	handler->address->CR1 |= ((handler->settings.ssm) << SPI_CR1_SSM);

	// Configure baud rate
	handler->address->CR1 |= ((handler->settings.br) << SPI_CR1_BR1);
}

void spi_peripheral_control(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		address->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		address->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
void spi_ssi_config(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
			address->CR1 |= (1 << SPI_CR1_SSI);
		} else {
			address->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}
void spi_ssoe_config(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
			address->CR1 |= (1 << SPI_CR2_SSOE);
		} else {
			address->CR1 &= ~(1 << SPI_CR2_SSOE);
		}
}

uint8_t spi_get_flag_status(SpiRegDef *address, uint32_t flag_name) {
	if (address->SR & flag_name) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void spi_tx(SpiRegDef *address, uint8_t *tx_buffer, uint32_t len) {
	while (len > 0) {
		// Wait until TXE is set
		// TODO: Add watchdog timer
		while (spi_get_flag_status(address, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// Check DFF bit in CR1
		if (address->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit data frame
			// Load data into the data register
			address->DR = *((uint16_t*) (tx_buffer));
			len -= 2;
			(uint16_t*) (tx_buffer)++;
		} else {
			//8 bit data frame
			address->DR = *(tx_buffer);
			len--;
			tx_buffer++;
		}
	}
}
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len) {
	while (len > 0) {
			while (spi_get_flag_status(address, SPI_RXNE_FLAG) == FLAG_RESET);
			// Check DFF bit in CR1
			if (address->CR1 & (1 << SPI_CR1_DFF)) {
				// 16 bit data frame
				// Load data from data register to rx buffer
				*((uint16_t*)rx_buffer) = address->DR;
				len -= 2;
				(uint16_t*) (rx_buffer)++;
			} else {
				//8 bit data frame
				*(rx_buffer) = address->DR;
				len--;
				rx_buffer++;
			}
		}
}

void spi_irq__interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void spi_irq_handling(SpiHandler *address);
