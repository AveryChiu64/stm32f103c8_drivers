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

void spi_init(SpiHandler *handler, SpiSettings *settings);
void spi_deinit(SpiRegDef *address);

void spi_tx(SpiRegDef *address, uint8_t *tx_buffer, uint32_t len);
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len);

void spi_irq__interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void spi_irq_handling(SpiHandler *address);
