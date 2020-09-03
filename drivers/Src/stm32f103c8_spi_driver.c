#include "stm32f103c8_spi_driver.h"

// Private Function Prototypes
static void spi_txe_interrupt_handle(SpiHandler *handler);
static void spi_rxne_interrupt_handle(SpiHandler *handler);
static void spi_ovr_err_interrupt_handle(SpiHandler *handler);

/*******************************************************************
 * NAME : spi_peri_clock_ctrl
 *
 * DESCRIPTION : Enables/disables peripheral clock for SPI
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		en_or_di		Enable or disable
 *
 * OUTPUTS : 	void
 */
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

/*******************************************************************
 * NAME : spi_init
 *
 * DESCRIPTION : Enables/disables peripheral clock for SPI
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 *
 * OUTPUTS : 	void
 */
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

/*******************************************************************
 * NAME : spi_peripheral_control
 *
 * DESCRIPTION : Enables/disables a SPI
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		en_or_di		Enable or disable
 *
 * OUTPUTS : 	void
 */
void spi_peripheral_control(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		address->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		address->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*******************************************************************
 * NAME : spi_ssi_config
 *
 * DESCRIPTION : Enables/disables the internal slave select
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		en_or_di		Enable or disable
 *
 * OUTPUTS : 	void
 */
void spi_ssi_config(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		address->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		address->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*******************************************************************
 * NAME : spi_ssi_config
 *
 * DESCRIPTION : Enables/disables the the slave select output. This should only
 * 				 be disabled when dealing with a multimaster configuration.
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		en_or_di		Enable or disable
 *
 * OUTPUTS : 	void
 */
void spi_ssoe_config(SpiRegDef *address, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		address->CR1 |= (1 << SPI_CR2_SSOE);
	} else {
		address->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*******************************************************************
 * NAME : spi_get_flag_status
 *
 * DESCRIPTION : Gets the status of the flag from the status register for SPI
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint32_t 		flag_name		The name of the flag
 *
 * OUTPUTS : 	uint8_t			Either FLAT_SET or FLAG_RESET macros
 */
uint8_t spi_get_flag_status(SpiRegDef *address, uint32_t flag_name) {
	if (address->SR & flag_name) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*******************************************************************
 * NAME : spi_tx
 *
 * DESCRIPTION : Sends data with SPI
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		*tx_buffer		tx buffer to temporarily store the data to send
 * 				uint32_t		len				The length of the tx_buffer data
 *
 * OUTPUTS : 	void
 */
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

/*******************************************************************
 * NAME : spi_rx
 *
 * DESCRIPTION : Receives data with SPI
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 * 				uint8_t 		*rx_buffer		rx buffer to temporarily store the data received
 * 				uint32_t		len				The length of the rx_buffer data
 *
 * OUTPUTS : 	void
 */
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len) {
	while (len > 0) {
		while (spi_get_flag_status(address, SPI_RXNE_FLAG) == FLAG_RESET)
			;
		// Check DFF bit in CR1
		if (address->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit data frame
			// Load data from data register to rx buffer
			*((uint16_t*) rx_buffer) = address->DR;
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

/*******************************************************************
 * NAME : spi_tx_it
 *
 * DESCRIPTION : Sends data with SPI, uses interrupt instead of polling to send data
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 * 				uint8_t 		*tx_buffer		tx buffer to temporarily store the data to send
 * 				uint32_t		len				The length of the tx_buffer data
 *
 * OUTPUTS : 	uint8_t			Returns the application state (eg SPI_READY)
 */
uint8_t spi_tx_it(SpiHandler *handler, uint8_t *tx_buffer, uint32_t len) {

	uint8_t state = handler->storage.tx_state;
	if (state != SPI_BUSY_IN_TX) {

		// Save tx buffer address and length
		handler->storage.tx_buffer = tx_buffer;
		handler->storage.tx_len = len;

		// Mark SPI state as busy
		handler->storage.tx_state = SPI_BUSY_IN_TX;

		// Enable TXEIE control bit to enable interrupt whenever TXE flag is set in SR
		handler->address->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/*******************************************************************
 * NAME : spi_rx_it
 *
 * DESCRIPTION : Receives data with SPI, uses interrupt instead of polling to receive data
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 * 				uint8_t 		*rx_buffer		rx buffer to temporarily store the data received
 * 				uint32_t		len				The length of the rx_buffer data
 *
 * OUTPUTS : 	uint8_t			Returns the application state (eg SPI_READY)
 */
uint8_t spi_rx_it(SpiHandler *handler, uint8_t *rx_buffer, uint32_t len) {
	uint8_t state = handler->storage.rx_state;
	if (state != SPI_BUSY_IN_RX) {

		// Save rx buffer address and length
		handler->storage.rx_buffer = rx_buffer;
		handler->storage.rx_len = len;

		// Mark SPI state as busy
		handler->storage.rx_state = SPI_BUSY_IN_RX;

		// Enable RXEIE control bit to enable interrupt whenever RXE flag is set in SR
		handler->address->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/*******************************************************************
 * NAME : spi_irq_interrupt_config
 *
 * DESCRIPTION : Enables/disables peripheral clock for SPI
 *
 * PARAMETERS:	uint8_t 	irq_number		The Interrupt Request Number as denoted by the processor
 *				uint8_t 	en_or_di		Enable or disable
 *
 * OUTPUTS : 	void
 */
void spi_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_di) {
	uint8_t index = irq_number / 32;
	uint8_t section = irq_number % 32;
	if (en_or_di == ENABLE) {
		*(NVIC_ISER_BASEADDR + index * 4) |= (1 << section);
	} else {
		*(NVIC_ICER_BASEADDR + index * 4) |= (1 << section);
	}
}

/*******************************************************************
 * NAME : spi_irq_priority_config
 *
 * DESCRIPTION : Enables/disables peripheral clock for SPI
 *
 * PARAMETERS:	uint8_t 	irq_number		The Interrupt Request Number as denoted by the processor
 *				NvicIrqPriority 	irq_priority	The priority of the interrupt
 *
 * OUTPUTS : 	void
 */
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority) {
	// Find IPR register
	uint8_t index = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;
	uint8_t shift = ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*(NVIC_IPR_BASEADDR + (index * 4)) |= (irq_priority << shift);
}

/*******************************************************************
 * NAME : spi_irq_handling
 *
 * DESCRIPTION : Checks which type of interrupt has occurred and runs callback
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 *
 * OUTPUTS : 	void
 */
void spi_irq_handling(SpiHandler *handler) {
	// Check for tx flag and mask
	if ((handler->address->SR & (1 << SPI_SR_TXE))
			&& (handler->address->CR2 & (1 << SPI_CR2_TXEIE))) {
		spi_txe_interrupt_handle(handler);
	}

	// Check for rx flag and mask
	if ((handler->address->SR & (1 << SPI_SR_RXNE))
			&& (handler->address->CR2 & (1 << SPI_CR2_RXNEIE))) {
		spi_rxne_interrupt_handle(handler);
	}
	// Check for OVR flag
	if ((handler->address->SR & (1 << SPI_SR_OVR))
			&& (handler->address->CR2 & (1 << SPI_CR2_ERRIE))) {
		spi_ovr_err_interrupt_handle(handler);
	}
}

/*******************************************************************
 * NAME : spi_clear_ovr_flag
 *
 * DESCRIPTION : Clears overrun flag by reading the data and status register
 *
 * PARAMETERS:	SpiRegDef 		*address		Address of the SPI peripheral
 *
 * OUTPUTS : 	void
 */
void spi_clear_ovr_flag(SpiRegDef *address) {
	uint8_t temp;
	temp = address->DR;
	temp = address->SR;
	(void)temp;
}

/*******************************************************************
 * NAME : spi_close_tx
 *
 * DESCRIPTION : Halts SPI transmission
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 *
 * OUTPUTS : 	void
 */
void spi_close_tx(SpiHandler *handler) {
	handler->address->CR2 &= ~(1 << SPI_CR2_TXEIE);
	handler->storage.tx_buffer = NULL;
	handler->storage.tx_len = 0;
	handler->storage.tx_state = SPI_READY;
}

/*******************************************************************
 * NAME : spi_close_rx
 *
 * DESCRIPTION : Halts SPI receiving
 *
 * PARAMETERS:	SpiHandler 		*handler		Holds the address, settings, and storage for SPI
 *
 * OUTPUTS : 	void
 */
void spi_close_rx(SpiHandler *handler) {
	handler->address->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	handler->storage.rx_buffer = NULL;
	handler->storage.rx_len = 0;
	handler->storage.rx_state = SPI_READY;
	spi_application_event_callback(handler, SPI_EVENT_RX_CMPLT);
}

/*******************************************************************
 * NAME : spi_application_event_callback
 *
 * DESCRIPTION : A weak callback function for the user to override
 * 				that runs when transmission/receiving is complete
 *
 * PARAMETERS:	SpiHandler 		*handler			Holds the address, settings, and storage for SPI
 * 				uint8_t 		application_event	Shows what interrupt has occurred (eg SPI_EVENT_TX_CMPLT)
 *
 * OUTPUTS : 	void
 */
__weak void spi_application_event_callback(SpiHandler *handler, uint8_t application_event) {
	// User may override this function since it is weak
}

// Helper Functions to handle SPI interrupts during transmission, receiving, or when an overrun error occurs
// This is used for the spi_tx_it and spi_rx_it when interrupts are used instead of polling
static void spi_txe_interrupt_handle(SpiHandler *handler) {
	if (handler->address->CR1 & (1 << SPI_CR1_DFF)) {
		handler->address->DR = *((uint16_t*) (handler->storage.tx_buffer));
		handler->storage.tx_len -= 2;
		(uint16_t*) (handler->storage.tx_buffer)++;
	} else {
		handler->address->DR = *(handler->storage.tx_buffer);
		handler->storage.tx_len--;
		handler->storage.tx_buffer++;
	}

	// End SPI tx when length becomes 0
	if (!handler->storage.tx_len) {
		spi_close_tx(handler);
		spi_application_event_callback(handler, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SpiHandler *handler) {
	if (handler->address->CR1 & (1 << SPI_CR1_DFF)) {
		*((uint16_t*) (handler->storage.rx_buffer)) = handler->address->DR;
		handler->storage.rx_len -= 2;
		(uint16_t*) (handler->storage.rx_buffer)++;
	} else {
		*(handler->storage.rx_buffer) = handler->address->DR;
		handler->storage.rx_len--;
		handler->storage.rx_buffer++;
	}

	// End SPI rx when length becomes 0
	if (!handler->storage.rx_len) {
		spi_close_rx(handler);
		spi_application_event_callback(handler, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SpiHandler *handler) {
	// Clear OVR Flag
	if (handler->storage.tx_state != SPI_BUSY_IN_TX) {
		spi_clear_ovr_flag(handler->address);
	}
	// Inform application
	spi_application_event_callback(handler, SPI_EVENT_RX_CMPLT);
}
