#ifndef INC_STM32F103C8_SPI_DRIVER_H_
#define INC_STM32F103C8_SPI_DRIVER_H_

#include "stm32f103c8.h"

typedef enum {
	SLAVE = 0, MASTER = 1
} SpiDeviceMode;

typedef enum {
	DF_8_BIT_ = 0, DF_16_BIT = 1
} SpiDataFrameFormat;

typedef enum {
	SPI_MODE_0 = 0,  // CPOL: 0 CPHA: 0
	SPI_MODE_1,      // CPOL: 0 CPHA: 1
	SPI_MODE_2,      // CPOL: 1 CPHA: 0
	SPI_MODE_3,      // CPOL: 1 CPHA: 1
} SpiMode;

typedef enum {
	SSM_DISABLED = 0, SSM_ENABLED
} SpiSoftwareSlaveManagement;

typedef struct {
	SpiDeviceMode device_mode;
	SpiDataFrameFormat format;
	SpiMode mode;
	SpiSoftwareSlaveManagement ssm;
} SpiSettings;

typedef struct {
	SpiRegDef *address;
	SpiSettings settings;
} SpiHandler;

//Peripheral Clock Setup
void spi_peri_clock_ctrl(SpiRegDef *address, uint8_t en_or_di);

//Initialization
void spi_init(SpiHandler *handler, SpiSettings *settings);
void spi_deinit(SpiRegDef *address);

// Data TX and RX
void spi_tx(SpiRegDef *address, uint8_t *tx_buffer, uint32_t len);
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len);

// IRQ Handling
void spi_irq__interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void spi_irq_handling(SpiHandler *address);

#endif /* INC_STM32F103C8_SPI_DRIVER_H_ */
