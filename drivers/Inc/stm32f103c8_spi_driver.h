#ifndef INC_STM32F103C8_SPI_DRIVER_H_
#define INC_STM32F103C8_SPI_DRIVER_H_

#include "stm32f103c8.h"

// Flags
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1 << SPI_SR_BSY)

// SPI Application States
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

typedef enum {
	SLAVE = 0, MASTER = 1
} SpiDeviceMode;

typedef enum {
	FULL_DUPLEX = 0,
	HALF_DUPLEX,
	SIMPLEX_RX,
}SpiBusConfig;

typedef enum {
	SPI_DFF_8_BIT = 0, SPI_DFF_16_BIT = 1
} SpiDataFrameFormat;

typedef enum {
	SPI_MODE_0 = 0,  // CPOL: 0 CPHA: 0
	SPI_MODE_1,      // CPOL: 0 CPHA: 1
	SPI_MODE_2,      // CPOL: 1 CPHA: 0
	SPI_MODE_3,      // CPOL: 1 CPHA: 1
} SpiMode;

typedef enum {
	SSM_DI = 0, SSM_EN
} SpiSoftwareSlaveManagement;

typedef enum {
	SPI_SCLK_SPEED_DIV2 = 0,
	SPI_SCLK_SPEED_DIV4,
	SPI_SCLK_SPEED_DIV8,
	SPI_SCLK_SPEED_DIV16,
	SPI_SCLK_SPEED_DIV32,
	SPI_SCLK_SPEED_DIV64,
	SPI_SCLK_SPEED_DIV128,
	SPI_SCLK_SPEED_DIV256,
}SpiBaudRate;

typedef struct {
	SpiDeviceMode device_mode;
	SpiBusConfig bus_config;
	SpiDataFrameFormat dff;
	SpiMode mode;
	SpiSoftwareSlaveManagement ssm;
	SpiBaudRate br;
} SpiSettings;

typedef struct {
	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
	uint32_t tx_len;
	uint32_t rx_len;
	uint8_t tx_state;
	uint8_t rx_state;
}SpiStorage;

typedef struct {
	SpiRegDef *address;
	SpiSettings settings;
	SpiStorage storage;
} SpiHandler;

//Peripheral Clock Setup
void spi_peri_clock_ctrl(SpiRegDef *address, uint8_t en_or_di);

//Initialization
void spi_init(SpiHandler *handler);

//Other peripheral control
void spi_peripheral_control(SpiRegDef *address, uint8_t en_or_di);
void spi_ssi_config(SpiRegDef *address, uint8_t en_or_di);
void spi_ssoe_config(SpiRegDef *address, uint8_t en_or_di);
uint8_t spi_get_flag_status(SpiRegDef *address, uint32_t flag_name);

// Data TX and RX
void spi_tx(SpiRegDef *address, uint8_t *tx_buffer, uint32_t len);
void spi_rx(SpiRegDef *address, uint8_t *rx_buffer, uint32_t len);

// Data TX and RX with Interrupt
uint8_t spi_tx_it(SpiHandler *handler, uint8_t *tx_buffer, uint32_t len);
uint8_t spi_rx_it(SpiHandler *handler, uint8_t *rx_buffer, uint32_t len);

// IRQ Handling
void spi_irq__interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void spi_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void spi_irq_handling(SpiHandler *address);

#endif /* INC_STM32F103C8_SPI_DRIVER_H_ */
