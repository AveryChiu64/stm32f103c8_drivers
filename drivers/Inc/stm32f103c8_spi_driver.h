#ifndef INC_STM32F103C8_SPI_DRIVER_H_
#define INC_STM32F103C8_SPI_DRIVER_H_

#include "stm32f103c8.h"

typedef enum {
	SLAVE = 0,
	MASTER = 1
}SpiDeviceMode;

typedef enum {
	DF_8_BIT_ = 0,
	DF_16_BIT = 1
}SpiDataFrameFormat;

typedef enum {
  SPI_MODE_0 = 0,  // CPOL: 0 CPHA: 0
  SPI_MODE_1,      // CPOL: 0 CPHA: 1
  SPI_MODE_2,      // CPOL: 1 CPHA: 0
  SPI_MODE_3,      // CPOL: 1 CPHA: 1
} SpiMode;

typedef enum {
	SSM_DISABLED = 0,
	SSM_ENABLED
}SpiSoftwareSlaveManagement;

typedef struct {
	SpiDeviceMode device_mode;
	SpiDataFrameFormat format;
	SpiMode mode;
	SpiSoftwareSlaveManagement ssm;
}SpiSettings;

typedef struct {
	SpiRegDef *address;
	SpiSettings settings;
}SpiHandler;

#endif /* INC_STM32F103C8_SPI_DRIVER_H_ */
