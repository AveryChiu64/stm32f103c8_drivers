#ifndef INC_STM32F103C8_H_
#define INC_STM32F103C8_H_

#include <stdint.h>
#include <stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))

// ARM Cortex M3 Processor NVIC Register Addresses
#define NVIC_ISER_BASEADDR ((__vo uint32_t*)0XE000E100)
#define NVIC_ICER_BASEADDR ((__vo uint32_t*)0XE000E180)
#define NVIC_IPR_BASEADDR ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

// Base Addresses of Flash and SRAM memories
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define ROM_BASEADDR 0x1FFFF000U // system memory
#define SRAM SRAM1_BASEADDR

// AHB and APBx Bus Peripheral base addresses
#define PERIPH_BASE 0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHBPERIPH_BASE 0x40018000U

// Base Addresses of peripherals hanging on APB1 bus
#define SPI2_BASEADDR (APB1PERIPH_BASE + 0X3C00)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0X3800)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0X5800)

// Base Addresses of peripherals hanging on APB2 bus
#define AFIO_BASEADDR APB2PERIPH_BASE
#define EXTI_BASEADDR  (APB2PERIPH_BASE + 0x0400)

#define GPIOA_BASEADDR (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR (APB2PERIPH_BASE + 0x2000)

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0x3800)

// Base Addresses of peripherals hanging on AHB bus
#define RCC_BASEADDR (AHBPERIPH_BASE + 0x9000)

// Peripheral Register Definition Structures
typedef struct {
	__vo uint32_t CR; // Clock control register
	__vo uint32_t CFGR; // Clock configuration register
	__vo uint32_t CIR; // Clock interrupt register
	__vo uint32_t APB2RSTR; // APB2 peripheral reset register
	__vo uint32_t APB1RSTR; // APB1 peripheral reset register
	__vo uint32_t AHBENR; // AHB peripheral clock enable register
	__vo uint32_t APB2ENR; // APB2 peripheral clock enable register
	__vo uint32_t APB1ENR; // APB1 peripheral clock enable register
	__vo uint32_t BDCR; // Backup domain control register
	__vo uint32_t CSR; // Control/status register
} RccRegDef;

typedef struct {
	__vo uint32_t CR[2]; // Port configuration register
	__vo uint32_t IDR; // Port input data register
	__vo uint32_t ODR; // Port output data register
	__vo uint32_t BSRR; // Port bit set/reset register
	__vo uint32_t BRR; // Port bit reset register
	__vo uint32_t LCKR; // Port configuration lock register
} GpioRegDef;

typedef struct {
	__vo uint32_t EVCR; // Event control register
	__vo uint32_t MAPR; // AF remap and debug I/O configuration register
	__vo uint32_t EXTICR[4]; // External interrupt configuration register
	__vo uint32_t MAPR2;
} AfioRegDef;

typedef struct {
	__vo uint32_t CR1; // control registers
	__vo uint32_t CR2;
	__vo uint32_t SR; // status register
	__vo uint32_t DR; // data register
	__vo uint32_t CRCPR; //CRC polynomial register
	__vo uint32_t RXCRCR; // RX CRC register
	__vo uint32_t TXCRCR; // TX CRC register
	__vo uint32_t I2SCFGR; // I2S configuration register
	__vo uint32_t I2SPR; // I2S prescaler register
}SpiRegDef;

typedef struct {
	__vo uint32_t CR1; // control registers
	__vo uint32_t CR2;
	__vo uint32_t OAR1; // own address registers
	__vo uint32_t OAR2;
	__vo uint32_t DR; // data register
	__vo uint32_t SR1; // status register
	__vo uint32_t SR2;
	__vo uint32_t CCR; // clock control register
	__vo uint32_t TRISE; // TRISE register
}I2CRegDef;

typedef struct {
	__vo uint32_t IMR; // Interrupt mask register
	__vo uint32_t EMR; // Event mask register
	__vo uint32_t RTSR; // Rising trigger selection register
	__vo uint32_t FTSR; // Falling trigger selection register
	__vo uint32_t SWIER; // Software interrupt event register
	__vo uint32_t PR; // Pending register
} ExtiRegDef;

// Bit position definitions for SPI registers
typedef enum {
	SPI_CR1_CPHA = 0,
	SPI_CR1_CPOL,
	SPI_CR1_MSTR,
	SPI_CR1_BR1,
	SPI_CR1_BR2,
	SPI_CR1_BR3,
	SPI_CR1_SPE,
	SPI_CR1_LSB_FIRST,
	SPI_CR1_SSI,
	SPI_CR1_SSM,
	SPI_CR1_RX_ONLY,
	SPI_CR1_DFF,
	SPI_CR1_CRC_NEXT,
	SPI_CR1_CRC_EN,
	SPI_CR1_BIDI_OE,
	SPI_CR1_BIDI_MODE
}SpiControlRegister1;

typedef enum {
	SPI_CR2_RXDMAEN = 0,
	SPI_CR2_TXDMAEN,
	SPI_CR2_SSOE,
	SPI_CR2_ERRIE = 5,
	SPI_CR2_RXNEIE,
	SPI_CR2_TXEIE,
}SpiControlRegister2;

typedef enum {
	SPI_SR_RXNE = 0,
	SPI_SR_TXE,
	SPI_SR_CHSIDE,
	SPI_SR_UDR,
	SPI_SR_CRC_ERR,
	SPI_SR_MODF,
	SPI_SR_OVR,
	SPI_SR_BSY,
}SpiStatusRegister;

// Bit position definitions for I2C registers
typedef enum {
 I2C_CR1_PE = 0,
 I2C_CR1_SMBUS,
 I2C_CR1_SMB_TYPE = 3,
 I2C_CR1_ENARP,
 I2C_CR1_ENPEC,
 I2C_CR1_ENGC,
 I2C_CR1_NO_STRETCH,
 I2C_CR1_START,
 I2C_CR1_STOP,
 I2C_CR1_ACK,
 I2C_CR1_POS,
 I2C_CR1_PEC,
 I2C_CR1_ALERT,
 I2C_CR1_SWRST = 15
}I2CControlRegister1;

typedef enum {
	I2C_CR2_FREQ0=0,
	I2C_CR2_FREQ1,
	I2C_CR2_FREQ2,
	I2C_CR2_FREQ3,
	I2C_CR2_FREQ4,
	I2C_CR2_FREQ5,
	I2C_CR2_ITERREN = 8,
	I2C_CR2_ITEVTEN,
	I2C_CR2_ITBUFEN,
	I2C_CR2_DMAEN,
	I2C_CR2_LAST
}I2CControlRegister2;

typedef enum {
	I2C_SR1_SB = 0,
	I2C_SR1_ADDR,
	I2C_SR1_BTF,
	I2C_SR1_ADD10,
	I2C_SR1_STOPF,
	I2C_SR1_RXNE = 6,
	I2C_SR1_TXE,
	I2C_SR1_BERR,
	I2C_SR1_ARLO,
	I2C_SR1_AF,
	I2C_SR1_OVR,
	I2C_SR1_PEC_ERR,
	I2C_SR1_TIMEOUT = 14,
	I2C_SR1_SMB_ALERT
}I2CStatusRegister1;

typedef enum {
	I2C_SR2_MSL = 0,
	I2C_SR2_BUSY,
	I2C_SR2_TRA,
	I2C_SR2_GEN_CALL = 4,
	I2C_SR2_SMBDE_FAULT,
	I2C_SR2_SMB_HOST,
	I2C_SR2_DUALF,
	I2C_SR2_PEC0,
	I2C_SR2_PEC1,
	I2C_SR2_PEC2,
	I2C_SR2_PEC3,
	I2C_SR2_PEC4,
	I2C_SR2_PEC5,
	I2C_SR2_PEC6,
	I2C_SR2_PEC7
}I2CStatusRegister2;

typedef enum {
	I2C_CCR0 = 0,
	I2C_CCR1,
	I2C_CCR2,
	I2C_CCR3,
	I2C_CCR4,
	I2C_CCR5,
	I2C_CCR6,
	I2C_CCR7,
	I2C_CCR8,
	I2C_CCR9,
	I2C_CCR10,
	I2C_CCR11,
	I2C_CCR_DUTY = 14,
	I2C_CCR_FS
}I2CClockControlRegister;

// Peripheral Definitions
#define GPIOA ((GpioRegDef*)GPIOA_BASEADDR)
#define GPIOB ((GpioRegDef*)GPIOB_BASEADDR)
#define GPIOC ((GpioRegDef*)GPIOC_BASEADDR)
#define GPIOD ((GpioRegDef*)GPIOD_BASEADDR)
#define GPIOE ((GpioRegDef*)GPIOE_BASEADDR)
#define GPIOF ((GpioRegDef*)GPIOF_BASEADDR)
#define GPIOG ((GpioRegDef*)GPIOG_BASEADDR)

#define SPI1 ((SpiRegDef*)SPI1_BASEADDR)
#define SPI2 ((SpiRegDef*)SPI2_BASEADDR)
#define SPI3 ((SpiRegDef*)SPI3_BASEADDR)

#define I2C1 ((I2CRegDef*)I2C1_BASEADDR)
#define I2C2 ((I2CRegDef*)I2C2_BASEADDR)

#define RCC ((RccRegDef*)RCC_BASEADDR)
#define AFIO ((AfioRegDef*)AFIO_BASEADDR)
#define EXTI ((ExtiRegDef*)EXTI_BASEADDR)

// Clock Enable Macro for AFIO
#define AFIO_PCLK_EN() (RCC->APB2ENR |= (1 << 0))

// Clock Enable Macros for GPIOx Peripherals
#define GPIOA_PCLK_EN() (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN() (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN() (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN() (RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN() (RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN() (RCC->APB2ENR |= (1 << 8))

// Clock Enable Macros for I2Cx Peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))

// Clock Enable Macros for SPIx Peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

// Clock Enable Macros for USARTx Peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))

// Clock Enable Macro for AFIO
#define AFIO_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))

// Clock Disable Macros for GPIOx Peripherals
#define GPIOA_PCLK_DI() (RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI() (RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI() (RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI() (RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 8))

// Clock Disable Macros for I2Cx Peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))

// Clock Disable Macros for SPIx Peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))

// Clock Enable Macros for USARTx Peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))

// Macros for resetting GPIOx Peripherals
#define GPIOA_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); } while(0)
#define GPIOB_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); } while(0)
#define GPIOC_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); } while(0)
#define GPIOD_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); } while(0)
#define GPIOE_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6)); } while(0)
#define GPIOF_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7)); } while(0)
#define GPIOG_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8)); } while(0)

// Returns port for given GPIOx base address
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : \
								 (x == GPIOB) ? 1 : \
								 (x == GPIOC) ? 2 : \
							     (x == GPIOD) ? 3 : \
								 (x == GPIOE) ? 4 : \
								 (x == GPIOF) ? 5 : \
								 (x == GPIOG) ? 6 : 0)

// IRQ Numbers for STM32F103C8
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51

typedef enum {
	NVIC_IRQ_PRIORITY0 = 0,
	NVIC_IRQ_PRIORITY1,
	NVIC_IRQ_PRIORITY2,
	NVIC_IRQ_PRIORITY3,
	NVIC_IRQ_PRIORITY4,
	NVIC_IRQ_PRIORITY5,
	NVIC_IRQ_PRIORITY6,
	NVIC_IRQ_PRIORITY7,
	NVIC_IRQ_PRIORITY8,
	NVIC_IRQ_PRIORITY9,
	NVIC_IRQ_PRIORITY10,
	NVIC_IRQ_PRIORITY11,
	NVIC_IRQ_PRIORITY12,
	NVIC_IRQ_PRIORITY13,
	NVIC_IRQ_PRIORITY14,
	NVIC_IRQ_PRIORITY15,
}NvicIrqPriority;

// Generic Macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_RESET	RESET
#define FLAG_SET	SET

// Drivers
#include "stm32f103c8_gpio_driver.h"
#include "stm32f103c8_spi_driver.h"
#include "stm32f103c8_i2c_driver.h"

#endif /* INC_STM32F103C8_H_ */
