#ifndef INC_STM32F103C8_H_
#define INC_STM32F103C8_H_

#include <stdint.h>
#define __vo volatile

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
	__vo uint32_t CRL; // Port configuration register low
	__vo uint32_t CRH; // Port configuration register high
	__vo uint32_t IDR; // Port input data register
	__vo uint32_t ODR; // Port output data register
	__vo uint32_t BSRR; // Port bit set/reset register
	__vo uint32_t BRR; // Port bit reset register
	__vo uint32_t LCKR; // Port configuration lock register
} GpioRegDef;

typedef struct {
	__vo uint32_t EVCR; // Event control register
	__vo uint32_t MAPR; // AF remap and debug I/O configuration register
	__vo uint32_t EXTICR1; // External interrupt configuration register
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	__vo uint32_t MAPR2;
} AfioRegDef;

// Peripheral Definitions
#define GPIOA ((GpioRegDef*)GPIOA_BASEADDR)
#define GPIOB ((GpioRegDef*)GPIOB_BASEADDR)
#define GPIOC ((GpioRegDef*)GPIOC_BASEADDR)
#define GPIOD ((GpioRegDef*)GPIOD_BASEADDR)
#define GPIOE ((GpioRegDef*)GPIOE_BASEADDR)
#define GPIOF ((GpioRegDef*)GPIOF_BASEADDR)
#define GPIOG ((GpioRegDef*)GPIOG_BASEADDR)

#define RCC ((RccRegDef*)RCC_BASEADDR)

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

// Generic Macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

// Drivers
#include "stm32f103c8_gpio_driver.h"

#endif /* INC_STM32F103C8_H_ */
