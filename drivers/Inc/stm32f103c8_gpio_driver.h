#ifndef INC_STM32F103C8_GPIO_DRIVER_H_
#define INC_STM32F103C8_GPIO_DRIVER_H_

#include "stm32f103c8.h"

#define NUM_PINS 15
#define MAX_PIN_NUM_CRL 7

typedef struct {
	GpioRegDef *port;
	uint8_t pin;
} GpioAddress;

typedef enum {
	GPIO_STATE_LOW = 0, GPIO_STATE_HIGH,
} GpioState;

typedef enum {
  INTERRUPT_EDGE_RISING = 0,
  INTERRUPT_EDGE_FALLING,
  INTERRUPT_EDGE_RISING_FALLING,
} GpioInterruptEdge;

typedef enum {
	ANALOG_INPUT = 0,
	FLOATING_INPUT,
	PUPD_INPUT,
	PUSH_PULL_OUTPUT = 0,
	OPEN_DRAIN_OUTPUT,
	ALTFN_PUSH_PULL_OUTPUT,
	ALTFN_OPEN_DRAIN_OUTPUT,
} GpioType;

typedef enum {
	INPUT_MODE = 0, OUTPUT_MODE_10MHZ, OUTPUT_MODE_2MHZ, OUTPUT_MODE_50MHZ,
} GpioMode;

typedef enum {
	INPUT_PULL_DOWN = 0, INPUT_PULL_UP
} GpioRes;

typedef struct {
	GpioMode mode;
	GpioType type;
	GpioRes pupd;
	GpioInterruptEdge edge;
} GpioSettings;

//Peripheral Clock Setup
void gpio_peri_clock_ctrl(GpioRegDef *port, uint8_t en_or_di);

//Initialization
void gpio_init(GpioAddress *address, GpioSettings *settings);
void gpio_deinit(GpioRegDef *port);

//Read and Write
GpioState gpio_read_pin(GpioAddress *address);
uint16_t gpio_read_port(GpioRegDef *port);
void gpio_write_pin(GpioAddress *address, GpioState state);
void gpio_write_port(GpioRegDef *port, uint16_t value);
void gpio_toggle_pin(GpioAddress *address);

// IRQ Configuration and ISR Handling
void gpio_irq_config(uint8_t irq_numbeer, uint8_t irq_priority,
		uint8_t en_or_di);
void gpio_irq_handling(GpioAddress *address);

#endif /* INC_STM32F103C8_GPIO_DRIVER_H_ */
