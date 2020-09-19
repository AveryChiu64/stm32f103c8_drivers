#ifndef INC_STM32F103C8_USART_DRIVER_H_
#define INC_STM32F103C8_USART_DRIVER_H_

#include "stm32f103c8.h"

#define USART_PARITY_ERROR_FLAG (1 << USRAT_PE)
#define USART_FRAMING_ERROR_FLAG (1 << USRAT_FE)
#define USART_NOISE_ERROR_FLAG (1 << USRAT_NE)
#define USART_OVERRUN_ERROR_FLAG (1 << USRAT_ORE)
#define USART_IDLE_FLAG (1 << USRAT_IDLE)
#define USART_RXNE_FLAG (1 << USRAT_RXNE)
#define USART_TC_FLAG (1 << USRAT_TC)
#define USART_TXE_FLAG (1 << USRAT_TXE)
#define USART_LBD_FLAG (1 << USRAT_LBD)
#define USART_CTS_FLAG (1 << USRAT_CTS)


#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

typedef enum {
	USART_MODE_TX = 0,
	USART_MODE_RX,
	USART_MODE_TX_RX,
}UsartMode;

typedef enum {
	USART_PARITY_DISABLE = 0,
	USART_PARITY_EN_EVEN,
	USART_PARITY_EN_ODD,
}UsartParity;

typedef enum {
	USART_WORDLEN_8BITS = 0,
	USART_WORDLEN_9BITS
}UsartWordLength;

typedef enum {
	USART_STOPBITS_1 = 0,
	USART_STOPBITS_0_5,
	USART_STOPBITS_2,
	USART_STOPBITS_1_5
}UsartStopbits;

typedef enum {
	USART_HW_FLOW_CTRL_NONE = 0,
	USART_HW_FLOW_CTRL_CTS,
	USART_HW_FLOW_CTRL_RTS,
	USART_HW_FLOW_CTRL_CTS_RTS
}UsartHwFlowCtrl;

typedef struct {
	UsartMode mode;
	UsartParity parity;
	UsartWordLength word_length;
	UsartStopbits stop_bits;
	UsartHwFlowCtrl flow_ctrl;
	uint32_t baud_rate;
}UsartSettings;

typedef struct {
	UsartRegDef *address;
	UsartSettings settings;
}UsartHandler;

void usart_peri_clock_ctrl(UsartRegDef *address, uint8_t en_or_di);

// Init and De-init
void usart_init(UsartHandler *handler);
void usart_deinit(UsartRegDef *address);

// Data Send and Receive
void usart_tx(UsartRegDef *address,uint8_t *tx_buffer, uint32_t len);
void usrat_rx(UsartRegDef *address, uint8_t *rx_buffer, uint32_t len);
uint8_t usart_tx_it(UsartHandler *handler,uint8_t *tx_buffer, uint32_t len);
uint8_t usart_rx_it(UsartHandler *handler, uint8_t *rx_buffer, uint32_t len);

// IRQ Handling
void usart_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void usart_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);
void usart_irq_handling(UsartHandler *handler);

// Other Peripheral Control APIs
void usart_peripheral_control(UsartRegDef *address, uint8_t en_or_di);
uint8_t usart_get_flag_status(UsartRegDef *address , uint32_t flag_name);
void usart_clear_flag(UsartRegDef *address, uint16_t status_flag_name);

// Application callback
void usart_application_event_callback(UsartHandler *handler,uint8_t app_event);

#endif /* INC_STM32F103C8_USART_DRIVER_H_ */
