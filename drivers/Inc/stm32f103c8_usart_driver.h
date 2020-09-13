#ifndef INC_STM32F103C8_USART_DRIVER_H_
#define INC_STM32F103C8_USART_DRIVER_H_

#include "stm32f103c8.h"

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


#endif /* INC_STM32F103C8_USART_DRIVER_H_ */
