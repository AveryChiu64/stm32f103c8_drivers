#ifndef INC_STM32F103C8_USART_DRIVER_H_
#define INC_STM32F103C8_USART_DRIVER_H_

#include "stm32f103c8.h"

typedef struct {

}UsartSettings;

typedef struct {
	UsartRegDef *address;
	UsartSettings settings;
}UsartHandler;


#endif /* INC_STM32F103C8_USART_DRIVER_H_ */
