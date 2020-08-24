#ifndef INC_STM32F103C8_GPIO_DRIVER_H_
#define INC_STM32F103C8_GPIO_DRIVER_H_

#include "stm32f103c8.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	// Pointer to hold base address of the GPIO Peripheral
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

//Initialization
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Configuration and ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F103C8_GPIO_DRIVER_H_ */
