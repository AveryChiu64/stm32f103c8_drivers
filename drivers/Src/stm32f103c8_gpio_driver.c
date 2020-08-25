#include "stm32f103c8_gpio_driver.h"

/*******************************************************************
 * NAME : GPIO_PeriClockControl
 *
 * DESCRIPTION : Enables or dsiables the peripheral clock
 *
 *PARAMETERS:	GPIO_RegDef_t 	*pGPIOx 		Address of GPIO Port
 * 			  	uint8_t		 	 EnorDi			ENABLE or DISABLE macros
 *
 * OUTPUTS : void
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
	}

	/*******************************************************************
	 * NAME : GPIO_Init
	 *
	 * DESCRIPTION : Initializes GPIO
	 *
	 *PARAMETERS:	GPIO_Handle_t* 	pGPIOHandle		Holds port and pin including its settings
	 *
	 * OUTPUTS : void
	 */
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

	/*******************************************************************
	 * NAME : GPIO_DeInit
	 *
	 * DESCRIPTION : Deinitializes GPIO
	 *
	 *PARAMETERS:	GPIO_RegDef_t 	*pGPIOx 		Address of GPIO Port to reset
	 *
	 * OUTPUTS : void
	 */
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

	/*******************************************************************
	 * NAME : GPIO_DeInit
	 *
	 * DESCRIPTION : Deinitializes GPIO
	 *
	 *PARAMETERS:	GPIO_RegDef_t 	*pGPIOx 		Address of GPIO Port to reset
	 *				uint8_t 		PinNumber		The pin number for GPIO
	 *
	 * OUTPUTS : A uint8_t providing the state of the input pin
	 */
	uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

	uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
	void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
			uint8_t Value);
	void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
	void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

	void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
	void GPIO_IRQHandling(uint8_t PinNumber);
