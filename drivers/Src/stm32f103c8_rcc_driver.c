#include "stm32f103c8_rcc_driver.h"

uint16_t ahb_prescaler[8] = { 2, 4, 8, 16, 32, 64, 128, 256 };
uint16_t apb1_prescaler[4] = { 2, 4, 8, 16 };

uint32_t rcc_get_pclk1_value() {
	uint8_t hpre, ppre, ahb, apb1;
	uint32_t sysclk;

	uint8_t clksrc = ((RCC->CFGR >> 2) & 0x3);

	// HSI
	if (clksrc == 0) {
		sysclk = 16; //16 MHz
	}
	// HSE
	else if (clksrc == 1) {
		sysclk = 8; //8 MHz
	}
	// AHB
	hpre = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	if (hpre < 8) {
		ahb = 1;
	} else {
		ahb = ahb_prescaler[hpre - 8];
	}

	// APB1
	ppre = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0xF);
	if (ppre < 4) {
		apb1 = 1;
	} else {
		apb1 = apb1_prescaler[ppre - 4];
	}
	return (sysclk / ahb) / apb1;
}
