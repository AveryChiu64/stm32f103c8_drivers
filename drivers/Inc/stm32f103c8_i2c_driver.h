#ifndef INC_STM32F103C8_I2C_DRIVER_H_
#define INC_STM32F103C8_I2C_DRIVER_H_

#include "stm32f103c8.h"

// Flags
#define I2C_SB_FLAG (1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG (1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG (1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG (1 << I2C_SR1_STOPF)
#define I2C_RXNE_FLAG (1 << I2C_SR1_RXNE)
#define I2C_TXE_FLAG (1 << I2C_SR1_TXE)
#define I2C_BERR_FLAG (1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG (1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG (1 << I2C_SR1_AF)
#define I2C_OVR_FLAG (1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG (1 << I2C_SR1_TIMEOUT)

// I2C scl speeds
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000

typedef enum {
	ACK_DISABLE = 0, ACK_ENABLE
} I2CAck;

typedef enum {
	FM_DUTY_2 = 0, FM_DUTY_16_9
} I2CDutyCycle;

typedef struct {
	uint32_t scl_speed;
	uint8_t device_address;
	I2CAck ack;
	I2CDutyCycle duty_cycle;
} I2CSettings;

typedef struct {
	I2CRegDef *address;
	I2CSettings settings;
} I2CHandler;

//Peripheral Clock Setup
void i2c_peri_clock_ctrl(I2CRegDef *address, uint8_t en_or_di);

//Initialization
void i2c_init(I2CHandler *handler);

// Tx and Rx
void i2c_master_tx(I2CHandler *handler, uint8_t *tx_buffer, uint32_t len,
		uint8_t slave_address);

//Other peripheral control
void i2c_peripheral_control(I2CRegDef *address, uint8_t en_or_di);
uint8_t i2c_get_flag_status(I2CRegDef *address, uint32_t flag_name);

// IRQ Handling
void i2c_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_di);
void i2c_irq_priority_config(uint8_t irq_number, NvicIrqPriority irq_priority);

// Application Callback
void i2c_application_event_callback(I2CHandler *handler,
		uint8_t application_event);

#endif /* INC_STM32F103C8_I2C_DRIVER_H_ */
