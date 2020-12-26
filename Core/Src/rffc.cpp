/*
 * rffc.cpp
 *
 *  Created on: Dec 13, 2020
 *      Author: aronno
 */
#include "rffc.h"
#include "delay.h"

/*
#define RFFC5071_SCL()	delay_us(RFFC5071_FSCL, &htim2);\
						gpio_set_pin_high(chip->scl);\
						delay_us(RFFC5071_FSCL, &htim2);\
						gpio_set_pin_low(chip->scl)
*/

#define RFFC5071_SCL()	delay_us(RFFC5071_FSCL, &htim2);\
						HAL_GPIO_WritePin(Port_sclk, Pin_sclk, GPIO_PIN_SET);\
						delay_us(RFFC5071_FSCL, &htim2);\
						HAL_GPIO_WritePin(Port_sclk, Pin_sclk, GPIO_PIN_RESET)

extern TIM_HandleTypeDef htim2;

RFFC::RFFC(GPIO_TypeDef *port_sclk, uint16_t pin_sclk, GPIO_TypeDef *port_sdata, uint16_t pin_sdata, GPIO_TypeDef *port_resetx, uint16_t pin_resetx, GPIO_TypeDef *port_enx, uint16_t pin_enx)
{
	Port_sclk = port_sclk;
	Pin_sclk = pin_sclk;
	Port_sdata = port_sdata;
	Pin_sdata = pin_sdata;
	Port_resetx = port_resetx;
	Pin_resetx = pin_resetx;
	Port_enx = port_enx;
	Pin_enx = pin_enx;
}

void RFFC::waitGPIO4IsHigh()
{
	uint8_t pinState = GPIO_PIN_RESET;

	while(pinState == GPIO_PIN_RESET)
	{
		pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	}
}

uint16_t RFFC::read(uint8_t reg)
{

	return 0;
}
