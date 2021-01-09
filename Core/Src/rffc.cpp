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

void RFFC::resetHigh()
{
	HAL_GPIO_WritePin(Port_resetx, Pin_resetx, GPIO_PIN_SET);
}

void RFFC::resetLow()
{
	HAL_GPIO_WritePin(Port_resetx, Pin_resetx, GPIO_PIN_RESET);
}

void RFFC::waitGPIO4IsHigh(GPIO_TypeDef *port, uint16_t pin)
{
	uint8_t pinState = GPIO_PIN_RESET;

	while(pinState == GPIO_PIN_RESET)
	{
		pinState = HAL_GPIO_ReadPin(port, pin);
	}
}

void RFFC::write(uint8_t reg, uint16_t val)
{
	/*
	uint8_t i = 0;
	uint8_t rmask = 0x80;
	uint16_t vmask = 0x8000;
	//two clocks before enx goes low, undocumented
	RFFC5071_SCL();
	RFFC5071_SCL();
	//gpio_set_pin_low(chip->enx);
	HAL_GPIO_WritePin(Port_enx, Pin_enx, GPIO_PIN_RESET);
	//clock out the undefined bit, set sda = 0
	//gpio_set_pin_low(chip->sda);
	HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_RESET);
	RFFC5071_SCL();
	//reg's MSB is zero already, and will be sent as W bit
	for(i=0; i<8; i++)
	{
		if(rmask & reg)
		{
			//gpio_set_pin_high(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_SET);
		}
		else
		{
			//gpio_set_pin_low(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_RESET);
		}
		RFFC5071_SCL();
		rmask >>= 1;
	}
	//now the value
	for(i=0; i<16; i++)
	{
		if(vmask & val)
		{
			//gpio_set_pin_high(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_SET);
		}
		else
		{
			//gpio_set_pin_low(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_RESET);
		}
		RFFC5071_SCL();
		vmask >>= 1;
	}
	//pull enx high again
	gpio_set_pin_high(chip->enx);
	//one clock after enx goes high, undocumented
	RFFC5071_SCL();
	*/
}

uint16_t RFFC::read(uint8_t reg)
{
	uint8_t i = 0;
	uint8_t rmask = 0x80;
	uint16_t vmask = 0x8000;
	uint16_t val = 0;
	uint8_t ireg = 0x80 | reg;

	/*
	 * As described in
	 * integrated_synthesizer_mixer_register_map_programming_guide.pdf Page-6 2.2.1 Three-Wire Bus Read Operation
	 *
	 * Also from https://github.com/mossmann/hackrf/blob/master/firmware/common/rffc5071_spi.c
	 * Line 128
	 *
	 * SPI register read.
	 *
	 * Send 9 bits:
	 *   first bit is ignored,
	 *   second bit is one for read operation,
	 *   next 7 bits are register address.
	 * Then receive 16 bits (register value).
	 *
	 * At Line 153 it states that:
	 * The device requires two clocks while ENX is high before a serial
	 * transaction.  This is not clearly documented.
	 *
	 *
	 */

	//two clocks before enx goes low, undocumented
	RFFC5071_SCL();
	RFFC5071_SCL();

	//gpio_set_pin_low(chip->enx);
	HAL_GPIO_WritePin(Port_enx, Pin_enx, GPIO_PIN_RESET);

	//clock out the undefined bit, set sda = 0
	//gpio_set_pin_low(chip->sda);
	HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_SET);

	RFFC5071_SCL();

	//ireg is reg with R bit set
	for(i=0; i<8; i++)
	{
		if(rmask & ireg)
		{
			//gpio_set_pin_high(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_SET);
		}
		else
		{
			//gpio_set_pin_low(chip->sda);
			HAL_GPIO_WritePin(Port_sdata, Pin_sdata, GPIO_PIN_RESET);
		}
		RFFC5071_SCL();
		rmask >>= 1;
	}

	//1.5 clocks delay
	RFFC5071_SCL();
	//set SDA to input for a while
	//ioport_set_pin_dir(chip->sda,IOPORT_DIR_INPUT);
	changePinDirection(Port_sdata, Pin_sdata, 0); //0 for input
	//now the value
	for(i=0; i<16; i++)
	{
		RFFC5071_SCL();
		//if(gpio_pin_is_high(chip->sda))
		if(HAL_GPIO_ReadPin(Port_sdata, Pin_sdata) == GPIO_PIN_SET)
		{
			val |= vmask;
		}
		vmask >>= 1;
	}

	HAL_GPIO_WritePin(Port_enx, Pin_enx, GPIO_PIN_SET);
	//pull enx high again
	RFFC5071_SCL();
	RFFC5071_SCL();

	changePinDirection(Port_sdata, Pin_sdata, 1); //1 for output

	return val;
}

//#define PIN_MODE_OUTPUT	1
//#define PIN_MODE_INPUT	0
void RFFC::changePinDirection(GPIO_TypeDef *port, uint16_t pin, uint16_t dir)
{
	if(dir == PIN_MODE_OUTPUT)
	{
		port->MODER |= (0b1001<<pin); // GPIO_MODER_MODE8_0;

	}
	else if(dir == PIN_MODE_INPUT)
	{
		port->MODER &= ~(0b0111<<pin);
	}
}
