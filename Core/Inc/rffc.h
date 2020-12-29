/*
 * rffc.h
 *
 *  Created on: Dec 13, 2020
 *      Author: aronno
 */

#ifndef INC_RFFC_H_
#define INC_RFFC_H_

#define RFFC5071_FSCL 10	//in us

//register definitions that we will use
#define RFFC5071_LF 0x00
#define RFFC5071_PLL_CAL2 0x07
#define RFFC5071_MIX_CONT 0x11
#define RFFC5071_VCO_AUTO 0x08
#define RFFC5071_PLL_CTRL 0x09
#define RFFC5071_P1_FREQ1 0x0C
#define RFFC5071_P1_FREQ2 0x0D
#define RFFC5071_P1_FREQ3 0x0D
#define RFFC5071_P1_FREQ4 0x0E
#define RFFC5071_P2_FREQ1 0x0F
#define RFFC5071_P2_FREQ2 0x10
#define RFFC5071_P2_FREQ3 0x11
#define RFFC5071_GPO 0x16
#define RFFC5071_DEV_CTRL 0x1D
#define RFFC5071_SDI_CTRL 0x15

#include <stdint.h>
#include "main.h"

typedef enum
{
DIR_INPUT,
DIR_OUTPUT
}Data_Direction;

#define PIN_MODE_OUTPUT	1
#define PIN_MODE_INPUT	0

class RFFC
{
public:
	RFFC(GPIO_TypeDef *port_sclk, uint16_t pin_sclk, GPIO_TypeDef *port_sdata, uint16_t pin_sdata, GPIO_TypeDef *port_resetx, uint16_t pin_resetx, GPIO_TypeDef *port_enx, uint16_t pin_enx);
	void write(uint8_t reg, uint16_t val);
	uint16_t read(uint8_t reg);
	void set_freq(uint64_t freq);
	void waitGPIO4IsHigh(GPIO_TypeDef *port, uint16_t pin);

	GPIO_TypeDef *Port_sclk;
	uint16_t Pin_sclk;
	GPIO_TypeDef *Port_sdata;
	uint16_t Pin_sdata;
	GPIO_TypeDef *Port_resetx;
	uint16_t Pin_resetx;
	GPIO_TypeDef *Port_enx;
	uint16_t Pin_enx;

private:
	//#define PIN_MODE_OUTPUT	1
	//#define PIN_MODE_INPUT	0
	void changePinDirection(GPIO_TypeDef *port, uint16_t pin, uint16_t dir);
};



#endif /* INC_RFFC_H_ */
