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

class RFFC
{
public:
	RFFC(GPIO_TypeDef *port, uint16_t pin);
	void write(uint8_t reg, uint16_t val);
	uint16_t read(uint8_t reg);
	void set_freq(uint64_t freq);
	void waitGPIO4IsHigh();
};



#endif /* INC_RFFC_H_ */
