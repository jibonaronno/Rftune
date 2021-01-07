# STM32L496ZGTx C++ Code Base.
<img alt="NO IMAGE" src="STM32L496.png"><br>

Because of C++ incompatibility some extra code part is needed to apply. <br /> 
In main.cpp modify the function definition as :<br />
```c
extern "C" void SystemClock_Config(void);
```
<img alt="NO IMAGE" src="SystemClock_Config.png"><br>
But this changes is not required in case of STM32F103C8T6 . Do not know why.<br />
___

### Class design RFFC

There are some issues about RFFC5071 chip communication. According to some codes from web:

```c
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
```

Adding support for both RFFC chip.
