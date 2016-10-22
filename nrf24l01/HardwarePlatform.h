/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "atmega328.h"
#include <string.h>

/* ============================================== */
//#define IF_SERIAL_DEBUG(x) x
#define IF_SERIAL_DEBUG(x)
#define IF_WRITE_DEBUG(x)
#define printf_P printf

/* ============================================== */
class HardwarePlatform {

private:
	volatile uint8_t * direction_register;
	volatile uint8_t * out_register;
	volatile uint8_t csn_pin;
	volatile uint8_t ce_pin;

public:
	HardwarePlatform(volatile uint8_t * direction_r,
			         volatile uint8_t * out_r,
					 uint8_t csn_p,
					 uint8_t ce_p) :
						 direction_register(direction_r),
						 out_register(out_r),
						 csn_pin(csn_p),
						 ce_pin(ce_p) {
	}

	void initIO();
	void initSPI();
	void csn(uint8_t value);
	void ce(uint8_t value);
	uint8_t spiTransfer(uint8_t tx_);
	void delayMicroseconds(uint64_t micros);
	void delayMilliseconds(uint64_t milisec);

};
