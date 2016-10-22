#include "HardwarePlatform.h"

/* ======================================================= */
void HardwarePlatform::initIO() {
	_out(csn_pin, *direction_register);
	_out(ce_pin, *direction_register);
}

void HardwarePlatform::initSPI() {
	_out(DDB5, DDRB); // SCK
	_out(DDB3, DDRB); // MOSI
	_in(DDB4, DDRB);  // MISO
	_out(DDB2, DDRB); // CE

	/* Enable SPI, Master, set clock rate fck/2 */
	SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);
	SPSR |= (1<<SPI2X);
}

void HardwarePlatform::csn(uint8_t value) {
	if (value) {
		_on(csn_pin, *out_register);
	} else {
		_off(csn_pin, *out_register);
	}
}

void HardwarePlatform::ce(uint8_t value) {
	if (value) {
		_on(ce_pin, *out_register);
	} else {
		_off(ce_pin, *out_register);
	}
}

uint8_t HardwarePlatform::spiTransfer(uint8_t tx_) {
	/* Start transmission */
	SPDR = tx_;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return data register */
	return SPDR;
}

void HardwarePlatform::delayMicroseconds(uint64_t micros) {
	_delay_us(micros);
}

void HardwarePlatform::delayMilliseconds(uint64_t milisec) {
	_delay_ms(milisec);
}
