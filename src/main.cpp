/********************************************************************************
	Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>

#include "../nrf24l01/RF24.h"
#include "../common/util.h"
#include "../nrf24l01/atmega328.h"
#include "../atmega328/mtimer.h"

extern "C" {
#include "../atmega328/usart.h"
}

/********************************************************************************
	Macros and Defines
********************************************************************************/

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initGPIO();
void initRxRadio();
void initTxRadio();
void readRxRadio();
void setRxChannel(uint8_t channel);
void setTxChannel(uint8_t channel);
uint16_t adc_read(uint8_t adcx);

/********************************************************************************
	Global Variables
********************************************************************************/
HardwarePlatform rxHP(&DDRB, &PORTB, PB1, PB0);
RF24 rxRadio(&rxHP);

HardwarePlatform txHP(&DDRC, &PORTC, PC4, PC5);
RF24 txRadio(&txHP);

const uint64_t pipe = 0xF0F0F0F0E1LL;

volatile bool rx_received = false;
volatile bool adc_enabled = false;
volatile uint8_t rx_channel = 10;
volatile uint8_t tx_channel = 10;

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(USART_RX_vect)
{
	handle_usart_interrupt();
}

ISR(TIMER1_OVF_vect)
{
	incrementOvf();
}

ISR(INT0_vect)
{
}

ISR(INT1_vect)
{
	rx_received = true;
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {
    // initialize usart module
	usart_init();

	initGPIO();

    initTimer();

    // enable interrupts
    sei();

    initTxRadio();

    initRxRadio();

	// Console friendly output
    printf(CONSOLE_PREFIX);

    log_info("\nINITIALIZED 000001");

	// main loop
    while (1) {
    	// ================= main usart loop for console ========================
    	usart_check_loop();

    	if (adc_enabled) {
    		uint16_t mux0Value = adc_read(MUX0);
    		printf("%d\n", mux0Value);

    	} else if (rx_received) {
    		readRxRadio();
    	    rx_received = false;
    	}

    }
}

/********************************************************************************
	Functions
********************************************************************************/
void initGPIO() {
    _in(DDD2, DDRD); // INT0 input
    _in(DDD3, DDRD); // INT1 input

    _in(DDC0, DDRC); // Analog input 0
    // _in(DDC1, DDRC); // Analog input 1

    // Enable the ADC
    ADCSRA |= _BV(ADEN);

    // GPIO Interrupt INT0 & INT1
    // The falling edge of INT0 & INT1 generates an interrupt request.
    EICRA = (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00);

    // Enable INT0 * INT1
    EIMSK = (1<<INT1)|(1<<INT0);
}

void initRxRadio() {
    log_info("*********** Initialize RX Radio ***********");
    log_info(" ");

    rxRadio.begin();
    rxRadio.setRetries(15, 15);
    rxRadio.setPayloadSize(8);
    rxRadio.setPALevel(RF24_PA_MAX);
    rxRadio.setChannel(rx_channel);

    rxRadio.openWritingPipe(pipe);
    rxRadio.openReadingPipe(1, pipe);

    rxRadio.startListening();

    rxRadio.printDetails();
}

void initTxRadio() {
    log_info("*********** Initialize TX Radio ***********");
    log_info(" ");

    txRadio.begin();
    txRadio.setRetries(15, 15);
    txRadio.setPayloadSize(8);
    txRadio.setPALevel(RF24_PA_MAX);
    txRadio.setChannel(tx_channel);

    txRadio.openWritingPipe(pipe);
    //txRadio.openReadingPipe(1, pipe);

    rxRadio.startListening();

    txRadio.printDetails();

    txRadio.stopListening();

    uint8_t data[1] = {0};
    txRadio.write(data, 1);
    txRadio.flush_tx();
}


void readRxRadio() {
    bool tx_ok, tx_fail, rx_ok;
    rxRadio.whatHappened(tx_ok, tx_fail, rx_ok);

    if (rx_ok) {
        uint8_t data[32];
        uint8_t len = rxRadio.getDynamicPayloadSize();
        rxRadio.read(data, len);
        rxRadio.flush_rx();

        log_inline("DATA,");

        for (uint8_t i = 0; i < len; i++) {
        	log_inline("%d,", data[i]);
        }

        log_inline("DATA\n");

    } else {
    	log_error("RX is not ok");
    }
}

void setRxChannel(uint8_t channel) {
	if (rx_channel != channel) {
		rx_channel = channel;
		rxRadio.setChannel(channel);
		rxRadio.stopListening();
		rxRadio.startListening();
	}
}

void setTxChannel(uint8_t channel) {
	if (tx_channel != channel) {
		tx_channel = channel;
		txRadio.setChannel(channel);
		txRadio.flush_tx();
	}
}

void handle_usart_cmd(char *cmd, char* args[], uint8_t arg_count) {
	if (strcmp(cmd, "test") == 0) {
		for (uint8_t i = 0; i < arg_count; i++) {
			printf("\n ARG[%d]=[%s]", i, args[i]);
		}
	}

	if (strcmp(cmd, "setRxChannel") == 0) {
		if (arg_count > 0) {
			uint8_t channel = (uint8_t) atoi(args[0]);
			setRxChannel(channel);
			log_info("Set RX Channel to %d", channel);
		} else {
			log_error("Not enough arguments for setRxChannel function");
		}
	}

	if (strcmp(cmd, "adc") == 0) {
		if (arg_count > 0) {
			uint8_t enabled = (uint8_t) atoi(args[0]);
			if (enabled) {
				adc_enabled = true;
			} else {
				adc_enabled = false;
			}
		} else {
			log_error("Not enough arguments for adc function");
		}
	}

	if (strcmp(cmd, "send") == 0) {
		if (arg_count > 1) {
			uint8_t channel = (uint8_t) atoi(args[0]);
			setTxChannel(channel);

			uint8_t data[arg_count - 1];

			for (uint8_t i = 1; i < arg_count; i++) {
				data[i - 1] = (uint8_t) atoi(args[i]);
			}

			bool result = txRadio.write(data, (arg_count - 1));
			txRadio.flush_tx();
			log_info("Sent %d bytes to %d channel.", (arg_count - 1), channel);
			if (!result) {
				log_error("sending unsuccessful");
			}
		} else {
			log_error("Not enough arguments for send function");
		}
	}

}

uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX = adcx;
	ADMUX |= (0<<REFS1)|(1<<REFS0)|(0<<ADLAR);

	_delay_us(300);

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}
