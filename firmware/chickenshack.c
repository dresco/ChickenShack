//  chickenshack.c
//  Chickenshack -- automatic chicken house door opener
//
// Copyright (c) 2009 - 2014 Jon Escombe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//
// Chickenshack.c -- automatic chicken house door opener
//
// 8.0 MHz internal oscillator, clocked to 7.3728MHz by calibration routine
// 32.768 kHz watch crystal - async timer2 & oscillator calibration
// timer2 - 5 sec - periodic wakeup (clocked from 32.768 crystal)
//
// PORT B0 - output - motor control - (timer1a output compare)
// PORT B1 - output - motor direction control - up
// PORT B2 - output - motor direction control - down
//
// PORT C0   - output - enable voltage dividers
// PORT C1   - output - radio sleep select
// PORT C2   - input  - radio CTS status
// PORT C4   - output - debug LED
// PORT C5   - input  - ADC5 is ambient light level voltage
// PORT ADC6 - input  - ADC6 is light threshold voltage (trim pot)
// PORT ADC7 - input  - ADC7 is battery voltage (via voltage divider)
//
// PORT D0 - input  - USART RX
// PORT D1 - output - USART TX
// PORT D2 - input  - Manual door down override switch
// PORT D3 - input  - Manual door up override switch
// PORT D4 - output - motor driver enable
// PORT D5 - input  - Door at bottom of travel
// PORT D6 - input  - Door at top of travel
// PORT D7 - input  - pulled low for serial debug output (every timer tick)
//

#include <stdio.h>											// Note: big hit for using sprintf (1.5K)
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "osccal.h"

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define nop() asm volatile("nop")

#define TICK_INTERVAL 5										// timer 2 fires every 5 seconds
#define PROCESSING_INTERVAL 300								// process main loop every 300 seconds (5 min)
#define MAX_TIMER_TICKS (PROCESSING_INTERVAL / TICK_INTERVAL)
#define ADC_SAMPLES 3										// number of ADC readings to average
#define USART_BAUDRATE 115200
#define USART_BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define DOOR_ACTION_TIMEOUT_UP		7						// FIXME: Perhaps change to 8 seconds for next update....
#define DOOR_ACTION_TIMEOUT_DOWN	5						// FIXME: ...but this has been dropped from 6 to 5 instead
#define DOOR_INTERVAL_ITERATIONS	96 						// Processing intervals must elapse between door events - 8hrs
#define DEBUG_ENABLED (!(PIND & (1 << 7)))

//#define TEMPERATURE_ENABLED									// Comment out to remove experimental temperature reporting
//#define TEMPERATURE_OFFSET			343						// Convert K to C, plus manually adjusted for offset
#define RUNTIME_OSCCAL											// Comment out to remove runtime OSCCAL calibration via 32kHz crystal

#define DAY		0											// time_of_day
#define NIGHT	1
#define NOTHING	0											// door_action
#define RAISE 	1
#define LOWER	2
#define TOP		0											// door_position
#define BOTTOM	1
#define UNKNOWN	2

volatile uint8_t  timer_tick, timer_interval;
volatile uint16_t counter;

// Disable the watchdog timer before entering main(). The atmega88 has 'new style' watchdog that
// remains running after reset, using the quickest prescaler (15ms timeout).
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

void ADCRead (uint16_t* chan5, uint16_t* chan6, uint16_t* chan7, uint16_t* chan8)
{
	static uint16_t average_adc5[ADC_SAMPLES];
	static uint16_t average_adc6[ADC_SAMPLES];
	static uint16_t average_adc7[ADC_SAMPLES];
#ifdef TEMPERATURE_ENABLED
	static uint16_t average_adc8[ADC_SAMPLES];
#endif

	static uint8_t	average_adc5_index = 0;
	static uint8_t	average_adc6_index = 0;
	static uint8_t	average_adc7_index = 0;
#ifdef TEMPERATURE_ENABLED
	static uint8_t	average_adc8_index = 0;
#endif

	uint16_t		average_adc_sum;
	uint16_t		average_adc_result;
	uint8_t			average_adc_count;

	PORTC |= (1 << 0);										// Assert C0 to enable resistor dividers

	ADCSRA |= (1 << ADEN);									// Enable the ADC

	// Channel 5
	ADMUX  &= ~(1 << MUX0);									// Clear MUX bits
	ADMUX  &= ~(1 << MUX1);									//
	ADMUX  &= ~(1 << MUX2);									//
	ADMUX  &= ~(1 << MUX3);									//

	ADMUX  |= (1 << MUX0);									// Set MUX0 - channel 5
	ADMUX  |= (1 << MUX2);									// Set MUX2 - channel 5
	ADCSRA |= (1 << ADSC);									// Start ADC conversion
	while(ADCSRA & (1<<ADSC));								// Wait for conversion to finish

	// average out the readings
	average_adc5[average_adc5_index++] = ADC;

	if (average_adc5_index >= ADC_SAMPLES)
		average_adc5_index = 0;

	average_adc_sum = 0;
	for (average_adc_count = 0; average_adc_count < ADC_SAMPLES;  average_adc_count++)
	   average_adc_sum = average_adc_sum + average_adc5[average_adc_count];

	average_adc_result = average_adc_sum / ADC_SAMPLES;

	*chan5 = average_adc_result;							// Save conversion result

	// Channel 6
	ADMUX  &= ~(1 << MUX0);									// Clear MUX bits
	ADMUX  &= ~(1 << MUX1);									//
	ADMUX  &= ~(1 << MUX2);									//
	ADMUX  &= ~(1 << MUX3);									//

	ADMUX  |= (1 << MUX1);									// Set MUX1 - channel 6
	ADMUX  |= (1 << MUX2);									// Set MUX2 - channel 6
	ADCSRA |= (1 << ADSC);									// Start ADC conversion
	while(ADCSRA & (1<<ADSC));								// Wait for conversion to finish

	// average out the readings
	average_adc6[average_adc6_index++] = ADC;

	if (average_adc6_index >= ADC_SAMPLES)
		average_adc6_index = 0;

	average_adc_sum = 0;
	for (average_adc_count = 0; average_adc_count < ADC_SAMPLES;  average_adc_count++)
	   average_adc_sum = average_adc_sum + average_adc6[average_adc_count];

	average_adc_result = average_adc_sum / ADC_SAMPLES;

	*chan6 = average_adc_result;							// Save conversion result

	// Channel 7
	ADMUX  &= ~(1 << MUX0);									// Clear MUX bits
	ADMUX  &= ~(1 << MUX1);									//
	ADMUX  &= ~(1 << MUX2);									//
	ADMUX  &= ~(1 << MUX3);									//

	ADMUX  |= (1 << MUX0);									// Set MUX0 - channel 7
	ADMUX  |= (1 << MUX1);									// Set MUX1 - channel 7
	ADMUX  |= (1 << MUX2);									// Set MUX2 - channel 7
	ADCSRA |= (1 << ADSC);									// Start ADC conversion
	while(ADCSRA & (1<<ADSC));								// Wait for conversion to finish

	// average out the readings
	average_adc7[average_adc7_index++] = ADC;

	if (average_adc7_index >= ADC_SAMPLES)
		average_adc7_index = 0;

	average_adc_sum = 0;
	for (average_adc_count = 0; average_adc_count < ADC_SAMPLES;  average_adc_count++)
	   average_adc_sum = average_adc_sum + average_adc7[average_adc_count];

	average_adc_result = average_adc_sum / ADC_SAMPLES;

	*chan7 = average_adc_result;							// Save conversion result


#ifdef TEMPERATURE_ENABLED
	// Channel 8 - on chip temperature measurement
	//  work in progress - voltage changes on other ADC channels seem to massively affect this reading.
	//  trying a delay before/after changing the voltage reference, also throwing away the first conversion result

	ADMUX |= (1<<REFS0);									// Use Internal 1.1V Voltage Reference with external capacitor at AREF pin
	ADMUX |= (1<<REFS1);									// - not working, capacitor resisting change in AREF voltage?
	_delay_ms(100);											// FIXME: Short delay to help the voltage settle help?

	ADMUX  &= ~(1 << MUX0);									// Clear MUX bits
	ADMUX  &= ~(1 << MUX1);									//
	ADMUX  &= ~(1 << MUX2);									//
	ADMUX  &= ~(1 << MUX3);									//

	ADMUX  |= (1 << MUX3);									// Set MUX3 - channel 8 (temperature)

	//
	// Throw one conversion away, first one seems particularly random for temperature channel
	//
	ADCSRA |= (1 << ADSC);									// Start ADC conversion
	while(ADCSRA & (1<<ADSC));								// Wait for conversion to finish

	ADCSRA |= (1 << ADSC);									// Start ADC conversion
	while(ADCSRA & (1<<ADSC));								// Wait for conversion to finish

	// average out the readings
	average_adc8[average_adc8_index++] = ADC;

	if (average_adc8_index >= ADC_SAMPLES)
		average_adc8_index = 0;

	average_adc_sum = 0;
	for (average_adc_count = 0; average_adc_count < ADC_SAMPLES;  average_adc_count++)
	   average_adc_sum = average_adc_sum + average_adc8[average_adc_count];

	average_adc_result = average_adc_sum / ADC_SAMPLES;

	*chan8 = average_adc_result;							// Save conversion result

	ADMUX |= (1<<REFS0);									// Revert to AVcc with external capacitor at AREF pin
	ADMUX &= ~(1<<REFS1);									//
	_delay_ms(100);											// FIXME: Short delay to help the voltage settle help?
#endif

	ADCSRA &= ~(1 << ADEN);									// Disable the ADC

	PORTC &= ~(1 << 0);										// Clear C0 to disable resistor dividers
}

void PinChangeIntSetup(void)
{
	PCMSK2 |= (1<<PCINT18 | 1<<PCINT19);					// Enable pin change interrupt on PD2 / PD3
	PCICR |= (1<<PCIE2);
}

void EnableRadio(int enable)
{
	if (enable)
	{
		 // Power up the Xbee module
		 PORTC &= ~(1 << 1);								// De-assert port C1 to power radio on

		 // Wait for module to be ready (CTS)
		 while((PINC & (1<<2)));							// Wait until port C2 (CTS) goes low
	}
	else
	{
		 // Power down the Xbee
		 PORTC |= (1 << 1);									// Assert port C1 to power radio down
	}
}

void ADCSetup(void)
{
	ADMUX |= (1<<REFS0);									// Use AVcc with external capacitor at AREF pin
	ADCSRA |= (1<<ADPS2);									// ADC clock prescaler = F_CPU/16
															// 7.3728 MHz/32 = 115kHz
}

void USART_Setup(void)
{
	UBRR0L = USART_BAUD_PRESCALE;							// Load lower 8-bits of the baud rate register
	UBRR0H = (USART_BAUD_PRESCALE >> 8);					// Load upper 8-bits of the baud rate register
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);					// Turn on the transmit / receive circuitry
	UCSR0C |= (1 << USBS0);									// Use two stop bits to work around XBee series 1 bug
}

void USART_SendChar(char ByteToSend)
{
	while ((UCSR0A & (1 << UDRE0)) == 0) {};				// Wait until UDR is ready for more data
		UDR0 = ByteToSend;									// Write the current byte
}

void USART_SendString(char* StringPtr)
{
	// Enable the radio link
	EnableRadio(1);

	while (*StringPtr)
	{
		USART_SendChar(*StringPtr);
		StringPtr++;
	}

	// wait for transmit to complete before returning.
	//  - else we were powering down too quickly
	while (!(UCSR0A & (1 << TXC0)));

	// reset the transmit completion flag
	UCSR0A |= (1 << TXC0);

	// Disable the radio link
	EnableRadio(0);
}

void TimerSetup(void)
{
	//
	// Setup timer 2 -- used for periodic wakeup, supports wake from sleep mode
	//

	// Disable timer 2 interrupts
	TIMSK2 = 0;

	// Reset timer 2 counter (perhaps do this after stablised?)
	TCNT2  = 0;

	// Set timer 2 to asyncronous mode (32.768KHz crystal)
	//  - should already be in async mode from osccal routine
	//	but do we want to rely on this here?
	ASSR |= (1 << AS2);

	// Start timer 2 at external clock/1024
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);

	// Wait for TCN2xUB, OCR2xUB, and TCR2xUB.
	while ((ASSR & ((1 << TCN2UB)  |
						 (1 << OCR2AUB) |
						 (1 << OCR2BUB) |
						 (1 << TCR2AUB) |
						 (1 << TCR2BUB))));

	OCR2A	= 0x9F;											// Set CTC compare value
															//  0x9F = 5 sec @ 32.768 kHz clock, with prescaler of 1024
	TCCR2A |= (1 << WGM21);									// Configure timer 2 for CTC mode

	// Clear timer 2 output compare interrupt flag to prevent interrupt
	// firing immediately when CTC interrupt is enabled following
	// calibration routine timer overflows...
	TIFR2 |= (1 << OCF2A);

	// Enable CTC interrupt
	TIMSK2 |= (1 << OCIE2A);
}

uint8_t DoorControl(uint8_t action)
{
	uint16_t i, target;

	// Move the door - use the door position sensors to check position before moving
	if (action == LOWER)
	{
		if (!(PIND & (1 << 5)))								// Already at bottom of travel
			return 0;

		PORTB &= ~(1 << 1);									// Door direction down - clear B1
		PORTB |= (1 << 2);									// Door direction down - set B2
	}
	if (action == RAISE)
	{
		if (!(PIND & (1 << 6)))								// Already at top of travel
			return 0;

		PORTB &= ~(1 << 2);									// Door direction up - clear B2
		PORTB |= (1 << 1);									// Door direction up - set B1
	}

	if (action == LOWER)
		target = DOOR_ACTION_TIMEOUT_DOWN * 50;				// Lower door for 1 second less than raised
	else													//  - else slowly ratchets down if
		target = DOOR_ACTION_TIMEOUT_UP * 50;				//    stop sensors aren't reached

	// Use the door position sensors to end motion at correct point, polling at 50Hz
	for (i=0; i<target; i++)
	{
		// Stop moving if we've reached the limits of travel, as detected by mag switches
		if ((action == LOWER) && (!(PIND & (1 << 5))))
		{
			// leave motor running a little longer to give us some slack in the cable
			_delay_ms(200);
			break;

		}

		if ((action == RAISE) && (!(PIND & (1 << 6))))
		{
			//_delay_ms(100);
			break;
		}

		PORTD |= (1 << 4);									// Enable motor driver - set D4
		PORTB |= (1 << 0);									// Enable door motor - set B0
		_delay_ms(20);										// Wait 20ms before checking position again..
	}

	PORTB &= ~(1 << 0);										// Disable door motor - clear B0
	PORTD &= ~(1 << 4);										// Disable motor driver - clear D4

	return i / 50;
}

void EnablePinPullups (uint8_t action)
{
	if (action)												// Enable while running
	{
		PORTD |= (1 << 5);									// Enable pullup resistor on port D5 (pulled low - door @ bottom
		PORTD |= (1 << 6);									// Enable pullup resistor on port D6 (pulled low - door @ top
		PORTD |= (1 << 7);									// Enable pullup resistor on port D7 (pulled low enables debug output)
	}
	else													// Disable when sleeping
	{
		PORTD &= ~(1 << 5);									// Disable pullup resistor on port D5 (pulled low - door @ bottom
		PORTD &= ~(1 << 6);									// Disable pullup resistor on port D6 (pulled low - door @ top
		PORTD &= ~(1 << 7);									// Disable pullup resistor on port D7 (pulled low enables debug output)
	}

}

void PinConfig(void)
{
	// PIN CONFIGURATION
	DDRB |= (1 << 0);										// Set port B0 as output for motor control (not required if using timer pwm)
	DDRB |= (1 << 1);										// Set port B1 as output for motor direction control - up
	DDRB |= (1 << 2);										// Set port B2 as output for motor direction control - down
	DDRC |= (1 << 0);										// Set port C0 as output for enabling resistor dividers
	DDRC |= (1 << 1);										// Set port C1 as output for radio sleep select
	DDRC |= (1 << 4);										// Set port C4 as output for debug LED

	PORTC |= (1 << 1);										// Assert port C1 to enable xigbee sleep

	// Enable pullups on switch sensors (always on)
	PORTD |= (1 << 2);										// Enable pullup resistor on port D2 (pulled low - manual door lower)
	PORTD |= (1 << 3);										// Enable pullup resistor on port D3 (pulled low - manual door raise)

	// Enable pullups on door/debug sensors (turned off while sleeping)
	EnablePinPullups(1);

	//Enable pullups on unused pins
	PORTC |= (1 << 3);							  			// Enable pullup resistor on port C3
}

// Convert a (max) four digit unsigned int to a text string with (n)n.nn formatting
//  - doing it this way to avoid floating point sprintf() overhead
void UnsignedToDecimalString4(uint16_t input, char * output_string)
{
	char temp_string[6];

	utoa(input, temp_string, 10);

	switch (strlen(temp_string))
	{
		case 4:
			*output_string++ = temp_string[0];
			*output_string++ = temp_string[1];
			*output_string++ = '.';
			*output_string++ = temp_string[2];
			*output_string++ = temp_string[3];
			*output_string++ = 0x0;
			break;

		case 3:
			*output_string++ = temp_string[0];
			*output_string++ = '.';
			*output_string++ = temp_string[1];
			*output_string++ = temp_string[2];
			*output_string++ = 0x0;
			break;

		case 2:
			*output_string++ = '0';
			*output_string++ = '.';
			*output_string++ = temp_string[0];
			*output_string++ = temp_string[1];
			*output_string++ = 0x0;
			break;

		case 1:
			*output_string++ = '0';
			*output_string++ = '.';
			*output_string++ = '0';
			*output_string++ = temp_string[0];
			*output_string++ = 0x0;
			break;

		default:
			*output_string = 0x0;
			break;
	}
}

ISR(TIMER2_COMPA_vect)
{
	PORTC ^= (1 << 4);										// Toggle the debug LED on port C4

	// Watchdog debugging - do not reset watchdog if D7 pulled low
	// if ((PIND & (1 << 7)))
	// 	wdt_reset();

	wdt_reset();											// Reset the watchdog timer

	if ((++timer_tick == MAX_TIMER_TICKS))					// If we have reached sampling interval,
	{
		 timer_tick = 0;
		 timer_interval = 1;
	}
}

ISR(PCINT2_vect)
{
	// Deal with manual override button presses, we don't care too much about bounces
	// as long as we reliably get a button release at the end !!
	if (!(PIND & (1 << 2)))									// Down button pressed
	{
		PORTB &= ~(1 << 1);									// Door direction down - clear PB1
		PORTB |= (1 << 2);									// Door direction down - set PB2
		PORTD |= (1 << 4);									// Enable motor driver - set PD4
		PORTB |= (1 << 0);									// Enable door motor - set PB0
	}
	else if (!(PIND & (1 << 3)))							// Up button pressed
	{
		PORTB &= ~(1 << 2);									// Door direction up - clear PB2
		PORTB |= (1 << 1);									// Door direction up - set PB1
		PORTD |= (1 << 4);									// Enable motor driver - set PD4
		PORTB |= (1 << 0);									// Enable door motor - set PB0
	}
	else													// Neither button pressed
	{
		PORTB &= ~(1 << 0);									// Disable door motor - clear PB0
		PORTD &= ~(1 << 4);									// Disable motor driver - clear PD4
	}
}

int main (void)
{
	char Buffer[64];
	uint8_t time_of_day, door_position, door_action_time, door_action = NOTHING;
	uint16_t adc5, adc6, adc7, adc8 = 0;						// raw ADC readings
	uint16_t i, battery;
	uint16_t door_interval_countdown = 0;
	char str_batteryV[5];
#ifdef TEMPERATURE_ENABLED
	int8_t  temperature;
#endif

	// *** TODO: STK500 configuration - not required for standalone design ***
	// Use external 32.768kHz clock instead of a crystal
	// ASSR |= (1 << EXCLK);

    // Make sure all clock division is turned off (8MHz RC clock)
    CLKPR  = (1 << CLKPCE);
    CLKPR  = 0x00;

	PinConfig();											// Set pins as inputs / outputs / enable pullups etc
	ADCSetup();

	// Get initial ADC readings, call enough times to pre-fill the averaging arrays
	for (i=0; i < ADC_SAMPLES; i++)
		ADCRead(&adc5, &adc6, &adc7, &adc8);

	// Set initial state from light sensor
	//  - Use the door position sensors to determine initial door position, and raise/lower as necessary
	// FIXME: ADC check reversed for sensor layout on 1.01 board
	if (adc5 < adc6)
	{
		time_of_day = DAY;
		door_action_time = DoorControl(RAISE);				// Won't move if already in position
	}
	else
	{
		time_of_day = NIGHT;
		door_action_time = DoorControl(LOWER);				// Won't move if already in position
	}

	USART_Setup();
	PinChangeIntSetup();
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);					// Set POWER SAVE sleep mode
	wdt_enable(WDTO_8S);									// Enable watchdog timer - 8 second timeout...
	sei();													// Enable global interrupts

	#ifdef RUNTIME_OSCCAL
	// Initialise OSCCAL to centre point of it's range before the initial calibration
	OSCCAL = (0x7F / 2);

	// Calibrate the internal oscillator for reliable serial comms (resets timer 2)
	OSCCAL_Calibrate();
#else
	OSCCAL = 89;											// Typical historical value (89 on new board)
#endif

	TimerSetup();											// Restart timer 2 - async (needed for watchdog reset)

	if (DEBUG_ENABLED)
	{
		sprintf(Buffer, "Starting....\n");
		USART_SendString(Buffer);
		sprintf(Buffer, "DEBUG: F_CPU: %lu, OSCCAL: %i, MCUSR: %i\n", F_CPU, OSCCAL, mcusr_mirror);
		USART_SendString(Buffer);
	}

	if (DEBUG_ENABLED)
	{
		sprintf(Buffer, "DEBUG: bright: %u, thresh: %u, batt: %u, temp: %u\n", adc5, adc6, adc7, adc8);
		USART_SendString(Buffer);
	}

	while(1)
	{
		if (timer_interval)
		{
			timer_interval = 0;

#ifdef RUNTIME_OSCCAL
			// Calibrate the internal oscillator for reliable serial comms (resets timer 2 so do it asap after rollover)
			OSCCAL_Calibrate();
#endif

			// Restart timer2
			//  - Note: We lose a bit of time here with the recalibration, but have
			//	optimised the recalibration routine - so minimal impact
			TimerSetup();

			// Get ADC readings
			ADCRead(&adc5, &adc6, &adc7, &adc8);

			// Calculate a voltage from the battery value
			// 0 = 0v, 1023 = 3.3v ADC reference
			//
			// Expecting battery voltage up to ~12v, so use 18k:6k7 voltage divider
			//  giving us 0 = 0v, 1023 = 3.3v, therefore v = ADC / 85
			//
			battery = (adc7 * 100UL) / 85;

			// Format the voltage as a string with two decimal places..
			UnsignedToDecimalString4(battery, str_batteryV);

			// Calculate the temperature in degrees C
			// FIXME: optimise this formula, 1mV/degree C. 1.1V reference, 0-1023 raw values
#ifdef TEMPERATURE_ENABLED
			temperature = adc8 - TEMPERATURE_OFFSET;
#endif

			// Get the door position
			if (!(PIND & (1 << 5)))
				door_position = BOTTOM;
			else if (!(PIND & (1 << 6)))
				door_position = TOP;
			else
				door_position = UNKNOWN;

			if (DEBUG_ENABLED)
			{
				sprintf(Buffer, "DEBUG: F_CPU: %lu, OSCCAL: %i, MCUSR: %i\n", F_CPU, OSCCAL, mcusr_mirror);
				USART_SendString(Buffer);
				sprintf(Buffer, "DEBUG: bright: %u, thresh: %u, batt: %u, temp: %u\n", adc5, adc6, adc7, adc8);
				USART_SendString(Buffer);
#ifdef TEMPERATURE_ENABLED
				sprintf(Buffer, "DEBUG: battery voltage: %s, time of day: %i, temperature: %i%cC\n", str_batteryV, time_of_day, temperature, 0xf8);
#else
				sprintf(Buffer, "DEBUG: battery voltage: %s, time of day: %i\n", str_batteryV, time_of_day);
#endif
				USART_SendString(Buffer);
				sprintf(Buffer, "DEBUG: door position: %i, door interval countdown: %u\n", door_position, door_interval_countdown);
				USART_SendString(Buffer);
			}

			// Use elapsed time countdown to prevent opening/closing too soon to previous event
			//  this should mostly be useful to prevent opening the door too early mid summer
			//  currently set to 8hrs, ensure this doesn't leave it open too late mid-winter
			if (door_interval_countdown)
			{
				door_interval_countdown--;

				// May 2011 changes;
				//  Add some resilience in case the door does not open correctly first time
				//  (too much slack in line, frosted shut or whatever)
				//
				//  If daytime, and door is not fully opened 30/60/90/120 mins after initial opening
				//  then try again
				if ((time_of_day == DAY) && (door_position != TOP))
				{
					if ((door_interval_countdown == DOOR_INTERVAL_ITERATIONS - 7)  ||
						(door_interval_countdown == DOOR_INTERVAL_ITERATIONS - 13) ||
						(door_interval_countdown == DOOR_INTERVAL_ITERATIONS - 19) ||
						(door_interval_countdown == DOOR_INTERVAL_ITERATIONS - 25))
					{
						door_action_time = DoorControl(RAISE);
						if (door_action_time == DOOR_ACTION_TIMEOUT_UP)
							sprintf(Buffer, "WARNING: door FAILED to open in %u seconds!!!\n", door_action_time);
						else
							sprintf(Buffer, "ACTION: door raised in %u seconds!\n", door_action_time);
						USART_SendString(Buffer);
					}
				}
			}
			else
			{
				// Has light sensor crossed threshold ?
				// FIXME: ADC check reversed for sensor layout on 1.01 board
				if ((time_of_day == DAY) && (adc5 > adc6))
				{
					// time for bed ;)
					time_of_day = NIGHT;
					door_action = LOWER;
				}
				// FIXME: ADC check reversed for sensor layout on 1.01 board
				else if ((time_of_day == NIGHT) && (adc5 < adc6	))
				{
					// rise and shine ;)
					time_of_day = DAY;
					door_action = RAISE;
				}

				if ((door_action == RAISE) || (door_action == LOWER))
				{
					// (re)set the countdown timer
					door_interval_countdown = DOOR_INTERVAL_ITERATIONS - 1;

					// Open/Close the door, returns elapsed time in seconds
					door_action_time = DoorControl(door_action);

					// Write the door movement result to the serial port
					//
					if (door_action == RAISE)
					{
						if (door_action_time == DOOR_ACTION_TIMEOUT_UP)
							sprintf(Buffer, "WARNING: door FAILED to open in %u seconds!!!\n", door_action_time);
						else
							sprintf(Buffer, "ACTION: door raised in %u seconds!\n", door_action_time);
					}
					else if (door_action == LOWER)
					{
						if (door_action_time == DOOR_ACTION_TIMEOUT_DOWN)
							sprintf(Buffer, "WARNING: door FAILED to close in %u seconds!!!\n", door_action_time);
						else
							sprintf(Buffer, "ACTION: door closed in %u seconds!\n", door_action_time);
					}
					USART_SendString(Buffer);

					door_action = NOTHING;
				}
			}
		}

		// Sleeping too quickly after waking from aysnc clock causes issues, so re-write
		// the CTC value and wait for OCR2AUB to clear before re-entering sleep mode
		OCR2A = 0x9F;
		while (ASSR & (1 << OCR2AUB));

		// Disable pullups on door/debug sensors (save power while sleeping)
		EnablePinPullups(0);

		// Sleep until next interrupt
		sleep_mode();

		// Enable pullups on door/debug sensors
		EnablePinPullups(1);
	}
	return (0);
}
