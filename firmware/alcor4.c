/* Alcor4
 * Copyright (c) 2017-2019 Sven Kreiensen <sven.kreiensen@web.de>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include "usb_serial.h"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz	0x00
#define CPU_125kHz	0x07
#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))

#define USE_ADC 1
#define USB_ECHO	1

#define LED_ON()	(PORTD |= (1<<6))
#define LED_OFF()	(PORTD &= ~(1<<6))
#define LED_OUTPUT()	(DDRD |= (1<<6))

#define GPIO_GOSOUTH_OUTPUT()	(DDRB |= (1<<1))
#define GPIO_GONORTH_OUTPUT()	(DDRB |= (1<<0))
#define GPIO_PWM_OUTPUT()		(DDRB |= (1<<7))

#define GPIO_GOSOUTH_ON()		(PORTB |= (1<<1))
#define GPIO_GOSOUTH_OFF()	(PORTB &= ~(1<<1))
#define GPIO_GONORTH_ON()		(PORTB |= (1<<0))
#define GPIO_GONORTH_OFF()	(PORTB &= ~(1<<0))
#define GPIO_PWM_ON()		(PORTB |= (1<<7))
#define GPIO_PWM_OFF()		(PORTB &= ~(1<<7))

#define GPIO_RAPHASE1_OUTPUT()	(DDRC |= (1<<6))
#define GPIO_RAPHASE2_OUTPUT()	(DDRC |= (1<<7))

#define GPIO_RAPHASE1_ON()	(PORTC |= (1<<6))
#define GPIO_RAPHASE1_OFF()	(PORTC &= ~(1<<6))
#define GPIO_RAPHASE2_ON()	(PORTC |= (1<<7))
#define GPIO_RAPHASE2_OFF()	(PORTC &= ~(1<<7))

#define KEY_RAEAST	(1<<PF5)
#define KEY_RAWEST	(1<<PF0)
#define KEY_DECNORTH	(1<<PF4)
#define KEY_DECSOUTH	(1<<PF1)
#define ALL_KEYS        (KEY_RAEAST | KEY_RAWEST | KEY_DECNORTH | KEY_DECSOUTH )

#define GPIO_KEYS_INPUT()	(DDRF &= ~ALL_KEYS)

#define NO_DURATION 0
#define DEFAULT_POLL_INTERVAL	5	// 20ms
#define DEFAULT_RA_GUIDE_SPEED	0	// slow
#define TRACKING_RATE_SIDERIAL	0
#define TRACKING_RATE_LUNAR	1
#define TRACKING_RATE_SOLAR	2
#define TRACKING_RATE_KING	3
#define TRACKING_RATE_STOP	4

/*********************************************************************
 Function prototypes
 *********************************************************************/
void send_str(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);

/*********************************************************************
 Global variables
 *********************************************************************/
volatile uint8_t halfwave;               // First or seconds half wave
volatile uint8_t dec_slew_counter;
volatile uint8_t ra_slew_counter;
volatile uint16_t ra_preload;
volatile uint8_t dec_duty;
volatile uint8_t key_decnorth_state;
volatile uint8_t key_decsouth_state;
volatile uint8_t key_raeast_state;
volatile uint8_t key_rawest_state;
volatile uint8_t key_debug;
volatile uint8_t ra_guide_speed_fast;
volatile uint8_t tracking_rate;
volatile uint32_t dec_north_counter;
volatile uint32_t dec_south_counter;
volatile uint32_t ra_west_counter;
volatile uint32_t ra_east_counter;


/*********************************************************************
 RA motor drive
 *********************************************************************/

/*
    Duty cycle 50% per half wave:
    Th(on) = Th / 2
    Th(off) = Th / 2

     ^ Relais Output 1
     |
 on  +-----+
     |     |
     |     |
 off +-----+-----+-----------+-->
      1st halfw.   2nd halfw.

     ^ Relais Output 2
     |
 on  |           +-----+
     |           |     |
     |           |     |
 off +-----------+-----+-----+-->
      1st halfw.   2nd halfw.

 */

/* Overflow Interrupt Handler */
ISR(TIMER3_OVF_vect) {
    GPIO_RAPHASE1_OFF();
    GPIO_RAPHASE2_OFF();

    TCNT3 = ra_preload;
}

/* Compare match Interrupt Handler
   Called at 50% of 1/2 wave
 */
ISR(TIMER3_COMPA_vect) {
    uint8_t h = halfwave;

    if (h) {
	// Second half wave
	GPIO_RAPHASE1_OFF();
	GPIO_RAPHASE2_ON();
	h=0;
    } else {
	// First half wave
	GPIO_RAPHASE1_ON();
	GPIO_RAPHASE2_OFF();
	h=1;
    }

    halfwave = h;
}

void ra_init(void) {
    tracking_rate = TRACKING_RATE_STOP;
    ra_guide_speed_fast = DEFAULT_RA_GUIDE_SPEED;
    halfwave = 0;

    GPIO_RAPHASE1_OUTPUT();
    GPIO_RAPHASE2_OUTPUT();

    GPIO_RAPHASE1_OFF();
    GPIO_RAPHASE2_OFF();

    ra_west_counter = 0;
    ra_west_counter = 0;
}


/* Tracking frequency calculation

Find the custom siderial tracking frequency with this formula

           T x SF
       F = ------
             R

F: frequency
T: number of teath in the R.A. gear
R: speed of the motor in rph
SF: Speed factor (siderial = 2.507, solar = 2.500, lunar = 2.414, king = 2.506)

*/

/*  Siderial rate: 
    Frequency 60.1643 Hz
    Period T = 1 / 60.164 Hz = 16.621235 ms
    1 half wave Th =  T / 2 = 8.3096 ms
    Verwendeter prescaler: 8
    Verwendeter preload: 48917
    TCCR3B |= (1<<CS31)
    TCNT3 = 48917
*/
#define SIDERIAL_CNTRPRELOAD	48917
#define SIDERIAL_COMPMATCH	56900
//#define SIDERIAL_COMPMATCH	57226 # Waere das nicht korrekt?
#define SIDERIAL_PRESCALER	(1 << CS31)

/* Driver motor at siderial rate */
void ra_siderial(void) {
    ra_preload = SIDERIAL_CNTRPRELOAD;
    TCNT3=0;
    TCCR3A = 0;
    TCCR3B = SIDERIAL_PRESCALER;
    TCNT3 = SIDERIAL_CNTRPRELOAD;
    OCR3A = SIDERIAL_COMPMATCH;

    TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);
    
    LED_ON();
}

/*
    Lunar rate:
    Frequency 58.696 Hz
    Period T = 1 / 58.696 Hz = 17.036936 ms
    1 half wave Th = T / 2 = 8.518468 ms
    Verwendeter prescaler: 8
    Verwendeter preload: 48499
    TCCR3B |= (1<<CS31)
    TCNT3 = 48499
*/
#define LUNAR_CNTRPRELOAD	48499
#define LUNAR_COMPMATCH		57018
#define LUNAR_PRESCALER		(1 << CS31)
void ra_lunar(void) {
    ra_preload = LUNAR_CNTRPRELOAD;
    TCNT3=0;
    TCCR3A = 0;
    TCCR3B = LUNAR_PRESCALER;
    TCNT3 = LUNAR_CNTRPRELOAD;
    OCR3A = LUNAR_COMPMATCH;

    TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);

    LED_ON();
}

/*
    Solar rate:
    Frequency 60 Hz
    Period T = 1 / 60 Hz = 16.666 ms
    1 half wave Th = T / 2 = 8.333 ms
    Verwendeter prescaler: 8
    Verwendeter preload: 48870
    TCCR3B |= (1<<CS31)
    TCNT3 = 48870
*/
#define SOLAR_CNTRPRELOAD	48870
#define SOLAR_COMPMATCH		57202
#define SOLAR_PRESCALER		(1 << CS31)
void ra_solar(void) {
    ra_preload = SOLAR_CNTRPRELOAD;
    TCNT3=0;
    TCCR3A = 0;
    TCCR3B = SOLAR_PRESCALER;
    TCNT3 = SOLAR_CNTRPRELOAD;
    OCR3A = SOLAR_COMPMATCH;

    TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);

    LED_ON();
}

/*
    King rate:
    This is a specialized rate developed by Havard astronomer E.S. King, circa 1900.
    As an object rises from the horizon towards the meridian, the amount of
    atmosphere its light passes through decreases. As it sets, the opposite is true.
    Because the thicker atmosphere at the horizon refracts (bends) the light more
    than the thinner atmosphere at the meridian, the rate of the object's apparent
    motion is constantly changing. The amount of changes varies with many factors,
    including temperature, barometric pressure, and observer's location.
    In order to best approximate actual conditions, the King rate based on an 
    average of all these factors. As the resultant rate is only 0.033% slower than
    the siderial rate

    Frequency 60.1643 Hz * 0.99967 = 60.1444458
    Period T = 1 / 60.1444458 Hz = 16.62664 ms
    1 half wave Th =  T / 2 = 8.31332 ms
    Verwendeter prescaler: 8
    Verwendeter preload: 48909
    TCCR3B |= (1<<CS31)
    TCNT3 = 48909
*/
#define KING_CNTRPRELOAD	48909
#define KING_COMPMATCH	57222
#define KING_PRESCALER	(1 << CS31)

void ra_king(void) {
    ra_preload = KING_CNTRPRELOAD;
    TCNT3=0;
    TCCR3A = 0;
    TCCR3B = KING_PRESCALER;
    TCNT3 = KING_CNTRPRELOAD;
    OCR3A = KING_COMPMATCH;

    TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);
    
    LED_ON();
}



void ra_stop(void) {
    TCCR3A &= ~(1<<WGM32);  // CTC mode 12
    TCCR3B &= ~((1<<CS31) | (1<<WGM33));   // Prescaler: fcpu / 1 and CTC mode 12

    TIMSK3 &= ~((1<<TOIE3) | (1<<OCIE3A)); // Disable Timer 3 Overflow and Compare Interrupt for channel A

    GPIO_RAPHASE1_OFF();
    GPIO_RAPHASE2_OFF();

    LED_OFF();
}

/*  East slewing rate:
    Frequency 44.164 Hz
    Period T = 1 / 44.164 Hz = 22.642877 ms
    1 half wave Th = T / 2 = 11.321439 ms
    Verwendeter prescaler: 8
    Verwendeter preload: 42893
    TCCR3B |= (1<<CS31)
    TCNT3 = 42893
*/
#define EAST_SLEW_CNTRPRELOAD	42893
#define EAST_SLEW_COMPMATCH	54215
#define EAST_SLEW_PRESCALER	(1 << CS31)

void ra_east(unsigned int duration) {
    unsigned int d = 0;

    if (ra_guide_speed_fast == 1) {
	ra_stop();
    } else {
	ra_preload = EAST_SLEW_CNTRPRELOAD;
	TCNT3=0;
	TCCR3A = 0;
	TCCR3B = EAST_SLEW_PRESCALER;
	TCNT3 = EAST_SLEW_CNTRPRELOAD;
	OCR3A = EAST_SLEW_COMPMATCH;

	TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);
    }
    
    if (duration > NO_DURATION) {
	d = duration / 100;
	ra_slew_counter = d + 1;
    }

    ra_east_counter++;
    LED_OFF();
}

/* West slewing rate: 4"/Sec
    Frequency 76.164 Hz
    Period T = 1 / 76.164 Hz = 13.129563 ms
    1 half wave Th = T / 2 = 6.5647815 ms
    Verwendeter prescaler: 64
    Verwendeter preload: 63899
    TCCR3B |= (1<<CS31) | (1<<CS30)
    TCNT3 = 63899
*/
#define WEST_SLEW_CNTRPRELOAD	63899
#define WEST_SLEW_COMPMATCH	64718
#define WEST_SLEW_PRESCALER	(1 << CS31) | (1<<CS30)

/* New
   Frequency 86 Hz
  Period T = 1 / 86 Hz = 12.0 ms
    1 half wave Th = T / 2 = 5.8 ms
    Verwendeter prescaler: 64
    Verwendeter preload: 64086
    TCCR3B |= (1<<CS31) | (1<<CS30)
    TCNT3 = 64086

 */
#define WEST_FASTSLEW_CNTRPRELOAD	64086
#define WEST_FASTSLEW_COMPMATCH	64811
#define WEST_FASTSLEW_PRESCALER	(1 << CS31) | (1<<CS30)

void ra_west(unsigned int duration) {
    unsigned int d = 0;
    
    if (ra_guide_speed_fast == 1) {
	ra_preload = WEST_SLEW_CNTRPRELOAD;
	TCNT3=0;
	TCCR3A = 0;
	TCCR3B = WEST_FASTSLEW_PRESCALER;
	TCNT3 = WEST_FASTSLEW_CNTRPRELOAD;
	OCR3A = WEST_FASTSLEW_COMPMATCH;

	TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);
    } else {
	ra_preload = WEST_SLEW_CNTRPRELOAD;
	TCNT3=0;
	TCCR3A = 0;
	TCCR3B = WEST_SLEW_PRESCALER;
	TCNT3 = WEST_SLEW_CNTRPRELOAD;
	OCR3A = WEST_SLEW_COMPMATCH;

	TIMSK3 |= (1<<TOIE3) | (1 << OCIE3A);
    }

    if (duration > NO_DURATION) {
	d = duration / 100;
	ra_slew_counter = d + 1;
    }

    ra_west_counter++;
    LED_OFF();
}

/*********************************************************************
 Declination and focuser motor drive
 ********************************************************************/

/* TIMER1 overflow interrupt function */
ISR(TIMER0_OVF_vect) {
    TCNT0 = 217;
    GPIO_PWM_ON();
}

ISR(TIMER0_COMPA_vect) {
    GPIO_PWM_OFF();
}

void dec_set_speed(unsigned int duty) {
    if (duty > 66) {
	OCR0A = 254;	// duty 99%
	dec_duty = 99;
    }
    if ((duty <= 66) && (duty > 50)) {
	OCR0A = 242;	// duty 66% = 217 + (255-217)*0.66
	dec_duty = 66;
    }
    if ((duty <= 50) && (duty > 33)) {
	OCR0A = 236;	// duty 50% = 217 + (255-217)*0.5
	dec_duty = 50;
    }
    if (duty <= 33) {
	OCR0A = 229;	// duty 33% = 217 + (255-217)*0.33
	dec_duty = 33;
    }
}

void dec_init(void) {
    dec_duty = 99; // 99%

    GPIO_PWM_OUTPUT();
    GPIO_PWM_OFF();

    /* Start motors, PWM 1,6 kHz */
    GPIO_PWM_OUTPUT();
    TCNT0 = 0;

    TCCR0A = 0;	// Normal mode
    TCCR0B |= (1<<CS02);	// Prescaler 256
    dec_set_speed(dec_duty);
    TCNT0 = 217;

    TIMSK0 |= (1 << TOIE0) | (1<<OCIE0A); // Enable overflow interrupt

    GPIO_GONORTH_OUTPUT();
    GPIO_GONORTH_OFF();
    GPIO_GOSOUTH_OUTPUT();
    GPIO_GOSOUTH_OFF();
    GPIO_PWM_ON();
    
    dec_north_counter = 0;
    dec_south_counter = 0;
}

void dec_stop(void) {
    GPIO_GOSOUTH_OFF();
    GPIO_GONORTH_OFF();
}

void dec_north(unsigned int duration) {
    unsigned int d = 0;

    GPIO_GONORTH_OFF();
    GPIO_GOSOUTH_ON();

    if (duration > NO_DURATION) {
	d = duration / 100;
	dec_slew_counter = d + 1;
    }
    
    dec_north_counter++;
}

void dec_south(unsigned int duration) {
    unsigned int d = 0;

    GPIO_GONORTH_ON();
    GPIO_GOSOUTH_OFF();

    if (duration > NO_DURATION) {
	d = duration / 100;
	dec_slew_counter = d + 1;
    }
    
    dec_south_counter++;
}

/*********************************************************************
 System clock
 ********************************************************************/

/* TIMER1 overflow interrupt function */
ISR(TIMER1_OVF_vect) {
    //TCNT1 = 64911;
    //TCNT1 = 64911;	// 10ms
    TCNT1 = 59286;	// 100ms

    if (dec_slew_counter > 1)
	dec_slew_counter--;
    else if (dec_slew_counter == 1) {
	// Stop dec motor and disable slew counter
	dec_stop();
	dec_slew_counter = 0;
    }

    if (ra_slew_counter > 1)
	ra_slew_counter--;
    else if (ra_slew_counter == 1) {
	// Restart ra motor and disable slew counter
	if (tracking_rate == TRACKING_RATE_SIDERIAL)
	    ra_siderial();
	else if (tracking_rate == TRACKING_RATE_LUNAR)
	    ra_lunar();
	else if (tracking_rate == TRACKING_RATE_SOLAR)
	    ra_solar();
	else if (tracking_rate == TRACKING_RATE_KING)
	    ra_king();
	else
	    ra_stop();

	ra_slew_counter = 0;

	LED_ON();
    }
}

/* Initialize timer with 100ms period */
void clock_init(void) {
    dec_slew_counter = 0;
    ra_slew_counter = 0;

    TCNT1 = 0;

    TCCR1A = 0;	// Normal counter mode
    TCCR1B |= (1<<CS12);	// Prescaler 256
    //TCNT1 = 64911;	// 10ms
    TCNT1 = 59286;

    TIMSK1 |= (1 << TOIE1); // Enable overflow interrupt
}

#if USE_ADC
/*********************************************************************
 ADC
 ********************************************************************/
#define ADC_MUX_PIN_F0    0x00
#define ADC_MUX_PIN_F1    0x01
#define ADC_MUX_PIN_F2    0x02
#define ADC_MUX_PIN_F3    0x03
#define ADC_MUX_PIN_F4    0x04
#define ADC_MUX_PIN_F5    0x05
#define ADC_MUX_PIN_F6    0x06
#define ADC_MUX_PIN_F7    0x07
#define ADC_MUX_PIN_D4    0x20
#define ADC_MUX_PIN_D6    0x21
#define ADC_MUX_PIN_D7    0x22
#define ADC_MUX_PIN_B4    0x23
#define ADC_MUX_PIN_B5    0x24
#define ADC_MUX_PIN_B6    0x25

#define ADC_REF_POWER     (1<<REFS0)
#define ADC_REF_INTERNAL  ((1<<REFS1) | (1<<REFS0))
#define ADC_REF_EXTERNAL  (0)

#define ADC_TRIGGER_FREE_RUNNING	0
#define ADC_TRIGGER_ANALOG_COMP		1
#define ADC_TRIGGER_EXT_INT0		2
#define ADC_TRIGGER_TIMER0_COMPARE	3
#define ADC_TRIGGER_TIMER0_OVERFLOW	4
#define ADC_TRIGGER_TIMER1_COMPARE_B	5
#define ADC_TRIGGER_TIMER1_OVERFLOW	6
#define ADC_TRIGGER_TIMER1_CAPTURE	7
#define ADC_TRIGGER_TIMER4_OVERFLOW	8
#define ADC_TRIGGER_TIMER4_COMPARE_A	9
#define ADC_TRIGGER_TIMER4_COMPARE_B	10
#define ADC_TRIGGER_TIMER4_COMPARE_D	11

// These prescaler values are for high speed mode, ADHSM = 1
#if F_CPU == 16000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS1))
#elif F_CPU == 8000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS0))
#elif F_CPU == 4000000L
#define ADC_PRESCALER ((1<<ADPS2))
#elif F_CPU == 2000000L
#define ADC_PRESCALER ((1<<ADPS1) | (1<<ADPS0))
#elif F_CPU == 1000000L
#define ADC_PRESCALER ((1<<ADPS1))
#else
#define ADC_PRESCALER ((1<<ADPS0))
#endif

// Make one single measurement and throw it away
void adc_start(uint8_t mux, uint8_t aref) {
    ADCSRA = (1<<ADEN) | ADC_PRESCALER;	// enable the ADC, interrupt disabled
    ADCSRB = (1<<ADHSM) | (mux & 0x20);
    ADMUX = aref | (mux & 0x1F);		// configure mux and ref
    ADCSRA = (1<<ADEN) | ADC_PRESCALER | (1<<ADSC); // start the conversion
    while (ADCSRA & (1<<ADSC)) ;                    // wait for result

    (void) ADCL;
    (void) ADCH;
}

void adc_setup(uint8_t mux, uint8_t aref) {
    //ADMUX = (0<<REFS0) | (0<<REFS1); // select external AREF
    //DIDR2 = ADC_MUX_PIN_F6; // digital input disable
    ADCSRA = (1<<ADEN) | ADC_PRESCALER; // enable ADC, 128 clk divider
}

void adc_select(uint8_t mux, uint8_t aref) {
    ADMUX = aref | (mux & 0x1F);   //select channel (MUX0-4 bits)
    ADCSRB = (1<<ADHSM) | (mux & 0x20);   //select channel (MUX5 bit) 
    ADCSRA |= (1<<ADSC);	// ADC Start Conversion
}

uint16_t adc_read(uint8_t mux, uint8_t aref) {
    uint8_t low;

    while (ADCSRA & (1<<ADSC)) ;                    // wait for result
    low = ADCL;                                     // must read LSB first

    return (ADCH << 8) | low;                       // must read MSB only once!
//    return (ADCW);
}

// Single measurement
int16_t adc_read_single(uint8_t mux, uint8_t aref) {
    uint8_t low;

    // Select channel
    ADMUX = aref | (mux & 0x1F);		// configure mux and ref
    ADCSRB = (1<<ADHSM) | (mux & 0x20);
    ADCSRA = (1<<ADEN) | ADC_PRESCALER;	// enable the ADC, interrupt disabled

    // 
    ADCSRA = (1<<ADEN) | ADC_PRESCALER | (1<<ADSC); // start the conversion
    while (ADCSRA & (1<<ADSC)) ;                    // wait for result
    low = ADCL;                                     // must read LSB first

    return (ADCH << 8) | low;                       // must read MSB only once!
}

// Average value of nsamples multiple measurements 
int16_t adc_read_avg(uint8_t mux, uint8_t aref, uint8_t nsamples) {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < nsamples; ++i )
	sum += adc_read(mux, aref);

    return (uint16_t)( sum / nsamples );
}

#endif

/*********************************************************************
 Keypad handling
 ********************************************************************/
#define KEY_RELEASED		-1
#define KEY_NOT_PRESSED		0
#define KEY_PRESSED		1
#define KEY_STILL_PRESSED	2

void keys_init(void) {
//    GPIO_KEYS_INPUT();

#if USE_ADC
    adc_setup(ADC_MUX_PIN_F0 | ADC_MUX_PIN_F1 | ADC_MUX_PIN_F4 | ADC_MUX_PIN_F5, ADC_REF_POWER);
#else
    DDRF &= ~(1 << PF0);
    DDRF &= ~(1 << PF1);
    DDRF &= ~(1 << PF4);
    DDRF &= ~(1 << PF5);
#endif

    key_decnorth_state = KEY_NOT_PRESSED;
    key_decsouth_state = KEY_NOT_PRESSED;
    key_raeast_state = KEY_NOT_PRESSED;
    key_rawest_state = KEY_NOT_PRESSED;
}

unsigned char get_key_decnorth(uint8_t debug) {
#if USE_ADC
    adc_select(ADC_MUX_PIN_F4, ADC_REF_POWER);
    uint16_t val;
    val = adc_read_avg(ADC_MUX_PIN_F4, ADC_REF_POWER,4);

    if (val > 0x280) {
#else
    if (PINF & KEY_DECNORTH) {
#endif

	// Key is OFF

	if ((key_decnorth_state == KEY_STILL_PRESSED) || (key_decnorth_state == KEY_PRESSED)) {
	    // Key is released
	    key_decnorth_state = KEY_RELEASED;
	    LED_OFF();
	    dec_stop();
	} else
	    key_decnorth_state = KEY_NOT_PRESSED;
    } else {
	// Key is ON

	if ((key_decnorth_state == KEY_NOT_PRESSED) || (key_decnorth_state == KEY_RELEASED)) {
	    // Key is pressed
	    key_decnorth_state = KEY_PRESSED;
	    dec_north(NO_DURATION);
	    LED_ON();
	} else 
	    key_decnorth_state = KEY_STILL_PRESSED;
    }

    return key_decnorth_state;
}

unsigned char get_key_decsouth(uint8_t debug) {
#if USE_ADC
    adc_select(ADC_MUX_PIN_F1, ADC_REF_POWER);
    uint16_t val;
    val = adc_read_avg(ADC_MUX_PIN_F1, ADC_REF_POWER,4);

    if (val > 0x280) {
#else
    if (PINF & KEY_DECSOUTH) {
#endif
	// Key is OFF

	if ((key_decsouth_state == KEY_STILL_PRESSED) || (key_decsouth_state == KEY_PRESSED)) {
	    // Key is released
	    key_decsouth_state = KEY_RELEASED;
	    LED_OFF();
	    dec_stop();
	} else
	    key_decsouth_state = KEY_NOT_PRESSED;
    } else {
	// Key is ON

	if ((key_decsouth_state == KEY_NOT_PRESSED) || (key_decsouth_state == KEY_RELEASED)) {
	    // Key is pressed
	    key_decsouth_state = KEY_PRESSED;
	    dec_south(NO_DURATION);
	    LED_ON();
	} else 
	    key_decsouth_state = KEY_STILL_PRESSED;
    }

    return key_decsouth_state;
}

unsigned char get_key_raeast(uint8_t debug) {
#if USE_ADC
    adc_select(ADC_MUX_PIN_F0, ADC_REF_POWER);
    uint16_t val;
    val = adc_read_avg(ADC_MUX_PIN_F0, ADC_REF_POWER,4);
    
    if (val > 0x280) {
#else
    if (PINF & KEY_RAEAST) {
#endif
	// Key is OFF

	if ((key_raeast_state == KEY_STILL_PRESSED) || (key_raeast_state == KEY_PRESSED)) {
	    // Key is released
	    key_raeast_state = KEY_RELEASED;
	    LED_OFF();
	    
	    if (tracking_rate == TRACKING_RATE_SIDERIAL)
		ra_siderial();
	    else if (tracking_rate == TRACKING_RATE_LUNAR)
		ra_lunar();
	    else if (tracking_rate == TRACKING_RATE_SOLAR)
		ra_solar();
	    else if (tracking_rate == TRACKING_RATE_KING)
		ra_king();
	    else
		ra_stop();
	} else
	    key_raeast_state = KEY_NOT_PRESSED;
    } else {
	// Key is ON

	if ((key_raeast_state == KEY_NOT_PRESSED) || (key_raeast_state == KEY_RELEASED)) {
	    // Key is pressed
	    key_raeast_state = KEY_PRESSED;
	    ra_east(NO_DURATION);
	    LED_ON();
	} else 
	    key_raeast_state = KEY_STILL_PRESSED;
    }

    return key_raeast_state;
}

unsigned char get_key_rawest(uint8_t debug) {

#if USE_ADC
    adc_select(ADC_MUX_PIN_F5, ADC_REF_POWER);
    uint16_t val;
    val = adc_read_avg(ADC_MUX_PIN_F5, ADC_REF_POWER, 4);

    if (val > 0x280) {
#else
    if (PINF & KEY_RAWEST) {
#endif
	// Key is OFF

	if ((key_rawest_state == KEY_STILL_PRESSED) || (key_rawest_state == KEY_PRESSED)) {
	    // Key is released
	    key_rawest_state = KEY_RELEASED;
	    LED_OFF();

	    if (tracking_rate == TRACKING_RATE_SIDERIAL)
		ra_siderial();
	    else if (tracking_rate == TRACKING_RATE_LUNAR)
		ra_lunar();
	    else if (tracking_rate == TRACKING_RATE_SOLAR)
		ra_solar();
	    else if (tracking_rate == TRACKING_RATE_KING)
		ra_king();
	    else
		ra_stop();
	} else
	    key_rawest_state = KEY_NOT_PRESSED;
    } else {
	// Key is ON

	if ((key_rawest_state == KEY_NOT_PRESSED) || (key_rawest_state == KEY_RELEASED)) {
	    // Key is pressed
	    key_rawest_state = KEY_PRESSED;
	    ra_west(NO_DURATION);
	    
	    // Initial key press
	    if (tracking_rate == TRACKING_RATE_STOP)
		tracking_rate = TRACKING_RATE_KING;
		
	    LED_ON();
	} else 
	    key_rawest_state = KEY_STILL_PRESSED;
    }

    return key_rawest_state;
}

/*********************************************************************
 Restart in bootloader mode for firmware flashing
 ********************************************************************/
void disable_periphericals(void) {
    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
    TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; UCSR1B = 0; TWCR = 0;
    DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0; TWCR = 0;
    PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
}

void restart_bootloader(void) {
    ra_stop();

    cli();

    // disable watchdog, if enabled
    // disable all peripherals
    UDCON = 1;
    USBCON = (1<<FRZCLK);  // disable USB
    UCSR1B = 0;

    // 5 ms delay after disabling USB
    _delay_ms(5);

    disable_periphericals(); // But leave USB intact

    // Jump into bootloader
    asm volatile("jmp 0x7E00");

    while(1);
}

void restart_normal(void) {
    ra_stop();

    cli();
    disable_periphericals();
    _delay_ms(15);

    asm volatile("jmp 0");
    
    while(1);
}

/*********************************************************************
 Main function
 ********************************************************************/
int main(void) {
    char buf[32];
    uint8_t n;

    CPU_PRESCALE(CPU_125kHz);
    _delay_ms(1);		// allow slow power supply startup
    CPU_PRESCALE(CPU_16MHz); // set for 16 MHz clock

    LED_OUTPUT();

    clock_init();
    ra_init();
    sei();

    dec_init();
    keys_init();

    // initialize the USB, and then wait for the host
    // to set configuration.  If the Teensy is powered
    // without a PC connected to the USB port, this 
    // will wait forever.
    usb_init();
    while (!usb_configured()) {
	get_key_decnorth(0);
	get_key_decsouth(0);
	get_key_rawest(0);
	get_key_raeast(0);

	/* Poll interval */ ;
	_delay_ms(DEFAULT_POLL_INTERVAL);
    }

    while (1) {
	// wait for the user to run their terminal emulator program
	// which sets DTR to indicate it is ready to receive.
	while (!(usb_serial_get_control() & USB_SERIAL_DTR)) {
	    get_key_decnorth(0);
	    get_key_decsouth(0);
	    get_key_rawest(0);
	    get_key_raeast(0);

	    /* Poll interval */ ;
	    _delay_ms(DEFAULT_POLL_INTERVAL);
	}

	// discard anything that was received prior.  Sometimes the
	// operating system or other software will send a modem
	// "AT command", which can still be buffered.
	usb_serial_flush_input();

	// print a nice welcome message
	send_str(PSTR("\r\nAlcor3 v1.6 telescope drive controller,\r\n "
	    "Example Commands\r\n"
	    " :Mg0????# guide north\r\n"
	    " :Mg1????# guide south\r\n"
	    " :Mg2????# guide east\r\n"
	    " :Mg3????# guide west\r\n"
	    " :Msd??#   dec speed 66,50,33\r\n"
	    " :Msr?#    ra guide speed 0=slow, 1=fast\r\n"
	    " :Mra#	siderial rate\r\n"
	    " :Mrl#     lunar rate\r\n"
	    " :Mrs#     solar rate\r\n"
	    " :Mrk#     kings rate\r\n"
	    " :Mrh#     stop tracking\r\n"
	    " :Mi#      Status\r\n"
	    " :Mbn#     restart\r\n"
	    " :Mbb#     restart in booloader mode\r\n"));

	// and then listen for commands and process them
	while (1) {
	    send_str(PSTR("> ")); // send prompt
	    n = recv_str(buf, sizeof(buf));
	    if (n == 255) 
		break;
	    send_str(PSTR("\r\n"));
	    parse_and_execute_command(buf, n);

	    for (uint8_t i=0; i < 32; i++)
		buf[i]=0;
	}
    }

    // Stop all engines
    ra_stop();
    dec_stop();
}

/*********************************************************************
 Send a string to the USB serial port.  The string must be in
 flash memory, using PSTR
 *********************************************************************/
void send_str(const char *s) {
    char c;
    
    while (1) {
	c = pgm_read_byte(s++);
	if (!c) 
	    break;
	usb_serial_putchar(c);
    }
}

void send_uint(unsigned int i) {
    char str[16];
    utoa(i,str,10);

    for (uint8_t j=0; j < 10; j++)
	if (isdigit(str[j]))
	    usb_serial_putchar(str[j]);
}

void send_ulint(unsigned long int i) {
    char str[16];
    ultoa(i,str,10);

    for (uint8_t j=0; j < 10; j++)
	if (isdigit(str[j]))
	    usb_serial_putchar(str[j]);
}



/*********************************************************************
 Receive a string from the USB serial port.  The string is stored
 in the buffer and this function will not exceed the buffer size.
 A carriage return or newline completes the string, and is not
 stored into the buffer.
 The return value is the number of characters received, or 255 if
 the virtual serial connection was closed while waiting.
 ********************************************************************/
uint8_t recv_str(char *buf, uint8_t size) {
    int16_t r;
    uint8_t count=0;

    while (count < size) {
	r = usb_serial_getchar();
	if (r != -1) {
	    if (r == '\r' || r == '\n') 
		return count;

	    // GC USBST4 init string
	    if (r == 0x6) {
		// Respond with A
		usb_serial_putchar('A');

		return 0;
	    }

	    if (r >= ' ' && r <= '~') {
		*buf++ = r;
#ifdef USB_ECHO
		usb_serial_putchar(r);
#endif
		count++;

		// GC USBST4 command terminator
		if (r == '#')
		    return count;
	    }
	} else {
#if 1
	    get_key_decnorth(0);
	    get_key_decsouth(0);
	    get_key_rawest(0);
	    get_key_raeast(0);
#endif
	    if (!usb_configured() ||
		 !(usb_serial_get_control() & USB_SERIAL_DTR)) {
		// user no longer connected
		return 255;
	    }
	    // just a normal timeout, keep waiting

	}
    }

    return count;
}

/*********************************************************************
 parse a user command and execute it, or print an error message
 *********************************************************************/
void parse_and_execute_command(const char *buf, uint8_t num) {
    // Do nothing
    if (num <= 1)
	return;

    if (num < 4) {
	send_str(PSTR("command is too short\r\n"));
	return;
    }

    if (num > 9) {
	send_str(PSTR("command is too long\r\n"));
	return;
    }

    // Check if it is a command
    if (!(buf[0] == ':')) {
	send_str(PSTR("Syntax error\r\n"));
	return;
    }
    
    if (!(buf[1] == 'M')) {
	send_str(PSTR("Syntax error\r\n"));
	return;
    }

    if (!(buf[num-1] == '#')) {
	send_str(PSTR("Missing command terminator\r\n"));
	return;
    }
    
    // Guide
    if ((buf[2] == 'g') && (num==9)) {
	    unsigned int duration = 0;
	    
	    // ...1#
	    if (isdigit(buf[7]))
		duration = buf[7]-0x30;
	    // ..1.#
	    if (isdigit(buf[6]))
		duration = duration + ((buf[6]-0x30)*10);
	    // .1..#
	    if (isdigit(buf[5]))
		duration = duration + ((buf[5]-0x30)*100);
	    // 1...#
	    if (isdigit(buf[4]))
		duration = duration + ((buf[4]-0x30)*1000);

	    // guide north
	    if (buf[3] == '0') {
	    	dec_north(duration);
	    }

	    // guide north
	    if (buf[3] == 'n') {
	    	dec_north(duration);
	    }

	    // guide south
	    if (buf[3] == '1') {
		dec_south(duration);
	    }

	    // guide south
	    if (buf[3] == 's') {
		dec_south(duration);
	    }
	
	    // guide east
	    if (buf[3] == '2') {
		ra_east(duration);
	    }

	    // guide east
	    if (buf[3] == 'e') {
		ra_east(duration);
	    }
	
	    // guide west
	    if (buf[3] == '3') {
		ra_west(duration);
	    }

	    // guide west
	    if (buf[3] == 'w') {
		ra_west(duration);
	    }
	
	return;
    }
	
    // Set guide speed
    if (buf[2] == 's') {
	if (buf[3] == 'd') {
	    // dec speed
	    unsigned int duty = 99;

	    // ...1#
	    if (isdigit(buf[5]))
		duty = buf[7]-0x30;
	    // ..1.#
	    if (isdigit(buf[4]))
		duty = duty + ((buf[6]-0x30)*10);

	    dec_set_speed(duty);

	    return;
	}

	if (buf[3] == 'r') {
	    // ra speed
	    if (buf[4] == '1')
		ra_guide_speed_fast = 1;
	    else
	        ra_guide_speed_fast = DEFAULT_RA_GUIDE_SPEED;

	    return;
	}
    }
	
    // Tracking rate
    if (buf[2] == 'r') {
	// Siderial rate
	if (buf[3] == 'a') {
	    tracking_rate = TRACKING_RATE_SIDERIAL;
	    ra_siderial();
	}

	// lunar rate
	if (buf[3] == 'l') {
	    tracking_rate = TRACKING_RATE_LUNAR;
	    ra_lunar();
	}

	// solar rate
	if (buf[3] == 's') {
	    tracking_rate = TRACKING_RATE_SOLAR;
	    ra_solar();
	}

	// kings rate
	if (buf[3] == 'k') {
	    tracking_rate = TRACKING_RATE_KING;
	    ra_king();
	}

	// stop tracking
	if (buf[3] == 'h') {
	    tracking_rate = TRACKING_RATE_STOP;
	    ra_stop();
	}
	
	return;
    }

    // Status
    if (buf[2] == 'i') {
	send_str(PSTR("Tracking rate: "));
	if (tracking_rate == TRACKING_RATE_SIDERIAL)
	    send_str(PSTR("siderial"));
	else if (tracking_rate == TRACKING_RATE_LUNAR)
	    send_str(PSTR("lunar"));
	else if (tracking_rate == TRACKING_RATE_SOLAR)
	    send_str(PSTR("solar"));
	else if (tracking_rate == TRACKING_RATE_KING)
	    send_str(PSTR("king"));
	else
	    send_str(PSTR("stopped"));
	send_str(PSTR("\r\n"));

	send_str(PSTR("RA guide speed: "));
	if (ra_guide_speed_fast == 1)
	    send_str(PSTR("fast"));
	else
	    send_str(PSTR("normal"));
	send_str(PSTR("\r\n"));

	send_str(PSTR("DEC guide PWM duty: "));
	send_uint(dec_duty);
	send_str(PSTR(" %\r\n"));
	
	send_str(PSTR("DEC north guide ticks: "));
	send_ulint(dec_north_counter);
	send_str(PSTR("\r\nDEC south guide ticks: "));
	send_ulint(dec_south_counter);
	send_str(PSTR("\r\nRA east guide ticks: "));
	send_ulint(ra_east_counter);
	send_str(PSTR("\r\nRA west guide ticks: "));
	send_ulint(ra_west_counter);
	send_str(PSTR("\r\n"));
	
	return;
    }

    // Restart in bootloader mode
    if (buf[2] == 'b') {
	if (buf[3] == 'b')
	    restart_bootloader();
	
	if (buf[3] == 'n')
	    restart_normal();

	return;
    }

    send_str(PSTR("Unknown command\r\n"));
    return;
}
