#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "lcd.h"

#define BUTTON_UP PB4
#define BUTTON_DOWN PB2
#define USE_EEPROM 0
#define OUTPUT_PIN PB3
#define MAX_FREQ 1200
#define MIN_FREQ 800
#define DELTA_FREQ (MAX_FREQ - MIN_FREQ)

uint16_t EEMEM FAV_FREQ = 20;

void initLCD(void);
void updateLCD(uint16_t);
void updateFavourite(uint16_t);

static const PROGMEM uint8_t SYMBOL_LINE[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b11111,
	0b11111,
	0b00000,
	0b00000,
	0b00000
};


static const PROGMEM uint8_t SYMBOL_HALF_LINE[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b11100,
	0b11100,
	0b00000,
	0b00000,
	0b00000
};


static const PROGMEM uint8_t SYMBOL_ARROWS[8] = {
	0b00100,
	0b01110,
	0b11111,
	0b00000,
	0b00000,
	0b11111,
	0b01110,
	0b00100
};

static const PROGMEM uint8_t SYMBOL_HEART[8] = {
	0b00000,
	0b00000,
	0b01010,
	0b11111,
	0b11111,
	0b11111,
	0b01110,
	0b00100
};

static const uint8_t* SYMBOLS[] = {
	SYMBOL_ARROWS,
	SYMBOL_HEART,
	SYMBOL_LINE,
	SYMBOL_HALF_LINE,
	NULL
};

void initLCD(void){
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_command(_BV(LCD_CGRAM));
	for (int i = 0;SYMBOLS[i] != NULL;i++){
		for(int j = 0;j < 8;j++){
			lcd_data(pgm_read_byte_near(&SYMBOLS[i][j]));
		}
	}
}

void updateFavourite(uint16_t freq){
	// Check current, set EEPROM
	if (USE_EEPROM == 1){
		eeprom_update_word(&FAV_FREQ, freq);
	}

	// Flash LCD screen as output
	lcd_clrscr();
	PORTB |= (1 << OUTPUT_PIN);
	_delay_ms(300);
	PORTB &= ~(1 << OUTPUT_PIN);
	updateLCD(freq);
}
void updateLCD(uint16_t freq){
	// Set the PWM duty cycle to the appropriate frequency
	// TODO Set it to actual freq, rather than numerical value
	OCR1A = (freq) % 1024;

	//Init for a new draw
	lcd_clrscr();
	lcd_home();

	// Draw number
	lcd_gotoxy(7,1);
	lcd_putc(((freq / 1000) % 10) + '0');
	lcd_putc(((freq / 100)  % 10) + '0');
	lcd_putc(((freq / 10)   % 10) + '0');
	lcd_putc('.');
	lcd_putc(((freq / 1)    % 10) + '0');
	lcd_puts("MHz");
	lcd_putc(0);

	// Draw Heart
	if (freq == eeprom_read_word(&FAV_FREQ)){
		lcd_gotoxy(0,1);
		lcd_putc(1);
	}
}

int main(void) {
	// Set Buttons as inputs
	DDRB &= ~(1 << BUTTON_UP);

	// Set PWM pin as output
	DDRB |= (1 << OUTPUT_PIN);

	// Set up registers for PWM
	TCCR1A = (1 << COM1A1) | (1 << WGM12) | (1 << WGM11) | (1 << WGM10);
	OCR1A = 0x10;
	TCCR1B = (1 << CS10);
	
	initLCD();
	// Init variables
	uint8_t currentButtonState = 0;	
	uint8_t lastButtonState = 0;
	int8_t iter = 0;
	uint8_t config = 0;
	uint16_t freq = eeprom_read_word(&FAV_FREQ);

	// TODO
	DDRD |= (1 << PD6);

	// Grab the initial button state
	currentButtonState = (((PINB & (1 << BUTTON_UP)) != 0) << 1) + ((PINB & (1 << BUTTON_DOWN)) != 0);
	// And enter 'config' mode if both are pressed on startup
	if (currentButtonState == 0b11){
		// Notify user we are in config
		config = 1;
		lcd_clrscr();
		lcd_puts("Entered config");
		_delay_ms(3000);
	}

	while(1) {
		// Grab current button state
		currentButtonState = (((PINB & (1 << BUTTON_UP)) != 0) << 1) + ((PINB & (1 << BUTTON_DOWN)) != 0);
		PORTD &= ~(1 << PD6);
		PORTD |= (currentButtonState&0b01 << PD6);

		// If this isn't the first time we've had the button down
		// (To prevent button debouncing)
		if (lastButtonState != 0){
			// Increase our iteration counter if button continues in its current state
			if (currentButtonState == lastButtonState){
				iter++;
			}
	
			// Test for button down for 500ms (Freq Down)
			if (currentButtonState == 0b01 && iter > 20){
				iter -= 6;
				freq--;
			}

			// Test for button up for 500ms (Freq Up)
			if (currentButtonState == 0b10 && iter > 20){
				iter -= 6;
				freq++;
			}
			
			// Test for both held for 2500ms (Favourite)
			if (currentButtonState == 0b11 && iter > 20){
				if (!config){
					updateFavourite(freq);
				}
				iter = -10;
			}

			// Test for release
			if (currentButtonState == 0b00){
				// On release, fulfill last button press and reset counter
				if (iter > 0){
					if (lastButtonState == 0b01){
						freq--;
					}
					if (lastButtonState == 0b10){
						freq++;
					}
				}
				iter = 0;
			}
			
		}
		lastButtonState = currentButtonState;
		if (config) {
			// Essentially a custom updateLCD() func
			lcd_clrscr();
			lcd_puts("100Mhz value:");
			lcd_gotoxy(0,1);
			lcd_putc(((freq / 1000)% 10) + '0');
			lcd_putc(((freq / 100) % 10) + '0');
			lcd_putc(((freq / 10)  % 10) + '0');
			lcd_putc(((freq / 1)   % 10) + '0');
			lcd_putc(0);	
			freq = freq % 1024;
			OCR1A = freq;
			
		} else {
			updateLCD(freq);
		}
		_delay_ms(25);
	}
}
