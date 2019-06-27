#define F_CPU 12000000UL
    
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <u8g2.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "max31865.h"
#include "SPI.h"

#define DISPLAY_CLK_DIR DDRA
#define DISPLAY_CLK_PORT PORTA
#define DISPLAY_CLK_PIN 4

#define DISPLAY_DATA_DIR DDRA
#define DISPLAY_DATA_PORT PORTA
#define DISPLAY_DATA_PIN 3

#define DISPLAY_CS_DIR DDRA
#define DISPLAY_CS_PORT PORTA
#define DISPLAY_CS_PIN 0

#define DISPLAY_DC_DIR DDRA
#define DISPLAY_DC_PORT PORTA
#define DISPLAY_DC_PIN 1

#define DISPLAY_RESET_DIR DDRA
#define DISPLAY_RESET_PORT PORTA
#define DISPLAY_RESET_PIN 2

#define P_CPU_NS (1000000000UL / F_CPU)

#define BUTTONS_PORT PORTD
#define BUTTONS_DDR DDRD
#define BUTTONS_PIN PIND
#define BUTTON_RU 1
#define BUTTON_LU 2
#define BUTTON_LD 3
#define BUTTON_RD 4

#define LEVEL_DDR DDRC
#define LEVEL_PORT PORTC
#define LEVEL_PIN 2
#define LEVEL_VALUE PINC & (1<<LEVEL_PIN)

#define GEAR_PORT PORTC
#define GEAR_DDR DDRC
#define GEAR_HEATER_PIN 0
#define GEAR_FAN_PIN 1

u8g2_t u8g2;
float EEMEM destTemperatureEEMEM = 24;
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;

uint8_t u8x8_avr_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	uint8_t cycles;

	switch(msg)
	{
		case U8X8_MSG_DELAY_NANO:     // delay arg_int * 1 nano second
			// At 20Mhz, each cycle is 50ns, the call itself is slower.
			break;
		case U8X8_MSG_DELAY_100NANO:    // delay arg_int * 100 nano seconds
			// Approximate best case values...
#define CALL_CYCLES 26UL
#define CALC_CYCLES 4UL
#define RETURN_CYCLES 4UL
#define CYCLES_PER_LOOP 4UL

			cycles = (100UL * arg_int) / (P_CPU_NS * CYCLES_PER_LOOP);

			if(cycles > CALL_CYCLES + RETURN_CYCLES + CALC_CYCLES) 
				break;

			__asm__ __volatile__ (
			"1: sbiw %0,1" "\n\t" // 2 cycles
			"brne 1b" : "=w" (cycles) : "0" (cycles) // 2 cycles
			);
			break;
		case U8X8_MSG_DELAY_10MICRO:    // delay arg_int * 10 micro seconds
			for(int i=0 ; i < arg_int ; i++)
				_delay_us(10);
			break;
		case U8X8_MSG_DELAY_MILLI:      // delay arg_int * 1 milli second
			for(int i=0 ; i < arg_int ; i++)
				_delay_ms(1);
			break;
		default:
			return 0;
	}
	return 1;
}


uint8_t u8x8_avr_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	// Re-use library for delays

	switch(msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:  // called once during init phase of u8g2/u8x8
			DISPLAY_CLK_DIR |= 1<<DISPLAY_CLK_PIN;
			DISPLAY_DATA_DIR |= 1<<DISPLAY_DATA_PIN;
			DISPLAY_CS_DIR |= 1<<DISPLAY_CS_PIN;
			DISPLAY_DC_DIR |= 1<<DISPLAY_DC_PIN;
			DISPLAY_RESET_DIR |= 1<<DISPLAY_RESET_PIN;
			break;              // can be used to setup pins
		case U8X8_MSG_GPIO_SPI_CLOCK:        // Clock pin: Output level in arg_int
			if(arg_int)
				DISPLAY_CLK_PORT |= (1<<DISPLAY_CLK_PIN);
			else
				DISPLAY_CLK_PORT &= ~(1<<DISPLAY_CLK_PIN);
			break;
		case U8X8_MSG_GPIO_SPI_DATA:        // MOSI pin: Output level in arg_int
			if(arg_int)
				DISPLAY_DATA_PORT |= (1<<DISPLAY_DATA_PIN);
			else
				DISPLAY_DATA_PORT &= ~(1<<DISPLAY_DATA_PIN);
			break;
		case U8X8_MSG_GPIO_CS:        // CS (chip select) pin: Output level in arg_int
			if(arg_int)
				DISPLAY_CS_PORT |= (1<<DISPLAY_CS_PIN);
			else
				DISPLAY_CS_PORT &= ~(1<<DISPLAY_CS_PIN);
			break;
		case U8X8_MSG_GPIO_DC:        // DC (data/cmd, A0, register select) pin: Output level in arg_int
			if(arg_int)
				DISPLAY_DC_PORT |= (1<<DISPLAY_DC_PIN);
			else
				DISPLAY_DC_PORT &= ~(1<<DISPLAY_DC_PIN);
			break;
		
		case U8X8_MSG_GPIO_RESET:     // Reset pin: Output level in arg_int
			if(arg_int)
				DISPLAY_RESET_PORT |= (1<<DISPLAY_RESET_PIN);
			else
				DISPLAY_RESET_PORT &= ~(1<<DISPLAY_RESET_PIN);
			break;
		default:
			if (u8x8_avr_delay(u8x8, msg, arg_int, arg_ptr))	// check for any delay msgs
				return 1;
			u8x8_SetGPIOResult(u8x8, 1);      // default return value
			break;
	}
	return 1;
}

ISR(TIMER1_OVF_vect)
{
	seconds++;
	
	if(seconds == 60)
	{
		seconds = 0;
		minutes++;
	}
	if(minutes == 60)
	{
		minutes = 0;
		hours++;
	}
	if(hours > 23)
		hours = 0;
	TCNT1 = 53817;
}


int main(void)
{	
	float temperature;
	float destTemperature = eeprom_read_float(&destTemperatureEEMEM);
	
	uint8_t heater = 0;
	uint8_t fan    = 1;
	uint8_t level  = 0;
	char strTemperature[5]     = {0};
	char strDestTemperature[5] = {0};
	char strTime[6] = {0};
	
	TCCR1B |= (1<<CS12)|(1<<CS10);//Предделитель = 1024
	TIMSK |= (1<<TOIE1);//Разрешить прерывание по переполнению таймера 1
	TCNT1 = 53817;
	sei();

	BUTTONS_PORT |= (1<<BUTTON_LU)|(1<<BUTTON_RU)|(1<<BUTTON_LD)|(1<<BUTTON_RD);
	BUTTONS_DDR  &= ~((1<<BUTTON_LU)|(1<<BUTTON_RU)|(1<<BUTTON_LD)|(1<<BUTTON_RD));
	
	GEAR_PORT &= ~((1<<GEAR_FAN_PIN)|(1<<GEAR_HEATER_PIN));
	GEAR_DDR |= (1<<GEAR_FAN_PIN)|(1<<GEAR_HEATER_PIN);
	
	LEVEL_PORT |= (1<<LEVEL_PIN);
	LEVEL_DDR &= ~(1<<LEVEL_PIN);
	
	DISPLAY_CS_PORT &= ~(1<<DISPLAY_CS_PIN);
	SPI_Init();
	/*
		Select a setup procedure for your display from here: https://github.com/olikraus/u8g2/wiki/u8g2setupc
		1. Arg: Address of an empty u8g2 structure
		2. Arg: Usually U8G2_R0, others are listed here: https://github.com/olikraus/u8g2/wiki/u8g2reference#carduino-example
		3. Arg: Protocol procedure (u8g2-byte), list is here: https://github.com/olikraus/u8g2/wiki/Porting-to-new-MCU-platform#communication-callback-eg-u8x8_byte_hw_i2c
		4. Arg: Defined in this code itself (see above)
	*/
	u8g2_Setup_ssd1306_128x64_noname_2( &u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8x8_avr_gpio_and_delay );
	
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	DISPLAY_CS_PORT |= (1<<DISPLAY_CS_PIN);	
	max31865_setup();
		
	while(1){
		temperature = max31865_temperature();
		if( temperature > 0 ){
			dtostrf(temperature, 4, 1, strTemperature);
		}else{
			strcpy(strTemperature, "000");	
		}
		
		if( (BUTTONS_PIN & (1<<BUTTON_RU)) == 0 ){
			destTemperature += .1;
			if( destTemperature > 99 ) destTemperature = 99;
			eeprom_update_float(&destTemperatureEEMEM, destTemperature);
		}
		if( (BUTTONS_PIN & (1<<BUTTON_RD)) == 0 ){
			destTemperature -= .1;
			if( destTemperature < 10 ) destTemperature = 10;
			eeprom_update_float(&destTemperatureEEMEM, destTemperature);
		}
		heater = destTemperature > temperature ? 1 : 0;
		
		level = (BUTTONS_PIN & (1<<BUTTON_LD)) == 0;
		if( level ){//LEVEL_VALUE == 0 ){
			heater = 0;
			fan = 0;
			GEAR_PORT &= ~(1<<GEAR_FAN_PIN);
		}else{
			fan = 1;
			GEAR_PORT |= (1<<GEAR_FAN_PIN);
		}

		if( heater ){
			GEAR_PORT |= (1<<GEAR_HEATER_PIN);
			}else{
			GEAR_PORT &= ~(1<<GEAR_HEATER_PIN);
		}
		dtostrf(destTemperature, 4, 1, strDestTemperature);
		
		strTime[0] = '0' + hours/10;
		strTime[1] = '0' + hours%10;
		strTime[2] = seconds%2 ? ':' : ' ';
		strTime[3] = '0' + minutes/10;
		strTime[4] = '0' + minutes%10;
		strTime[5] = 0;
		
    	DISPLAY_CS_PORT &= ~(1<<DISPLAY_CS_PIN);
		u8g2_FirstPage(&u8g2);
		do 
		{			
			u8g2_SetDrawColor(&u8g2, 1);
			u8g2_ClearBuffer(&u8g2); 
			u8g2_SetFont(&u8g2, u8g2_font_logisoso30_tn);
			u8g2_DrawStr(&u8g2, 1, 35, strTemperature);
			u8g2_SetFont(&u8g2, u8g2_font_crox4hb_tn );
			u8g2_DrawStr(&u8g2, 85, 21, strDestTemperature);

			u8g2_DrawStr(&u8g2, 2, 64, strTime);

			u8g2_SetFont(&u8g2, u8g2_font_open_iconic_embedded_2x_t );
			if( heater != 0 ){
				u8g2_DrawStr(&u8g2, 65, 64, "\x43");
			}
			if( fan != 0 ){
				u8g2_DrawStr(&u8g2, 86, 64, "\x4f");
			}
			if( level == 0 ){
				u8g2_DrawStr(&u8g2, 110, 64, "\x4c");
			}
						
			u8g2_DrawLine(&u8g2, 3, 40, 125, 40);
			u8g2_DrawLine(&u8g2, 72, 2, 78, 20);
			u8g2_DrawLine(&u8g2, 72, 38, 78, 20);
			u8g2_SendBuffer(&u8g2);
			//u8g2_SetDrawColor(&u8g2, 0);
			//u8g2_DrawBox(&u8g2, 80, 18, 90, 10);
		} while (u8g2_NextPage(&u8g2));
	    DISPLAY_CS_PORT |= (1<<DISPLAY_CS_PIN);
	}
}