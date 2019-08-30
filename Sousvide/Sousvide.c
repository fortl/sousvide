#define F_CPU 12000000UL
    
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <u8g2.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "max31865.h"
#include "SPI.h"

#define DISPLAY_CLK_DIR DDRA
#define DISPLAY_CLK_PORT PORTA
#define DISPLAY_CLK_PIN 0

#define DISPLAY_DATA_DIR DDRA
#define DISPLAY_DATA_PORT PORTA
#define DISPLAY_DATA_PIN 1

#define DISPLAY_RESET_DIR DDRA
#define DISPLAY_RESET_PORT PORTA
#define DISPLAY_RESET_PIN 2

#define DISPLAY_DC_DIR DDRA
#define DISPLAY_DC_PORT PORTA
#define DISPLAY_DC_PIN 3

#define DISPLAY_CS_DIR DDRA
#define DISPLAY_CS_PORT PORTA
#define DISPLAY_CS_PIN 4

#define P_CPU_NS (1000000000UL / F_CPU)

#define BUTTONS_PORT PORTD
#define BUTTONS_DDR DDRD
#define BUTTONS_PIN PIND
#define BUTTON_FIRST 2
#define BUTTON_RU 3
#define BUTTON_RD 2
#define BUTTON_LD 4
#define BUTTON_LU 5
#define BUTTON_LAST 5

#define LEVEL_DDR DDRC
#define LEVEL_PORT PORTC
#define LEVEL_PIN 1
#define LEVEL_VALUE PINC & (1<<LEVEL_PIN) ? 1 : 0

#define GEAR_PORT PORTC
#define GEAR_DDR DDRC
#define GEAR_HEATER_PIN 6
#define GEAR_FAN_PIN 0

#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define USART_CAN_READ (UCSRA & (1<<RXC))
#define BT_BUFFER_SIZE 8

u8g2_t u8g2;
float EEMEM destTemperatureEEMEM = 24;

uint8_t EEMEM destHoursEEMEM = 4;
uint8_t EEMEM destMinutesEEMEM = 0;
uint8_t EEMEM destSecontsEEMEM = 0;
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
uint8_t heater = 0;
uint8_t fan    = 1;
uint8_t level  = 0;
	
char strTemperature[5]     = {0};
char strDestTemperature[5] = {0};
char strTime[6] = {0};

uint8_t buttonsCounters[BUTTON_LAST+1] = {0};

// The inputted commands are never going to be
// more than 8 chars long. Volatile for the ISR.
volatile unsigned char data_in[8];
volatile unsigned char command_in[8];

volatile unsigned char data_count;
volatile unsigned char command_ready;


void USART_Init(void) //Функция инициализации USART
{
	UBRRH = (uint8_t) (UBRR_VALUE>>8); // Set baudrate
	UBRRL = (uint8_t) UBRR_VALUE;
	UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE); //Разрешение на прием и передачу через USART
	UCSRC = (1<<URSEL) | (0<<USBS) | (3<<UCSZ0); // Set frame format to 8 data bits, no parity, 1 stop bit
}
#define USART_READ UDR 

void USART_Transmit( unsigned char data ) //Функция отправки данных
{
	while ( !(UCSRA & (1<<UDRE)) ); //Ожидание опустошения буфера приема
	UDR = data; //Начало передачи данных
}unsigned char USART_Read(void){	while( !USART_CAN_READ );	return USART_READ;}void setBluetoothName(void){	_delay_us(500);
	USART_Transmit('A');
	USART_Transmit('T');
	USART_Transmit('+');
	USART_Transmit('N');
	USART_Transmit('A');
	USART_Transmit('M');
	USART_Transmit('E');
	USART_Transmit('S');
	USART_Transmit('o');
	USART_Transmit('u');
	USART_Transmit('s');
	USART_Transmit('v');
	USART_Transmit('i');
	USART_Transmit('d');
	USART_Transmit('e');
	USART_Transmit(13);
		_delay_us(500); }void copy_command ()
{
	// The USART might interrupt this - don't let that happen!
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		// Copy the contents of data_in into command_in
		for (int i=0; i<7; i++) command_in[i] = data_in[i];
	}
}
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

void display(void)
{
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
		if( level != 0 || (seconds & 1) == 0 ){
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

ISR(TIMER1_OVF_vect)
{	
	if(seconds > 0){
		seconds--;
	}else{
		seconds = 59;
		if(minutes > 0){
			minutes--;
		}else{
			minutes = 59;
			if(hours > 0) hours -= 1;
		}
	}

	display();
	eeprom_update_byte(&destSecontsEEMEM, seconds);
	eeprom_update_byte(&destMinutesEEMEM, minutes);
	eeprom_update_byte(&destHoursEEMEM, hours);
	
	TCNT1 = 53817;
}

ISR(USART_RXC_vect)
{
	if (command_ready == 1) return;
	// Get data from the USART in register
	data_in[data_count] = UDR;

	// End of line!
	if (data_count > 7 || data_in[data_count] == '\n') {
		command_ready = 1;
		// Reset to 0, ready to go again
		data_count = 0;
	} else {
		data_count++;
	}
}


int main(void)
{	
	uint8_t sensorFault = -1;
	float temperature = 0;
	uint8_t secondsCounterTime = 0;
	float temperatureJumpInterval;
	uint8_t timeJumpInterval;
	float destTemperature = eeprom_read_float(&destTemperatureEEMEM);
	
	_delay_us(50000);
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
	u8g2_SetDisplayRotation(&u8g2, U8G2_R2);
	DISPLAY_CS_PORT |= (1<<DISPLAY_CS_PIN);	
	max31865_setup();
	
	seconds = eeprom_read_byte(&destSecontsEEMEM);
	minutes = eeprom_read_byte(&destMinutesEEMEM);
	hours   = eeprom_read_byte(&destHoursEEMEM);
	
	TCCR1B |= (1<<CS12)|(1<<CS10);//Предделитель = 1024
	TIMSK |= (1<<TOIE1);//Разрешить прерывание по переполнению таймера 1
	TCNT1 = 53817;
	sei();
	
	USART_Init();

	BUTTONS_PORT |= (1<<BUTTON_LU)|(1<<BUTTON_RU)|(1<<BUTTON_LD)|(1<<BUTTON_RD);
	BUTTONS_DDR  &= ~((1<<BUTTON_LU)|(1<<BUTTON_RU)|(1<<BUTTON_LD)|(1<<BUTTON_RD));
	
	GEAR_PORT &= ~((1<<GEAR_FAN_PIN)|(1<<GEAR_HEATER_PIN));
	//GEAR_PORT |= ((1<<GEAR_FAN_PIN)|(1<<GEAR_HEATER_PIN));
	GEAR_DDR |= (1<<GEAR_FAN_PIN)|(1<<GEAR_HEATER_PIN);
	
	LEVEL_PORT |= (1<<LEVEL_PIN);
	LEVEL_DDR &= ~(1<<LEVEL_PIN);
		
	while(1){
		if (command_ready == 1) {
			copy_command();
			command_ready = 0;
			if( command_in[0] == 'S' ){
				if( command_in[1] == 'T' ){
					destTemperature = (float)(command_in[2] - '0')*10;
					destTemperature += (command_in[3] - '0');
					destTemperature += (float)(command_in[5] - '0')/10;
					eeprom_update_float(&destTemperatureEEMEM, destTemperature);
				}else if ( command_in[1] == 'S' ){
					hours   = (command_in[2] - '0')*10 + (command_in[3] - '0');
					minutes = (command_in[5] - '0')*10 + (command_in[6] - '0');
				}
			}
		}
		
		for(uint8_t i = BUTTON_FIRST; i <= BUTTON_LAST; i++){
			buttonsCounters[i] = (BUTTONS_PIN & (1<<i)) == 0
			? (buttonsCounters[i] < 255 ? buttonsCounters[i]+1 : 255)
			: 0;
		}		
		
		if( seconds != secondsCounterTime ){
			sensorFault = max31865_readFault();
			temperature = max31865_temperature();
			if( buttonsCounters[BUTTON_RU] > 0 ){
				if( buttonsCounters[BUTTON_RU] < 20 ) {
					temperatureJumpInterval = .1;
				}else if( buttonsCounters[BUTTON_RU] < 40 ){
					temperatureJumpInterval = 1;
				}else if( buttonsCounters[BUTTON_RU] < 100 ){
					temperatureJumpInterval = 5;
				}else{
					temperatureJumpInterval = 10;
				}
				destTemperature = round((destTemperature / temperatureJumpInterval) + 1)*temperatureJumpInterval;
				if( destTemperature > 99 ) destTemperature = 99;
				eeprom_update_float(&destTemperatureEEMEM, destTemperature);
			}
			if( buttonsCounters[BUTTON_RD] > 0 ){
				if( buttonsCounters[BUTTON_RD] < 20 ) {
					temperatureJumpInterval = .1;
				}else if( buttonsCounters[BUTTON_RD] < 40 ){
					temperatureJumpInterval = 1;
				}else if( buttonsCounters[BUTTON_RD] < 100 ){
					temperatureJumpInterval = 5;
				}else{
					temperatureJumpInterval = 10;
				}
				
				destTemperature = round((destTemperature / temperatureJumpInterval) - 1 )*temperatureJumpInterval;
				if( destTemperature < 10 ) destTemperature = 10;
				eeprom_update_float(&destTemperatureEEMEM, destTemperature);
			}
			if( buttonsCounters[BUTTON_LU] > 0 ){
				minutes += buttonsCounters[BUTTON_LU] > 40 ? 30 : 10;
				if( minutes > 60 ){
					minutes -=60;
					hours += 1;
					if( hours > 99 ) hours = 99;
				}
			}
			if( buttonsCounters[BUTTON_LD] > 0 ){
				timeJumpInterval = buttonsCounters[BUTTON_LD] > 4 ? 30 : 10;
				if( minutes >= timeJumpInterval ){
					minutes -= timeJumpInterval;
					}else{
					if( hours > 0 ){
						minutes += 60 - timeJumpInterval;
						hours -= 1;
						}else{
						minutes = 0;
					}
				}
			}
			secondsCounterTime = seconds;
		}
		if( sensorFault ){
			strcpy(strTemperature, "---");
		}else if( temperature > 0 ){
			dtostrf(temperature, 4, 1, strTemperature);
		}else{
			strcpy(strTemperature, "000");
		}
		for(uint8_t i = 0; i <= 3; i++) USART_Transmit(strTemperature[i]);
		USART_Transmit(20);
		
		level = LEVEL_VALUE;
		heater = (level != 0) &&
		 (!sensorFault && temperature != 0 && (destTemperature > temperature)) ? 1 : 0;
		 
		if( level != 0 ){
			fan = 1;
			GEAR_PORT |= (1<<GEAR_FAN_PIN);
			//GEAR_PORT |= (1<<GEAR_HEATER_PIN);
		}else{
			fan = 0;
			GEAR_PORT &= ~(1<<GEAR_FAN_PIN);
			//GEAR_PORT &= ~(1<<GEAR_HEATER_PIN);
		}
		USART_Transmit('0'+level);
		USART_Transmit(20);

		if( heater != 0 ){
			GEAR_PORT |= (1<<GEAR_HEATER_PIN);
		}else{
			GEAR_PORT &= ~(1<<GEAR_HEATER_PIN);
		}
		USART_Transmit('0'+level);
		USART_Transmit(20);

		dtostrf(destTemperature, 4, 1, strDestTemperature);
		for(uint8_t i = 0; i <= 3; i++) USART_Transmit(strDestTemperature[i]);
		USART_Transmit(20);
		
		strTime[0] = '0' + hours/10;
		strTime[1] = '0' + hours%10;
		strTime[2] = seconds%2 ? ':' : ' ';
		strTime[3] = '0' + minutes/10;
		strTime[4] = '0' + minutes%10;
		strTime[5] = 0;

		for(uint8_t i = 0; i <= 4; i++) USART_Transmit(strTime[i]);
		USART_Transmit(20);
		USART_Transmit(10);
		_delay_us(50000);
	}
}