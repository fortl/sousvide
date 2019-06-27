#define F_CPU 12000000UL
#define TIMER_COUNTER 46786 
// <16 bit timer max value> - F_CPU/(<делитель>*<доли секунды>) 65536 - 12000000/(64*10) 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <u8g.h>
#include <avr/eeprom.h> 

u8g_t u8g;

#define BTN_COUNT 3 // 0 .. 2
#define BTN_PIN PINA
#define BTN_PORT PORTA
#define BTN_RIGHT PORTA5
#define BTN_CENTER PORTA6
#define BTN_LEFT PORTA7

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB
#define SPI_MOSI 5
#define SPI_MISO 6
#define SPI_SS 4
#define SPI_SCK 7

#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define USART_CAN_READ (UCSRA & (1<<RXC))
void USART_Init(void);
unsigned char USART_Receive(void);
void USART_Transmit( unsigned char data);

char timerHours = 1;
char timerMinutes = 0;
char timerSeconds = 0;
char timerDSeconds = 0;

char power;
uint16_t deltaTemp;
uint16_t temperature;
uint16_t EEMEM destTemperatureStore = 35 << 2;
uint16_t destTemperature;
uint8_t button_counter[BTN_COUNT];

/*инициализация SPI модуля в режиме master*/
void SPI_Init(void)
{
   /*настройка портов ввода-вывода
   все выводы, кроме MISO выходы*/
   SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);
   SPI_DDRX &= ~(0<<SPI_MISO);
   SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS)|(1<<SPI_MISO);
 
   /*разрешение spi,старший бит вперед,мастер, режим 0*/
   SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
   SPSR = (0<<SPI2X);
   
//   SPI_PORTX &= ~(1<<SPI_SS);
//   _delay_ms(100);
//   SPI_PORTX |= (1<<SPI_SS);
}

void SPI_WriteByte(uint8_t data)
{
	SPI_PORTX &= ~(1<<SPI_SS);
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	SPI_PORTX |= (1<<SPI_SS);
}

uint8_t SPI_ReadByte(uint8_t data)
{
	uint8_t report;
	SPI_PORTX &= ~(1<<SPI_SS);
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	report = SPDR;
	SPI_PORTX |= (1<<SPI_SS);
	return report;
}

void SPI_WriteArray(uint8_t num, uint8_t *data)
{
	SPI_PORTX &= ~(1<<SPI_SS);
	while(num--){
		SPDR = *data++;
		while(!(SPSR & (1<<SPIF)));
	}
	SPI_PORTX |= (1<<SPI_SS);
}


void SPI_ReadArray(uint8_t num, uint8_t *data)
{
	SPI_PORTX &= ~(1<<SPI_SS);
	_delay_ms(10);
	while(num--){
		SPDR = *data;
		while(!(SPSR & (1<<SPIF)));
		*data++ = SPDR;
	}
	SPI_PORTX |= (1<<SPI_SS);
	_delay_ms(170);
}

void USART_Init(void) //Функция инициализации USART
{
	UBRRH = (uint8_t) (UBRR_VALUE>>8); // Set baudrate
	UBRRL = (uint8_t) UBRR_VALUE;
	UCSRB = (1<<RXEN) | (1<<TXEN); //Разрешение на прием и н апередачу через USART
	UCSRC = (1<<URSEL) | (1<<USBS) | (3<<UCSZ0); // Set frame format to 8 data bits, no parity, 1 stop bit
}

unsigned char USART_Receive( void ) //Функция приема данных
{
	while ( !USART_CAN_READ );     //Ожидание приема символа
	return UDR; //Возврат символа
}

void USART_Transmit( unsigned char data ) //Функция отправки данных
{
	while ( !(UCSRA & (1<<UDRE)) ); //Ожидание опустошения буфера приема
	UDR = data; //Начало передачи данных
}

void fixPointToStr(uint16_t j, char * str)
{
	itoa(j>>2, str, 10);
	str[2] = '.'; 
	if( j&1 )
	{
		str[3] = j&2 ? '7' : '2';
		str[4] = '5';
	} else {
		str[3] = j&2 ? '5' : '0';
		str[4] = '0';
	}		
	str[5] = '\0';
}

void draw(void)
{
	char str[8] = "\0\0\0\0\0\0\0\0";

		PORTA &= ~(1<<PORTA2);

		u8g_FirstPage(&u8g);
		do
		{
			u8g_SetColorIndex(&u8g, 1);
			u8g_SetFont(&u8g, u8g_font_courR14);
			
			str[0] = '0';
			itoa(timerHours, timerHours < 9 ? &str[1] : str, 10);

			str[2] = timerSeconds & 1 ? ':' : ' ';

			str[3] = '0';
			itoa(timerMinutes, timerMinutes < 9 ? &str[4] : &str[3], 10);
			str[5] = '\0';
			
			u8g_DrawStr(&u8g, 0, 62, "TIME");
			u8g_DrawStr(&u8g, 60, 62, str);

			fixPointToStr(temperature, str);
			u8g_DrawStr(&u8g, 60, 16, str);
			u8g_DrawStr(&u8g, 115, 16, "C");

			itoa(power*20, str, 10);
			u8g_DrawStr(&u8g, 60, 40, str);
			u8g_DrawStr(&u8g, 115, 40, "%");
			
			u8g_SetFont(&u8g, u8g_font_fub35n);
			//u8g_DrawFrame(&u8g, 0, 0, 127, 21);
			//u8g_SetColorIndex(&u8g, 0);
			//itoa(destTemperature, str, 10);
			itoa(destTemperature>>2, str, 10);
			//fixPointToStr(destTemperature, str);
			u8g_DrawStr(&u8g, 0, 40, str);
		} while ( u8g_NextPage(&u8g) );
		//u8g_Delay(100);
		
		PORTA |= 1<<PORTA2;
}

ISR( TIMER1_OVF_vect )
{
	TCNT1 = TIMER_COUNTER; //выставляем начальное значение TCNT1

	// 000 001 010 011 100 101 110 111 1000 1001
	if( button_counter[0] )
	{
		destTemperature -= 
				button_counter[0] == 1 ? 1
			: button_counter[0] < 8 ? 0
			: button_counter[0] < 16 ? ( (timerDSeconds & 3) == 0?1:0)
			: button_counter[0] < 32 ? ( (timerDSeconds & 1) == 0?1:0)
			: button_counter[0] >> 5;
		eeprom_write_word(&destTemperatureStore, destTemperature);
	}
				
	if( button_counter[2] )
	{
		destTemperature +=
		button_counter[2] == 1 ? 1
		: button_counter[2] < 8 ? 0
		: button_counter[2] < 16 ? ( (timerDSeconds & 3) == 0?1:0)
		: button_counter[2] < 32 ? ( (timerDSeconds & 1) == 0?1:0)
		: button_counter[2] >> 5;
		eeprom_write_word(&destTemperatureStore, destTemperature);
	}
	
	if( destTemperature < 40 ) 
	{
		destTemperature = 40;
	} else if( destTemperature > 396 )
	{
		destTemperature = 396;
	}
	
	if( timerDSeconds > 5 ){
		PORTC |= 1<<PORTC0;	
	}else{
		PORTC &= ~(1<<PORTC0);
	}
		
/*	if( power == 5 ){
//		PORTC |= 1<<PORTC1;
		PORTC |= 1<<PORTC0;
//		PORTC &= ~(1<<PORTC0);
	}else if( power > (timerDSeconds % 5) ){
		PORTC |= 1<<PORTC0;
//		PORTC &= ~(1<<PORTC1);
	}else{
		PORTC &= ~(1<<PORTC0);
//		PORTC &= ~(1<<PORTC1);
	}*/

	if ( timerDSeconds > 0 )
	{
		timerDSeconds--;
		for( char i = 0; i < BTN_COUNT ; i++ )
		{
			if( button_counter[i] > 0 && button_counter[i] < 255 )
			{
				 button_counter[i]++;
			}
		}
	} else {
		timerDSeconds = 9;

		if( destTemperature <= temperature ){
			power = 0;
		} else {
			deltaTemp = destTemperature - temperature;
				
			if( deltaTemp > 4 ){
				power = 5;
			} else if( deltaTemp > 8 ) {
				power = 3;
			}
		}


		if ( timerSeconds > 0 ){
			timerSeconds--;
		} else {
			timerSeconds = 59;
			if ( timerMinutes > 0 )
			{
				timerMinutes--;
			} else {
				timerMinutes = 59;
				if ( timerHours > 0 ){
					timerHours--;		
				} else {
					timerHours = 1; //TODO
				}
			}
		}
	}
	
	if( (timerDSeconds & 1) == 0 ){ draw(); }
}

int main(void) {	
	//DDRA = 0;
	//BTN_PORT = (1<<BTN_RIGHT)|(1<<BTN_LEFT)|(1<<BTN_CENTER);
	
	DDRC |= (1<<PORTC0)|(1<<PORTC1);
	PORTC |= (1<<PORTC0)|(1<<PORTC1);

	destTemperature = eeprom_read_word(&destTemperatureStore);
	
	TCCR1B = (0<<CS12)|(1<<CS11)|(1<<CS10); // делитель 64
	TIMSK |= (1<<TOIE1); // разрешаем прерывание по переполнению таймера
	TCNT1 = TIMER_COUNTER;        // выставляем начальное значение TCNT1
	sei();                // выставляем бит общего разрешения прерываний
	
	//SPI_Init();

	//u8g_InitSPI(&u8g, &u8g_dev_ssd1306_128x64_sw_spi, PN(0, 3), PN(0, 4), PN(0, 0), PN(0, 1), PN(0, 2));
	//u8g_InitHWSPI(&u8g, &u8g_dev_ssd1306_128x64_hw_spi, PN(0, 0), PN(0, 1), PN(0, 2));

//	USART_Init();// Скорость соединения 9600 бит/с
	
	uint8_t temperature_buf[2];
	
	for ( ; ; ) {
		
		temperature_buf[1] = temperature_buf[0] = 0xff;
		//SPI_ReadArray(2,  temperature_buf);
		//temperature = (temperature_buf[0]<<8 | temperature_buf[1]) >> 3;
		temperature = 50;

		char buttons = BTN_PIN;
		if( buttons & (1<<BTN_LEFT) )
		{
			button_counter[0] = 0;
		} else if( !button_counter[0] ) {
			button_counter[0] = 1;
		}

		if( buttons & (1<<BTN_CENTER) )
		{
			button_counter[1] = 0;
		} else if( !button_counter[1] ) {
			button_counter[1] = 1;
		}
		
		if( buttons & (1<<BTN_RIGHT) )
		{
			button_counter[2] = 0;
		} else if( !button_counter[2] ) {
			button_counter[2] = 1;
		}
			
		//strcat(str, "t");

		
		/*
		USART_Transmit(temperature_buf[0]);
		USART_Transmit(temperature_buf[1]);
		*/
		//USART_Transmit(str[0]);
		//USART_Transmit(str[1]);
//USART_Transmit(10); //Отправка символа
//USART_Transmit(13); //Отправка символа
		/*_delay_ms(1000);
		if(USART_CAN_READ){
			act = USART_Receive();
			if(act == 49){
				PORTC |= 1<<PORTC0;
			}else if(act == 48){			
				PORTC &= ~(1<<PORTC0);
			}
		}
		*/
	}

}
