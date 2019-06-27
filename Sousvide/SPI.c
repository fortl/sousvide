#include <avr/io.h>

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB
#define SPI_PINX PINB

#define SPI_SCK 7
#define SPI_MISO 6 
#define SPI_MOSI 5

/*инициализация SPI модуля в режиме master*/
void SPI_Init(void)
{
   /*настройка портов ввода-вывода
   все выводы, кроме MISO выходы*/
   SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(0<<SPI_MISO);
   SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_MISO);

   /*разрешение spi,старший бит вперед,мастер, режим 0*/
   SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(1<<CPHA)|(1<<SPR1)|(1<<SPR0);
   SPSR = (1<<SPI2X);
}

void SPI_WriteByte(uint8_t data)
{
   unsigned char cnt = 8;
   /*SPDR = data;
   while(!(SPSR & (1<<SPIF)));
   */
   while (cnt--) {
      if (data & 0x80) SPI_PORTX |= (1<<SPI_MOSI);
      else SPI_PORTX &= ~(1<<SPI_MOSI);
      SPI_PORTX |= (1<<SPI_SCK);
      data <<= 1;
      SPI_PORTX &= ~(1<<SPI_SCK);
   }
   return;
}

uint8_t SPI_ReadByte(uint8_t data)
{
   SPDR = data;
   while(!(SPSR & (1<<SPIF)));
   data = SPDR;
   
   return data; 
}
