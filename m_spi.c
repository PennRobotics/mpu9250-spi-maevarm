#include "m_spi.h"

void m_spi_init()
{
  // PIN CONFIG
  clear(PRR0, PRSPI);  // disable SPI power reduction
  set(DDRB, 0);  // CS output
  set(DDRB, 1);  // SCLK output
  set(DDRB, 2);  // MOSI output
  clear(DDRB, 3);  // MISO input
  clear(PORTB, 3);  // MISO pull-up disabled
  set(PORTB, 2);  // MOSI starts high
  clear(PORTB, 1);  // SCLK starts low
  CS_HIGH();  // CS starts high

  // SPEED CONFIG (250kHz)
  clear(SPCR, SPR0);
  set(SPCR, SPR1);
  clear(SPSR, SPI2X);

  // ENABLE IN MASTER MODE
  set(SPCR, SPE);
  set(SPCR, MSTR);
}

uint8_t read_spi_byte()
{
  SPDR = 0xFF;
  while(!check(SPSR, SPIF));
  clear(SPSR, SPIF);
  return SPDR;
}

void write_spi_byte(uint8_t byte)
{
  SPDR = byte;
  while(!check(SPSR, SPIF));
  clear(SPSR, SPIF);
}

uint8_t exchange_spi_byte(uint8_t byte)
{
  SPDR = byte;
  while(!check(SPSR, SPIF));
  clear(SPSR, SPIF);
  return SPDR;
}
