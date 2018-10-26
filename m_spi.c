#include "m_spi.h"

uint8_t read_spi_byte();
void write_spi_byte(uint8_t);
uint8_t exchange_spi_byte(uint8_t);

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

uint8_t m_read_spi_register(uint8_t reg)
{
#ifndef  READ_FLAG
#define  READ_FLAG  0x80
#endif
  uint8_t response = 0xFF;
  SELECT_D1();
  write_spi_byte(reg | READ_FLAG);
  response = read_spi_byte();
  DESELECT_D1();
  return response;
}

void m_read_spi_registers(uint8_t start_reg, uint8_t count, uint8_t *dest)
{
#ifndef  READ_FLAG
#define  READ_FLAG  0x80
#endif
  uint8_t i;
  SELECT_D1();
  write_spi_byte(start_reg | READ_FLAG);
  for (i = 0; i < count; i++)
  {
    dest[i] = read_spi_byte();
  }
  DESELECT_D1();
}

void m_write_spi_register(uint8_t reg, uint8_t val)
{
#ifndef  WRITE_FLAG
#define  WRITE_FLAG  0x00
#endif
  SELECT_D1();
  write_spi_byte(reg | WRITE_FLAG);
  write_spi_byte(val);
  DESELECT_D1();
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
