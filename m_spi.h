#ifndef m_spi__
#define m_spi__

#include "m_general.h"
#include <stdlib.h>

#define  CS_LOW()  clear(PORTB, 0)
#define  CS_HIGH()  set(PORTB, 0)

void m_spi_init();
uint8_t read_spi_byte();
void write_spi_byte(uint8_t);
uint8_t exchange_spi_byte(uint8_t);

#endif
