#include "m_general.h"
#include "mpu9250.h"
#include "m_ss.h"

int main() {
  init_spi();

  init_mpu();

  init_ak();

  init_usb();

  for(;;) {
    _delay_ms(100);
    uint8_t i;
    for (i = 0; i < 10; i++) {
      uint8_t value_hi = read_uint8_via_spi(MPU9250_SPI_ADDR, REG_AX + i);
      uint8_t value_lo = read_uint8_via_spi(MPU9250_SPI_ADDR, REG_AX + i);
      int16_t value = (int16_t)((uint16_t)value_hi << 8 | value_lo);
      write_int16_to_usb(value);
    }
    write_newline_to_usb();
  }
}
