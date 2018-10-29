#include "m_general.h"
#include "m_ss.h"
#include "m_spi.h"
#include "mpu9250.h"

uint8_t _buffer[21];
int16_t data[10];

// TODO-lo: Transform accel and gyro to match magnetometer (see bolderflight/mpu9250.h:189)

int main()
{
  m_clockdivide(0);
  m_mpu9250_init();
  m_usb_init();

  for(;;)
  {
    uint8_t i;

    m_read_spi_registers(ACCEL_OUT, 21, _buffer);
    for (i = 0; i <  7; i++)  { data[i] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    for (i = 7; i < 10; i++)  { data[i] = (((int16_t)_buffer[2*i+1]) << 8) | _buffer[2*i]; }

    for (i = 0; i < 10; i++)  { write_int16_to_usb(data[i]); }
    write_newline_to_usb();

    _delay_ms(100);
  }
}
