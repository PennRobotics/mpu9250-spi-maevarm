#include "m_general.h"
#include "m_spi.h"
#include "m_usb.h"
#include "mpu9250.h"

void setup_timer();

volatile int16_t stream = 0;
volatile uint32_t time = 0;
uint8_t _buffer[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int16_t data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t *time_ptr = (uint32_t*)&data;

// TODO-lo: Transform accel and gyro to match magnetometer (see bolderflight/mpu9250.h:189)

int main()
{
  m_clockdivide(0);
  m_spi_init();
  m_spi_speed(SPI_125KHZ);
  m_mpu9250_init();
  m_mpu9250_fast_mode(PIN_D1);  // TODO-lo: Eventually use a for-loop lookup
  m_mpu9250_fast_mode(PIN_D2);
  m_spi_speed(SPI_1MHZ);
  m_usb_init();
  setup_timer();

  for(;;)
  {
    uint8_t i, incoming;

    m_read_spi_registers(PIN_D1, ACCEL_OUT, 21, _buffer);
    for (i = 0; i <  3; i++)  { data[i+2] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    for (i = 4; i <  7; i++)  { data[i+1] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    for (i = 7; i < 10; i++)  { data[i+1] = (((int16_t)_buffer[2*i+1]) << 8) | _buffer[2*i]; }

    m_read_spi_registers(PIN_D2, ACCEL_OUT, 21, _buffer);
    for (i = 0; i <  3; i++)  { data[i+11] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    for (i = 4; i <  7; i++)  { data[i+10] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    for (i = 7; i < 10; i++)  { data[i+10] = (((int16_t)_buffer[2*i+1]) << 8) | _buffer[2*i]; }
    // TODO-lo: Possibility of broadcasting "read" command to each device, then selecting each device serially for response?

    if (m_usb_rx_available())
    {
      incoming = m_usb_rx_char();
      switch (incoming)
      {
        case 'G':
          stream = ON;
          m_green(ON);
          break;
        case 'S':
          stream = OFF;
          m_green(OFF);
          break;
        default:
          break;
      }
    }
  }
}


void setup_timer()
{
  OCR1A = 2000;  // 8 kHz
  set(TCCR1B, WGM13);  // PWM (up to OCR1A)
  set(TCCR1B, WGM12);
  set(TCCR1A, WGM11);
  set(TCCR1A, WGM10);
  set(TIMSK1, OCIE1A);  // Interrupt when OCR1A
  set(TCCR1B, CS10);  // Div 1
  set(DDRB,6);
  OCR1B = 1000;
  set(TCCR1A,COM1B1);  // clear at B, set at A (to verify speed using B6)
  sei();
}


ISR(TIMER1_COMPA_vect)
{
  if (stream)
  {
    usb_serial_write((uint8_t*)data,40);
    (*time_ptr)++;
  }
}

