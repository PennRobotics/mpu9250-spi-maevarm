#include "m_general.h"
#include "m_spi.h"
#include "m_usb.h"
#include "mpu9250.h"

void setup_timer();

volatile int16_t stream = 0;
volatile uint32_t time = 0;
uint8_t _buffer[21] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0};
int16_t data[20] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0};
uint32_t *time_ptr = (uint32_t*)&data;

void m_mpu9250_dump_one_config_register();  // TODO-debug

// TODO-lo: Transform accel and gyro axes to match magnetometer (see bolderflight/mpu9250.h:189)

int main()
{
  m_clockdivide(0);
  m_spi_init();
  m_spi_speed(SPI_250KHZ);
  m_mpu9250_init();
  m_mpu9250_fast_mode(PIN_D1);
  m_mpu9250_fast_mode(PIN_D2);
  m_spi_speed(SPI_2MHZ);
  m_usb_init();
  setup_timer();

  for(;;)
  {
    uint8_t i, incoming;

    m_read_spi_registers(PIN_D1, ACCEL_OUT, 21, _buffer);
    for (i = 0; i <  3; i++)  { data[i+2] = (((int16_t)_buffer[2*i]) << 8) | _buffer[2*i+1]; }
    // TODO: remove data biases/drift from each
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
      uint8_t idx;
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
        case '1':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_accel(idx, ACCEL_2G); }
          break;
        case '2':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_accel(idx, ACCEL_4G); }
          break;
        case '3':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_accel(idx, ACCEL_8G); }
          break;
        case '4':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_accel(idx, ACCEL_16G); }
          break;
        case 'a':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_gyro(idx, GYRO_250DPS); }
          break;
        case 'b':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_gyro(idx, GYRO_500DPS); }
          break;
        case 'c':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_gyro(idx, GYRO_1000DPS); }
          break;
        case 'd':
          for (idx = 0; idx < NUM_IMU; idx++)  { m_mpu9250_set_gyro(idx, GYRO_2000DPS); }
          break;
        case 'A':
          m_mpu9250_dump_one_config_register();  // TODO-debug
          break;
        case '*':
          m_mpu9250_dump_all_registers();
          break;
        default:
          break;
      }
    }
  }
}
// TODO: gyro offset probably needs to be scaled if gyro range is changed


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

void m_mpu9250_dump_one_config_register()
{
  m2_gpio_t cs_pin = imu_pin_list[0];

  uint8_t val = m_read_spi_register(cs_pin, ACCEL_CONFIG);
  m_usb_tx_char(val);
}


