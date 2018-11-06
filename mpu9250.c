#include "mpu9250.h"
#include "m_usb.h"

m2_gpio_t imu_pin_list[NUM_IMU] = {PIN_D1, PIN_D2};

void m_mpu9250_init()
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];
    _setup_pin_as_chip_select(cs_pin);
  }

  _delay_ms(5);

  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];

    // Accel and gyro initialization
    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);  // Clock source
    m_write_spi_register(cs_pin, USER_CTRL, I2C_MST_EN);  // Isolate I2C for compass
    m_write_spi_register(cs_pin, I2C_MST_CTRL, I2C_MST_CLK);  // 400kHz
    m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);  // Turn off compass
    m_write_spi_register(cs_pin, PWR_MGMT_1, PWR_RESET);  // Reset registers to defaults
    _delay_ms(5);
    m_write_spi_mag_register(cs_pin, AK8963_CNTL2, AK8963_RESET);  // Reset compass to defaults
    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);  // Clock source

    // Sensor ID verification
    uint8_t whoami = m_read_spi_register(cs_pin, WHO_AM_I);
    _blink_yes_or_no( (whoami == 0x71) || (whoami == 0x73) );

    // Activate all accel and gyro channels
    m_write_spi_register(cs_pin, PWR_MGMT_2, SEN_ENABLE);

    // Set accel and gyro resolution
    m_mpu9250_set_accel(device_idx, ACCEL_16G);
    m_mpu9250_set_gyro(device_idx, GYRO_2000DPS);

    // DLPF
    m_mpu9250_set_accel_lpf(device_idx, ACC_LPF_218HZ);
    m_mpu9250_set_gyro_lpf(device_idx, GY_LPF_184HZ);

    // Sample rate divider
    m_write_spi_register(cs_pin, SMPDIV, 0x00);
    _srd[device_idx] = 0;

    // Set IMU as I2C master at 400kHz for compass, isolated from the SPI pins
    m_write_spi_register(cs_pin, USER_CTRL, I2C_MST_EN);
    m_write_spi_register(cs_pin, I2C_MST_CTRL, I2C_MST_CLK);
  }

  _m_ak8963_init();  // Compass setup

  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];

    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);  // Clock source
    m_read_spi_mag_registers(cs_pin, AK8963_HXL, 7, _buffer);  // Get 7 bytes of data from mag

    if (CALIBRATE_GYRO)  { _m_mpu9250_calibrate_gyro(device_idx); }
  }
  // Done!
}


void m_mpu9250_set_accel(uint8_t device_idx, a_range_t accel_range)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  switch (accel_range)
  {
    case ACCEL_2G:   _accel_scale[device_idx] =  2.0f / 32767.5f; break;
    case ACCEL_4G:   _accel_scale[device_idx] =  4.0f / 32767.5f; break;
    case ACCEL_8G:   _accel_scale[device_idx] =  8.0f / 32767.5f; break;
    case ACCEL_16G:  _accel_scale[device_idx] = 16.0f / 32767.5f; break;
  }
  _accel_range[device_idx] = accel_range;

  spi_freq_t current_freq = _spi_freq;  // Slow down SPI as needed
  if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(SPI_1MHZ); }
  m_write_spi_register(cs_pin, ACCEL_CONFIG, accel_range);
  if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(current_freq); }
}


void m_mpu9250_set_gyro(uint8_t device_idx, g_range_t gyro_range)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  switch (gyro_range)
  {
    case GYRO_250DPS:   _gyro_scale[device_idx] =  250.0f / 32767.5f; break;
    case GYRO_500DPS:   _gyro_scale[device_idx] =  500.0f / 32767.5f; break;
    case GYRO_1000DPS:  _gyro_scale[device_idx] = 1000.0f / 32767.5f; break;
    case GYRO_2000DPS:  _gyro_scale[device_idx] = 2000.0f / 32767.5f; break;
  }
  _gyro_range[device_idx] = gyro_range;

  spi_freq_t current_freq = _spi_freq;  // Slow down SPI as needed
  if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(SPI_1MHZ); }
  m_write_spi_register(cs_pin, GYRO_CONFIG, gyro_range);
  if ((uint8_t)current_freq > (uint8_t)SPI_1MHZ)  { m_spi_speed(current_freq); }
}


void m_mpu9250_set_accel_lpf(uint8_t device_idx, lpf_accel_bw_t bandwidth)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  _fchoice_accel[device_idx] = (bandwidth == ACC_LPF_1046HZ) ? 0 : 1;
  _accel_lpf_bandwidth[device_idx] = bandwidth;

  m_write_spi_register(cs_pin, ACCEL_CONFIG2, bandwidth);
}


void m_mpu9250_set_gyro_lpf(uint8_t device_idx, lpf_gyro_bw_t bandwidth)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  _fchoice_gyro[device_idx] = (uint8_t)bandwidth >> 3;
  _gyro_lpf_bandwidth[device_idx] = bandwidth;

  m_write_spi_register(cs_pin, CONFIG, ((uint8_t)bandwidth & 0b111) | /*TODO-lo:FIFO*/0);
  m_write_spi_register(cs_pin, GYRO_CONFIG, (uint8_t)_gyro_range[device_idx] | _fchoice_gyro[device_idx]);
}


void m_mpu9250_fast_mode(uint8_t device_idx)
{
  m_mpu9250_set_accel_lpf(device_idx, ACC_LPF_1046HZ);
  m_mpu9250_set_gyro_lpf(device_idx, GY_LPF_8800HZ);
}


void m_mpu9250_dump_all_registers()
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];
    uint8_t reg, val;

    m_usb_tx_uint(device_idx);
    m_usb_tx_string("-INDEX DEVICE:\n");

    for (reg = 0; reg < 0x7E; reg++) {
      if ((reg == 0x6F) || (reg == 0x74))  { continue; }
      val = m_read_spi_register(cs_pin, reg);
      m_usb_tx_string("REG 0x");
      m_usb_tx_hexchar(reg);
      m_usb_tx_string(" = ");
      m_usb_tx_uint(val);
      m_usb_tx_char('\n');
    }
  }
}


void m_read_spi_mag_registers(m2_gpio_t cs_pin, uint8_t start_reg, uint8_t count, uint8_t *dest)
{
#ifndef  I2C_READ_FLAG
#define  I2C_READ_FLAG  0x80
#endif
  m_write_spi_register(cs_pin, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);  // Mag reads to SLV0
  m_write_spi_register(cs_pin, I2C_SLV0_REG, start_reg);  // Set first AK8963 register to read
  m_write_spi_register(cs_pin, I2C_SLV0_CTRL, I2C_SLV0_EN | count);  // Enable I2C, get 'count' bytes
  _delay_ms(5);
  m_read_spi_registers(cs_pin, EXT_SENS_DATA_00, count, dest);
}


void m_write_spi_mag_register(m2_gpio_t cs_pin, uint8_t reg, uint8_t val)
{
  m_write_spi_register(cs_pin, I2C_SLV0_ADDR, AK8963_I2C_ADDR);  // Set slave 0 to AK8963 for writing
  m_write_spi_register(cs_pin, I2C_SLV0_REG, reg);  // Set AK8963 register for writing
  m_write_spi_register(cs_pin, I2C_SLV0_DO, val);  // Store the data for writing
  m_write_spi_register(cs_pin, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);  // Enable I2C, send 1 byte
  _delay_ms(5);
  // TODO-lo: read same register on AK8963 to confirm write successful
}


#ifndef  CHIP_SELECT
#define  CHIP_SELECT(cs) \
  do { \
    switch (cs) \
    { \
      case PIN_D1: \
        SELECT_D1(); \
        break; \
      case PIN_D2: \
        SELECT_D2(); \
        break; \
    } \
  } while (0)
#endif


#ifndef  CHIP_DESELECT
#define  CHIP_DESELECT(cs) \
  do { \
    switch (cs) \
    { \
      case PIN_D1: \
        DESELECT_D1(); \
        break; \
      case PIN_D2: \
        DESELECT_D2(); \
        break; \
    } \
  } while (0)
#endif


uint8_t m_read_spi_register(m2_gpio_t cs_pin, uint8_t reg)
{
#ifndef  READ_FLAG
#define  READ_FLAG  0x80
#endif
  uint8_t response = 0xFF;
  CHIP_SELECT(cs_pin);
  write_spi_byte(reg | READ_FLAG);
  response = read_spi_byte();
  CHIP_DESELECT(cs_pin);
  return response;
}


void m_read_spi_registers(m2_gpio_t cs_pin, uint8_t start_reg, uint8_t count, uint8_t *dest)
{
#ifndef  READ_FLAG
#define  READ_FLAG  0x80
#endif
  uint8_t i;
  CHIP_SELECT(cs_pin);
  write_spi_byte(start_reg | READ_FLAG);
  for (i = 0; i < count; i++)
  {
    dest[i] = read_spi_byte();
  }
  CHIP_DESELECT(cs_pin);
}


void m_write_spi_register(m2_gpio_t cs_pin, uint8_t reg, uint8_t val)
{
#ifndef  WRITE_FLAG
#define  WRITE_FLAG  0x00
#endif
  CHIP_SELECT(cs_pin);
  write_spi_byte(reg | WRITE_FLAG);
  write_spi_byte(val);
  CHIP_DESELECT(cs_pin);
}


// INTERNAL FUNCTIONS
void _setup_pin_as_chip_select(m2_gpio_t cs_pin)
{
  switch (cs_pin)
  {
    case PIN_D1:  CS_D1(); DESELECT_D1(); break;
    case PIN_D2:  CS_D2(); DESELECT_D2(); break;
  }
}


void _m_ak8963_init()
{
  uint8_t idx;
  for (idx = 0; idx < NUM_IMU; idx++)  { _m_ak8963_init_1(idx); }  _delay_ms(100);
  for (idx = 0; idx < NUM_IMU; idx++)  { _m_ak8963_init_2(idx); }  _delay_ms(100);
  for (idx = 0; idx < NUM_IMU; idx++)  { _m_ak8963_init_3(idx); }  _delay_ms(100);
  for (idx = 0; idx < NUM_IMU; idx++)  { _m_ak8963_init_4(idx); }  _delay_ms(100);
}


void _m_ak8963_init_1(uint8_t device_idx)  // WHOAMI
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  uint8_t whoami_compass[1] = {0xFF};
  m_read_spi_mag_registers(cs_pin, AK8963_WHO_AM_I, 1, whoami_compass);
  _blink_yes_or_no(whoami_compass[0] == 0x48);

  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);
}


void _m_ak8963_init_2(uint8_t device_idx)  // FUSE ROM MODE
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_FUSE_ROM);
}


// 4912 = magnetic flux density, in uT; 32760 = 16-bit max of AK8963
void _m_ak8963_init_3(uint8_t device_idx)  // GET MAG SENSITIVITY ADJUSTMENTS
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  m_read_spi_mag_registers(cs_pin, AK8963_ASA, 3, _buffer);  // Get 3 bytes: vx, vy, vz
  _mag_scale_x[device_idx] = ((((float)_buffer[0]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  _mag_scale_y[device_idx] = ((((float)_buffer[1]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  _mag_scale_z[device_idx] = ((((float)_buffer[2]) - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;

  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);
}


void _m_ak8963_init_4(uint8_t device_idx)  // CONTINUOUS MODE
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_CNT_MEAS2);  // 16-bit at 100 Hz
}


void _m_mpu9250_calibrate_gyro(uint8_t device_idx)
{
  m_red(ON); m_green(ON);  // Don't move the IMU when both LEDs are lit

  m2_gpio_t cs_pin = imu_pin_list[device_idx];

  // Temporarily set range, bandwidth, sample rate divider
  g_range_t temp_gyro_range = _gyro_range[device_idx];
  lpf_gyro_bw_t temp_gyro_lpf_bandwidth = _gyro_lpf_bandwidth[device_idx];
  uint8_t temp_srd = _srd[device_idx];

  m_mpu9250_set_gyro(device_idx, GYRO_250DPS);
  m_mpu9250_set_gyro_lpf(device_idx, GY_LPF_20HZ);

  // Sample rate setting is a bit complicated...
  // TODO-lo: Move this into its own function
  m_write_spi_register(cs_pin, SMPDIV, 19);
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);  // Turn off compass
  _delay_ms(100);
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_CNT_MEAS1);  // 16-bit at 8 Hz
  _delay_ms(100);
  m_read_spi_mag_registers(cs_pin, AK8963_HXL, 7, _buffer);  // Get 7 bytes of data from mag
  m_write_spi_register(cs_pin, SMPDIV, 19);

  // Take samples, find bias
  _gx_bias[device_idx] = 0;
  _gy_bias[device_idx] = 0;
  _gz_bias[device_idx] = 0;

  for (size_t i=0; i < NUM_CALIBRATION_SAMPLES; i++)
  {
    // Read IMU data
    m_read_spi_registers(PIN_D1, ACCEL_OUT, 21, _buffer);

    int16_t _gx = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    int16_t _gy = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    int16_t _gz = (((int16_t)_buffer[12]) << 8) | _buffer[13];

    // Accumulate bias
    _gx_bias[device_idx] += _gx;
    _gy_bias[device_idx] += _gy;
    _gz_bias[device_idx] += _gz;

    _delay_ms(20);
  }

  _gx_bias[device_idx] /= NUM_CALIBRATION_SAMPLES;
  _gy_bias[device_idx] /= NUM_CALIBRATION_SAMPLES;
  _gz_bias[device_idx] /= NUM_CALIBRATION_SAMPLES;

  // Restore range, bandwidth, sample rate divider
  m_mpu9250_set_gyro(device_idx, temp_gyro_range);
  m_mpu9250_set_gyro_lpf(device_idx, temp_gyro_lpf_bandwidth);

  // Sample rate setting is a bit complicated...
  // TODO-lo: Call the set_srd function after it is created
  m_write_spi_register(cs_pin, SMPDIV, 19);
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);  // Turn off compass
  _delay_ms(100);
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, (temp_srd > 9) ?
                                                  AK8963_CNT_MEAS1 :  // 16-bit at 8 Hz
                                                  AK8963_CNT_MEAS2);  // 16-bit at 100 Hz
  _delay_ms(100);
  m_read_spi_mag_registers(cs_pin, AK8963_HXL, 7, _buffer);  // Get 7 bytes of data from mag
  m_write_spi_register(cs_pin, SMPDIV, temp_srd);

  m_red(OFF); m_green(OFF);
}


void _blink_yes_or_no(bool equality)
{
  if (equality)
  {
    m_green(ON);
    _delay_ms(LED_DELAY_MS);
    m_green(OFF);
    _delay_ms(LED_DELAY_MS);
  }
  else
  {
    m_red(ON);
    _delay_ms(LED_DELAY_MS);
    if (!IGNORE_BAD_WHOAMI)  { while(1); }
    m_red(OFF);
    _delay_ms(LED_DELAY_MS);
  }
}
