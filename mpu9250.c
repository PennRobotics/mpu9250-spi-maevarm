#include "mpu9250.h"

m2_gpio_t imu_pin_list[NUM_IMU] = {PIN_D1, PIN_D2};

void m_mpu9250_init()  // TODO
{
 uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];
///    USE_PIN_AS_CHIP_SELECT(cs_pin);
    switch (cs_pin)
    {
      case PIN_D1:
        CS_D1();
        DESELECT_D1();
        break;
      case PIN_D2:
        CS_D2();
        DESELECT_D2();
        break;
    }
  }
  _delay_ms(5);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];

    // TODO
    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);
    m_write_spi_register(cs_pin, USER_CTRL, I2C_MST_EN);
    m_write_spi_register(cs_pin, I2C_MST_CTRL, I2C_MST_CLK);
    m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);
    m_write_spi_register(cs_pin, PWR_MGMT_1, PWR_RESET);
    _delay_ms(5);
    m_write_spi_mag_register(cs_pin, AK8963_CNTL2, AK8963_RESET);
    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);

    // Sensor ID verification
    uint8_t whoami = m_read_spi_register(cs_pin, WHO_AM_I);
    if ((whoami != 0x71) && (whoami != 0x73))
    {
      m_red(ON);
      if (!IGNORE_BAD_WHOAMI)  { while(1); }
      _delay_ms(LED_DELAY_MS);
      m_red(OFF);
      _delay_ms(LED_DELAY_MS);
    }
    else
    {
      m_green(ON);
      _delay_ms(LED_DELAY_MS);
      m_green(OFF);
      _delay_ms(LED_DELAY_MS);
    }

    // TODO
    m_write_spi_register(cs_pin, PWR_MGMT_2, SEN_ENABLE);

    // Accel and gyro resolution
    m_write_spi_register(cs_pin, ACCEL_CONFIG, ACCEL_FS_SEL_16G);
    _accel_scale[device_idx] = 16.0f / 32767.5f;
    _accel_range[device_idx] = ACCEL_16G;
    m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);
    _gyro_scale[device_idx] = 2000.0f / 32767.5f;
    _gyro_range[device_idx] = GYRO_2000DPS;

    // DLPF
    m_write_spi_register(cs_pin, ACCEL_CONFIG2, ACCEL_DLPF_184);
    m_write_spi_register(cs_pin, CONFIG, GYRO_DLPF_184);
    /// _bandwidth[device_idx] = DLPF_BANDWIDTH_184;

    // Sample rate divider
    m_write_spi_register(cs_pin, SMPDIV, 0x00);
    /// _srd[device_idx] = 0;

    // TODO
    m_write_spi_register(cs_pin, USER_CTRL, I2C_MST_EN);
    m_write_spi_register(cs_pin, I2C_MST_CTRL, I2C_MST_CLK);
  }

  _m_ak8963_init();  // Compass setup

  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m2_gpio_t cs_pin = imu_pin_list[device_idx];

    // TODO
    m_write_spi_register(cs_pin, PWR_MGMT_1, CLK_PLL);
    m_read_spi_mag_registers(cs_pin, AK8963_HXL, 7, _buffer);  // Get 7 bytes of data from magnetometer at sample rate

    // TODO
    _fchoice_accel[device_idx] = 1;  // By default, users may select sample freq
    _fchoice_gyro[device_idx] = 1;

    // TODO
    _m_mpu9250_calibrate_gyro(cs_pin);
  }
  // Done!
}

void m_mpu9250_set_accel(m2_gpio_t cs_pin, a_range_t accel_range)
{
  // TODO: SPI low-speed
  switch (accel_range)
  {
    case ACCEL_2G:
      m_write_spi_register(cs_pin, ACCEL_CONFIG, ACCEL_FS_SEL_2G);
///      _accel_scale = 2.0f/32767.5f;
      break;
    case ACCEL_4G:
      m_write_spi_register(cs_pin, ACCEL_CONFIG, ACCEL_FS_SEL_4G);
///      _accel_scale = 4.0f/32767.5f;
      break;
    case ACCEL_8G:
      m_write_spi_register(cs_pin, ACCEL_CONFIG, ACCEL_FS_SEL_8G);
///      _accel_scale = 8.0f/32767.5f;
      break;
    case ACCEL_16G:
      m_write_spi_register(cs_pin, ACCEL_CONFIG, ACCEL_FS_SEL_16G);
///      _accel_scale = 16.0f/32767.5f;
      break;
  }
///  _accel_range = accel_range;
  // TODO: restore SPI speed in-use when function was called
}

void m_mpu9250_set_gyro(m2_gpio_t cs_pin, g_range_t gyro_range)
{
  // TODO: SPI low-speed
  switch (gyro_range)
  {
    case GYRO_250DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_250DPS);
///      _gyro_scale = 250.0f / 32767.5f;
      break;
    case GYRO_500DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_500DPS);
///      _gyro_scale = 500.0f / 32767.5f;
      break;
    case GYRO_1000DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_1000DPS);
///      _gyro_scale = 1000.0f / 32767.5f;
      break;
    case GYRO_2000DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);
///      _gyro_scale = 2000.0f / 32767.5f;
      break;
  }
///  _gyro_range = gyro_range;
  // TODO: restore speed
}

void m_mpu9250_fast_mode(uint8_t device_idx)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];
  _fchoice_accel[device_idx] = 0;
  _fchoice_gyro[device_idx] = 0;
#ifndef  ACCEL_FCHOICE_OFF
#define  ACCEL_FCHOICE_OFF  0x08
#define  GYRO_FCHOICE_OFF  0x01
#endif
  // TODO: set lpf_accel_bw_t-type variable to ACC_LPF_1046HZ
  m_write_spi_register(cs_pin, ACCEL_CONFIG2, ACCEL_FCHOICE_OFF);
  // TODO: set lpf_gyro_bw_t-type variable to GY_LPF_8800HZ
  switch (_gyro_range[device_idx])
  {
    case GYRO_250DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_250DPS | GYRO_FCHOICE_OFF);
      break;
    case GYRO_500DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_500DPS | GYRO_FCHOICE_OFF);
      break;
    case GYRO_1000DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_1000DPS | GYRO_FCHOICE_OFF);
      break;
    case GYRO_2000DPS:
      m_write_spi_register(cs_pin, GYRO_CONFIG, GYRO_FS_SEL_2000DPS | GYRO_FCHOICE_OFF);
      break;
  }
}

void m_read_spi_mag_registers(m2_gpio_t cs_pin, uint8_t start_reg, uint8_t count, uint8_t *dest)
{
#ifndef  I2C_READ_FLAG
#define  I2C_READ_FLAG  0x80
#endif
m_write_spi_register(cs_pin, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);  // Set slave 0 to AK8963 for reading
m_write_spi_register(cs_pin, I2C_SLV0_REG, start_reg);  // Set first AK8963 register to read
m_write_spi_register(cs_pin, I2C_SLV0_CTRL, I2C_SLV0_EN | count);  // Enable I2C and request 'count' bytes
_delay_ms(5);  // TODO TODO
m_read_spi_registers(cs_pin, EXT_SENS_DATA_00, count, dest);
}

void m_write_spi_mag_register(m2_gpio_t cs_pin, uint8_t reg, uint8_t val)
{
  m_write_spi_register(cs_pin, I2C_SLV0_ADDR, AK8963_I2C_ADDR);  // Set slave 0 to AK8963 for writing
  m_write_spi_register(cs_pin, I2C_SLV0_REG, reg);  // Set AK8963 register for writing
  m_write_spi_register(cs_pin, I2C_SLV0_DO, val);  // Store the data for writing
  m_write_spi_register(cs_pin, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);  // Enable I2C and send 1 byte
  // TODO: read same register on AK8963 to confirm write successful
  _delay_ms(5);  // TODO TODO
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

void _m_ak8963_init()
{
  // TODO: Use proper chip select for each device_idx, in each subroutine
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_1(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_2(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_3(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_4(device_idx); } _delay_ms(100);
}

void _m_ak8963_init_1(uint8_t device_idx)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];
  uint8_t whoami_compass[1] = {0xFF};
  m_read_spi_mag_registers(cs_pin, AK8963_WHO_AM_I, 1, whoami_compass);
  if (whoami_compass[0] != 0x48)
  {
    m_red(ON);
    if (!IGNORE_BAD_WHOAMI)  { while(1); }
    _delay_ms(LED_DELAY_MS);
    m_red(OFF);
    _delay_ms(LED_DELAY_MS);
  }
  else
  {
    m_green(ON);
    _delay_ms(LED_DELAY_MS);
    m_green(OFF);
    _delay_ms(LED_DELAY_MS);
  }
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);
}

void _m_ak8963_init_2(uint8_t device_idx)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_FUSE_ROM);
}

void _m_ak8963_init_3(uint8_t device_idx)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];
  m_read_spi_mag_registers(cs_pin, AK8963_ASA, 3, _buffer);  // Get 3 bytes: vx, vy, vz
  // _mag_scale_x[device_idx] = ((((float)vx) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;  // TODO: double-check numerical values
  // _mag_scale_y[device_idx] = ((((float)vy) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  // _mag_scale_z[device_idx] = ((((float)vz) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_PWR_DOWN);
}

void _m_ak8963_init_4(uint8_t device_idx)
{
  m2_gpio_t cs_pin = imu_pin_list[device_idx];
  m_write_spi_mag_register(cs_pin, AK8963_CNTL1, AK8963_CNT_MEAS2);  // 16-bit at 100 Hz
}

void _m_mpu9250_calibrate_gyro()  {} // TODO-hi (see bolderflight/mpu9250.cpp:637)
