#include "mpu9250.h"

extern uint8_t _buffer[];

void m_mpu9250_init()  // TODO
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    CS_D1();
    m_spi_init();
    m_write_spi_register(PWR_MGMT_1, CLK_PLL);
    m_write_spi_register(USER_CTRL, I2C_MST_EN);
    m_write_spi_register(I2C_MST_CTRL, I2C_MST_CLK);
    m_write_spi_mag_register(AK8963_CNTL1, AK8963_PWR_DOWN);
    m_write_spi_register(PWR_MGMT_1, PWR_RESET);
    _delay_ms(1);
    m_write_spi_mag_register(AK8963_CNTL2, AK8963_RESET);
    m_write_spi_register(PWR_MGMT_1, CLK_PLL);
    uint8_t whoami = m_read_spi_register(WHO_AM_I);
    if ((whoami != 0x71) && (whoami != 0x73))  { m_red(ON); while(1); }  else  { /*DEBUG*/m_green(ON); m_wait(50); m_green(OFF); m_wait(50); }
    m_write_spi_register(PWR_MGMT_2, SEN_ENABLE);
    m_write_spi_register(ACCEL_CONFIG, ACCEL_FS_SEL_16G);
    // _accel_scale = G * 16.0f/32767.5f;
    // _accel_range = ACCEL_RANGE_16G;
    m_write_spi_register(GYRO_CONFIG, GYRO_FS_SEL_2000DPS);
    // _gyro_scale = 2000.0f/32767.5f;
    // _gyro_range = GYRO_RANGE_2000DPS;
    m_write_spi_register(ACCEL_CONFIG2, ACCEL_DLPF_184);
    m_write_spi_register(CONFIG, GYRO_DLPF_184);
    // _bandwidth = DLPF_BANDWIDTH_184;
    m_write_spi_register(SMPDIV, 0x00);
    // _srd = 0;
    m_write_spi_register(USER_CTRL, I2C_MST_EN);
    m_write_spi_register(I2C_MST_CTRL, I2C_MST_CLK);
  }
  _m_ak8963_init();
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    m_write_spi_register(PWR_MGMT_1, CLK_PLL);
    m_read_spi_mag_registers(AK8963_HXL, 7, _buffer);  // Get 7 bytes of data from magnetometer at sample rate
    _m_mpu9250_calibrate_gyro();
  }
  // Done!
}

void m_write_spi_mag_register(uint8_t reg, uint8_t val)
{
  m_write_spi_register(I2C_SLV0_ADDR, AK8963_I2C_ADDR);  // Set slave 0 to AK8963 for writing
  m_write_spi_register(I2C_SLV0_REG, reg);  // Set AK8963 register for writing
  m_write_spi_register(I2C_SLV0_DO, val);  // Store the data for writing
  m_write_spi_register(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);  // Enable I2C and send 1 byte
  // TODO: read same register on AK8963 to confirm write successful
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

void _m_ak8963_init()
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_1(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_2(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_3(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_4(device_idx); } _delay_ms(100);
}

void _m_ak8963_init_1(uint8_t cs_idx)
{
  uint8_t whoami_compass[1] = {0xFF};
  m_read_spi_mag_registers(AK8963_WHO_AM_I, 1, whoami_compass);
  if (whoami_compass[0] != 0x48)  { m_red(ON); while(1); }  else  { /*DEBUG*/m_green(ON); m_wait(50); m_green(OFF); m_wait(50); }
  m_write_spi_mag_register(AK8963_CNTL1, AK8963_PWR_DOWN);
}

void _m_ak8963_init_2(uint8_t cs_idx)
{
  m_write_spi_mag_register(AK8963_CNTL1, AK8963_FUSE_ROM);
}

void _m_ak8963_init_3(uint8_t cs_idx)
{
  m_read_spi_mag_registers(AK8963_ASA, 3, _buffer);  // Get 3 bytes: vx, vy, vz
  // _mag_scale_x = ((((float)vx) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;  // TODO: double-check numerical values
  // _mag_scale_y = ((((float)vy) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  // _mag_scale_z = ((((float)vz) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  m_write_spi_mag_register(AK8963_CNTL1, AK8963_PWR_DOWN);
}

void _m_ak8963_init_4(uint8_t cs_idx)
{
  m_write_spi_mag_register(AK8963_CNTL1, AK8963_CNT_MEAS2);  // 16-bit at 100 Hz
}

void _m_mpu9250_calibrate_gyro()  {} // TODO-hi (see bolderflight/mpu9250.cpp:637)

void m_read_spi_mag_registers(uint8_t start_reg, uint8_t count, uint8_t *dest)
{
#ifndef  I2C_READ_FLAG
#define  I2C_READ_FLAG  0x80
#endif
  m_write_spi_register(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);  // Set slave 0 to AK8963 for reading
  m_write_spi_register(I2C_SLV0_REG, start_reg);  // Set first AK8963 register to read
  m_write_spi_register(I2C_SLV0_CTRL, I2C_SLV0_EN | count);  // Enable I2C and request 'count' bytes
  _delay_ms(1);
  m_read_spi_registers(EXT_SENS_DATA_00, count, dest);
}
