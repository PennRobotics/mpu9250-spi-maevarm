#include "m_general.h"
#include "mpu9250.h"
#include "m_ss.h"
#include "m_spi.h"

#define  NUM_IMU  1

void m_mpu9250_init();
void _m_ak8963_init();
void _m_ak8963_init_1();
void _m_ak8963_init_2();
void _m_ak8963_init_3();
void _m_ak8963_init_4();
void _m_mpu9250_calibrate_gyro();

// TODO-lo: Transform accel and gyro to match magnetometer (see bolderflight/mpu9250.h:189)

int main()
{
  m_spi_init();
  m_mpu9250_init();
  m_usb_init();
  for(;;)
  {
    _delay_ms(100);
    uint8_t i;
    for (i = 0; i < 10; i++) {
      uint8_t value_hi = m_read_spi_register(MPU9250_REG_AXH + 2*i);
      uint8_t value_lo = m_read_spi_register(MPU9250_REG_AXH + 2*i + 1);
      int16_t value = (int16_t)value_hi << 8 | value_lo;
      write_int16_to_usb(value);
    }
    m_read_spi_register(MPU9250_REG_AXH + 20);  // Grab new compass data
    write_newline_to_usb();
  }
}

void m_mpu9250_init()  // TODO
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    // cs output
    // cs high
    // spi begin ( _spi->begin() )
    // wr PWR_MGMT_1, CLK_PLL
    // wr USER_CTRL, I2C_MST_EN
    // wr I2C_MST_CTRL, I2C_MST_CLK
    // wrAK AK8963_CNTL1, AK8963_PWR_DOWN
    // wr PWR_MGMT_1, PWR_RESET
    _delay_ms(1);
    // wrAK AK8963_CNTL2, AK8963_RESET
    // wr PWR_MGMT_1, CLK_PLL
    // whoami 0x71 or 0x73
    // wr PWR_MGMT_2, SEN_ENABLE
    // wr ACCEL_CONFIG, ACCEL_FS_SEL_16G
    // _accel_scale = G * 16.0f/32767.5f;
    // _accel_range = ACCEL_RANGE_16G;
    // wr GYRO_CONFIG, GYRO_FS_SEL_2000DPS
    // _gyro_scale = 2000.0f/32767.5f;
    // _gyro_range = GYRO_RANGE_2000DPS;
    // wr ACCEL_CONFIG2, ACCEL_DLPF_184
    // wr CONFIG, GYRO_DLPF_184
    // _bandwidth = DLPF_BANDWIDTH_184;
    // wr SMPDIV, 0x00
    // _srd = 0;
    // wr USER_CTRL, I2C_MST_EN
    // wr I2C_MST_CTRL, I2C_MST_CLK
  }
  _m_ak8963_init();
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)
  {
    // wr PWR_MGMT_1, CLK_PLL
    // rdAK x7 AK8963_HXL -> buffer
    _m_mpu9250_calibrate_gyro();
  }
}

void _m_ak8963_init()
{
  uint8_t device_idx;
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_1(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_2(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_3(device_idx); } _delay_ms(100);
  for (device_idx = 0; device_idx < NUM_IMU; device_idx++)  { _m_ak8963_init_4(device_idx); } _delay_ms(100);
}

void _m_ak8963_init_1()  // TODO
{
  // whoamiAK 0x48
  // wrAK AK8963_CNTL1, AK8963_PWR_DOWN
}

void _m_ak8963_init_2()  // TODO
{
  // wrAK AK8963_CNTL1, AK8963_FUSE_ROM
}

void _m_ak8963_init_3()  // TODO
{
  // rdAK x3 AK8963_ASA -> vx vy vz
  // _mag_scale_x = ((((float)vx) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;  // TODO: double-check numerical values
  // _mag_scale_y = ((((float)vy) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  // _mag_scale_z = ((((float)vz) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f;
  // wrAK AK8963_CNTL1, AK8963_PWR_DOWN
}

void _m_ak8963_init_4()  // TODO
{
  // wrAK AK8963_CNTL1, AK8963_CNT_MEAS2  // 16-bit at 100 Hz
}

void _m_mpu9250_calibrate_gyro()  {} // TODO-hi (see bolderflight/mpu9250.cpp:637)
