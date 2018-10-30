#ifndef mpu9250__
#define mpu9250__

#include "m_general.h"
#include "m_spi.h"
#include <stdlib.h>

#define  NUM_IMU  2

#define  CS_D1()  set(DDRD, 1)
#define  SELECT_D1()  clear(PORTD, 1)
#define  DESELECT_D1()  set(PORTD, 1)

#define  CS_D2()  set(DDRD, 2)
#define  SELECT_D2()  clear(PORTD, 2)
#define  DESELECT_D2()  set(PORTD, 2)

typedef enum
{
  PIN_D1,
  PIN_D2
} m2_gpio_t;

typedef enum
{
  ACCEL_2G,
  ACCEL_4G,
  ACCEL_8G,
  ACCEL_16G
} a_range_t;

typedef enum
{
  GYRO_250DPS,
  GYRO_500DPS,
  GYRO_1000DPS,
  GYRO_2000DPS
} g_range_t;

typedef enum
{
  FREQ_32KHZ,
  FREQ_8KHZ,
  FREQ_4KHZ,
  FREQ_1KHZ
} freq_t;  // TODO

typedef enum
{
  ACC_LPF_1046HZ,
  ACC_LPF_420HZ,
  ACC_LPF_218HZ,
  ACC_LPF_99HZ,
  ACC_LPF_45HZ,
  ACC_LPF_21HZ,
  ACC_LPF_10HZ,
  ACC_LPF_5HZ
} lpf_accel_bw_t;  // TODO

typedef enum
{
  GY_LPF_8800HZ,
  GY_LPF_3600HZ,
  GY_LPF_250HZ,
  GY_LPF_184HZ,
  GY_LPF_92HZ,
  GY_LPF_41HZ,
  GY_LPF_20HZ,
  GY_LPF_10HZ,
  GY_LPF_5HZ
} lpf_gyro_bw_t;  // TODO

extern uint8_t _buffer[];

m2_gpio_t imu_pin_list[NUM_IMU];

float _accel_scale[NUM_IMU];
float _gyro_scale[NUM_IMU];
a_range_t _accel_range[NUM_IMU];
g_range_t _gyro_range[NUM_IMU];
uint8_t _fchoice_accel[NUM_IMU];
uint8_t _fchoice_gyro[NUM_IMU];

void m_mpu9250_init();
void m_mpu9250_set_accel(m2_gpio_t, a_range_t);
void m_mpu9250_set_gyro(m2_gpio_t, g_range_t);
void m_mpu9250_fast_mode(uint8_t);
void m_read_spi_mag_registers(m2_gpio_t, uint8_t, uint8_t, uint8_t*);
void m_write_spi_mag_register(m2_gpio_t, uint8_t, uint8_t);
uint8_t m_read_spi_register(m2_gpio_t, uint8_t);
void m_read_spi_registers(m2_gpio_t, uint8_t, uint8_t, uint8_t*);
void m_write_spi_register(m2_gpio_t, uint8_t, uint8_t);

void _m_ak8963_init();
void _m_ak8963_init_1(uint8_t);
void _m_ak8963_init_2(uint8_t);
void _m_ak8963_init_3(uint8_t);
void _m_ak8963_init_4(uint8_t);
void _m_mpu9250_calibrate_gyro();

#define  ACCEL_OUT  0x3B
#define  GYRO_OUT  0x43
#define  TEMP_OUT  0x41
#define  EXT_SENS_DATA_00  0x49
#define  ACCEL_CONFIG  0x1C
#define  ACCEL_FS_SEL_2G  0x00
#define  ACCEL_FS_SEL_4G  0x08
#define  ACCEL_FS_SEL_8G  0x10
#define  ACCEL_FS_SEL_16G  0x18
#define  GYRO_CONFIG  0x1B
#define  GYRO_FS_SEL_250DPS  0x00
#define  GYRO_FS_SEL_500DPS  0x08
#define  GYRO_FS_SEL_1000DPS  0x10
#define  GYRO_FS_SEL_2000DPS  0x18
#define  ACCEL_CONFIG2  0x1D
#define  ACCEL_DLPF_184  0x01
#define  ACCEL_DLPF_92  0x02
#define  ACCEL_DLPF_41  0x03
#define  ACCEL_DLPF_20  0x04
#define  ACCEL_DLPF_10  0x05
#define  ACCEL_DLPF_5  0x06
#define  CONFIG  0x1A
#define  GYRO_DLPF_184  0x01
#define  GYRO_DLPF_92  0x02
#define  GYRO_DLPF_41  0x03
#define  GYRO_DLPF_20  0x04
#define  GYRO_DLPF_10  0x05
#define  GYRO_DLPF_5  0x06
#define  SMPDIV  0x19
#define  INT_PIN_CFG  0x37
#define  INT_ENABLE  0x38
#define  INT_DISABLE  0x00
#define  INT_PULSE_50US  0x00
#define  INT_WOM_EN  0x40
#define  INT_RAW_RDY_EN  0x01
#define  PWR_MGMT_1  0x6B
#define  PWR_CYCLE  0x20
#define  PWR_RESET  0x80
#define  CLK_PLL  0x01
#define  PWR_MGMT_2  0x6C
#define  SEN_ENABLE  0x00
#define  DIS_GYRO  0x07
#define  USER_CTRL  0x6A
#define  I2C_MST_EN  0x20
#define  I2C_MST_CLK  0x0D
#define  I2C_MST_CTRL  0x24
#define  I2C_SLV0_ADDR  0x25
#define  I2C_SLV0_REG  0x26
#define  I2C_SLV0_DO  0x63
#define  I2C_SLV0_CTRL  0x27
#define  I2C_SLV0_EN  0x80
#define  I2C_READ_FLAG  0x80
#define  MOT_DETECT_CTRL  0x69
#define  ACCEL_INTEL_EN  0x80
#define  ACCEL_INTEL_MODE  0x40
#define  LP_ACCEL_ODR  0x1E
#define  WOM_THR  0x1F
#define  WHO_AM_I  0x75
#define  FIFO_EN  0x23
#define  FIFO_TEMP  0x80
#define  FIFO_GYRO  0x70
#define  FIFO_ACCEL  0x08
#define  FIFO_MAG  0x01
#define  FIFO_COUNT  0x72
#define  FIFO_READ  0x74
#define  AK8963_I2C_ADDR  0x0C
#define  AK8963_HXL  0x03
#define  AK8963_CNTL1  0x0A
#define  AK8963_PWR_DOWN  0x00
#define  AK8963_CNT_MEAS1  0x12
#define  AK8963_CNT_MEAS2  0x16
#define  AK8963_FUSE_ROM  0x0F
#define  AK8963_CNTL2  0x0B
#define  AK8963_RESET  0x01
#define  AK8963_ASA  0x10
#define  AK8963_WHO_AM_I  0x00

#endif
