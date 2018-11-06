# mpu9250-spi-maevarm
_reads 9-DOF inertial measurement unit (IMU) data to a MAEVARM M2 microcontroller via SPI_

### Definitions
`NUM_IMU  =  2`
`IGNORE_BAD_WHOAMI  =  true`
`LED_DELAY_MS  =  250`


### Pinout
D1 - Chip Select IMU 0
D2 - Chip Select IMU 1
B0 - Chip Select for SPI (do not connect)
B1 - SPI Clock
B2 - SPI MOSI
B3 - SPI MISO
_An MPU-9250 operates at 3.3V. The MAEVARM M2 operates at 5V._


### API Reference
Function | returns | in
-------- | ------- | --
**`m_mpu9250_init()`** | _no_ | mpu9250
**`m_mpu9250_set_accel(index, accel_scale)`** | _no_ | mpu9250
**`m_mpu9250_set_gyro(index, gyro_scale)`** | _no_ | mpu9250
**`m_mpu9250_set_accel_lpf(index, lpf_accel_bw)`** | _no_ | mpu9250
**`m_mpu9250_set_gyro_lpf(index, lpf_gyro_bw)`** | _no_ | mpu9250
**`m_mpu9250_fast_mode(index)`** | _no_ | mpu9250
**`void m_mpu9250_dump_all_registers()`** | _no_ | mpu9250
**`m_read_spi_mag_registers(cs_pin, start_reg, count, *dest)`** | _no_ | mpu9250
**`m_write_spi_mag_register(cs_pin, reg, val)`** | _no_ | mpu9250
**`m_read_spi_register(cs_pin, reg)`** | unsigned byte | mpu9250
**`m_read_spi_registers(cs_pin, start_reg, count, *dest)`** | _no_ | mpu9250
**`m_write_spi_register(cs_pin, reg, val)`** | _no_ | mpu9250
**`m_spi_init()`** | _no_ | m\_spi
**`m_spi_speed(freq)`** | _no_ | m\_spi
**`read_spi_byte()`** | unsigned byte | spi
**`write_spi_byte(byte)`** | _no_ | spi
**`exchange_spi_byte(byte)`** | unsigned byte | spi

_all parameter variables are unsigned bytes or unsigned byte pointers_


### Enumerations
`lpf_accel_bw`    |
----------------- |
`ACC_LPF_1046HZ`  |
`ACC_LPF_420HZ`   |
`ACC_LPF_218HZ_B` |
`ACC_LPF_218HZ`   |
`ACC_LPF_99HZ`    |
`ACC_LPF_45HZ`    |
`ACC_LPF_21HZ`    |
`ACC_LPF_10HZ`    |
`ACC_LPF_5HZ`     |

`lpf_gyro_bw`         |
--------------------- |
`GY_LPF_8800HZ`       |
`GY_LPF_3600HZ_HISPD` |
`GY_LPF_3600HZ`       |
`GY_LPF_250HZ`        |
`GY_LPF_184HZ`        |
`GY_LPF_92HZ`         |
`GY_LPF_41HZ`         |
`GY_LPF_20HZ`         |
`GY_LPF_10HZ`         |
`GY_LPF_5HZ`          |

`freq`       |
------------ |
`SPI_125KHZ` |
`SPI_250KHZ` |
`SPI_500KHZ` |
`SPI_1MHZ`   |
`SPI_2MHZ`   |
`SPI_4MHZ`   |
`SPI_8MHZ`   |

`accel_scale` |
------------- |
`ACCEL_2G`    |
`ACCEL_4G`    |
`ACCEL_8G`    |
`ACCEL_16G`   |

`gyro_scale`   |
-------------- |
`GYRO_250DPS`  |
`GYRO_500DPS`  |
`GYRO_1000DPS` |
`GYRO_2000DPS` |

`cs_pin` |
-------- |
`PIN_D1` |
`PIN_D2` |

## Progress
- [ ] MPU-9250
  - [x] Initialization of IMU
  - [ ] Calibrate gyro
  - [ ] Set custom parameters
    - [x] accel range
    - [x] gyro range
    - [x] bypass DLPF
    - [x] set DLPF bandwidth
    - [ ] sample rate
    - [ ] FIFO
    - [ ] wake-on-motion
    - [ ] interrupt on data ready
    - [ ] get/set gyro bias
    - [ ] get/set accel bias
    - [ ] sensitivity scale factors?
  - [ ] Self-test
  - [ ] Accel calibration routine
  - [ ] Mag calibration routine
  - [ ] Frame sync?
  - [ ] Sleep and low-power modes
- [ ] Programs to capture values from serial port
  - [ ] Matlab
  - [ ] etc
- [ ] Flash/SD memory storage
