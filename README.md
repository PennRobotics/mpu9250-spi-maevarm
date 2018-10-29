# mpu9250-spi-maevarm
_reads 9-DOF inertial measurement unit (IMU) data to a MAEVARM M2 microcontroller via SPI_

Function | returns
-------- | -------
**`m_mpu9250_init()`** | _no_
**`m_mpu9250_set_accel(accel_scale)`** | _no_
**`m_mpu9250_set_gyro(gyro_scale)`** | _no_
**`m_spi_init()`** | _no_
**`m_write_spi_register(reg, val)`** | _no_
**`m_read_spi_register(reg)`** | unsigned byte
**`m_read_spi_registers(start_reg, count, *dest)`** | _no_

_all parameter variables are unsigned bytes or unsigned byte pointers_

- [x] Initialization of IMU
- [ ] Set custom parameters
  - [x] accel range
  - [x] gyro range
  - [ ] sample rate
  - [ ] DLPF
  - [ ] etc
- [ ] Programs to capture values from serial port
  - [ ] Matlab
  - [ ] etc
- [ ] Flash/SD memory storage
