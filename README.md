# mpu9250-spi-maevarm
_reads 9-DOF inertial measurement unit (IMU) data to a MAEVARM M2 microcontroller via SPI_

Function | returns
-------- | -------
**`m_mpu9250_init()`** | _no_
**`m_spi_init()`** | _no_
**`m_write_spi_register(reg, val)`** | _no_
**`m_read_spi_register(reg)`** | unsigned byte
**`m_read_spi_registers(start_reg, count, *dest)`** | _no_

- [x] Initialization of IMU
- [ ] Set custom parameters
  - [ ] accel range
  - [ ] gyro range
  - [ ] sample rate
  - [ ] DLPF
  - [ ] etc
- [ ] Programs to capture values from serial port
  - [ ] Matlab
  - [ ] etc
- [ ] Flash/SD memory storage
