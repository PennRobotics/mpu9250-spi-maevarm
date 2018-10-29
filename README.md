# mpu9250-spi-maevarm
_reads 9-DOF inertial measurement unit (IMU) data to a MAEVARM M2 microcontroller via SPI_

Function | returns | in
-------- | ------- | --
**`m_mpu9250_init()`** | _no_ | mpu9250
**`m_mpu9250_set_accel(accel_scale)`** | _no_ | mpu9250
**`m_mpu9250_set_gyro(gyro_scale)`** | _no_ | mpu9250
**`m_read_spi_register(reg)`** | unsigned byte | mpu9250
**`m_read_spi_registers(start_reg, count, *dest)`** | _no_ | mpu9250
**`m_write_spi_register(reg, val)`** | _no_ | mpu9250
**`m_spi_init()`** | _no_ | m_spi

_all parameter variables are unsigned bytes or unsigned byte pointers_

- [ ] MPU-9250
  - [x] Initialization of IMU
  - [ ] Calibrate gyro
  - [ ] Calibrate accel
  - [ ] Calibrate mag
  - [ ] Set custom parameters
    - [x] accel range
    - [x] gyro range
    - [ ] bypass DLPF
    - [ ] set DLPF bandwidth
    - [ ] sample rate
    - [ ] FIFO
    - [ ] wake-on-motion
    - [ ] interrupt on data ready
    - [ ] get/set gyro bias
    - [ ] get/set accel bias
    - [ ] etc? (refer to datsheet)
  - [ ] Self-test
  - [ ] Frame sync?
- [ ] Programs to capture values from serial port
  - [ ] Matlab
  - [ ] etc
- [ ] Flash/SD memory storage
