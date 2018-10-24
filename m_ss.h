#ifndef M_SS_H
#define M_SS_H

#define  MPU9250_REG_AXH  0x3B

#include "m_general.h"
#include "m_usb.h"

void init_mpu();
void init_ak();
void write_int16_to_usb(int16_t);
void write_newline_to_usb();

#endif
