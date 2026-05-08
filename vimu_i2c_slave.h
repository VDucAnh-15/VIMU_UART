#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "stm32f10x.h"

/*
* Khởi tạo I2C1 với PB6(SCL) PB7(SDA)
* Cấu hình địa chỉ slave 7 bit là 0x68 cho STM32 mô phỏng DS1307
*/
void I2C_Slave_Init(uint8_t address);

/********** Các hàm callback do người dùng định nghĩa **********/

#endif
