/*
 * ssd1306.h
 *	修改i2c handle定义
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include "stm32f1xx_hal.h"


////////////////////////////////////////////////////////////////////////////////

uint8_t ssd1306_init(I2C_HandleTypeDef* phI2c);
uint8_t ssd1306_show(uint8_t* pPos, uint16_t posSize, uint8_t* pData, uint16_t dataSize);


#endif /* INC_SSD1306_H_ */
