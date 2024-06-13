/*
 * ssd1306.c
 *
 */

#include "ssd1306.h"
#include "cmsis_os.h"

// i2c地址
#define ADDR	0x78
// 写命令
#define WCOM	0x00
// 写数据
#define WDATA	0x40


////////////////////////////////////////////////////////////////////////////////

I2C_HandleTypeDef* pI2CHandle;

// ssd1306初始化
uint8_t InitData[] = {
		0xae,		/* sleep mode */
		0x20, 0x02,	/* mem addr mode=page, 0x00 hori, 0x01 veri, 0x02 page */
		0xa1,		/* hori direct=👉, 0xa1 👉, 0xa0 👈 */
		0xc8,		/* veri direct=👇, 0xc8 👇, 0xc0 👆 */

		0xb0,		/* page start addr=page0, b0~b7 page0~page7 */
		0x00,		/* col low addr, 00~0f */
		0x10,		/* col high addr, 10~1f */

		0x40,		/* display start line=0, 40~7F 0~63 */
		0x81, 0x00,	/* contrast 00~ff */
		0xa6,		/* 0xa6 normal, 0xa7 invert */
		0xa8, 0x3f,	/* mux [5:0] 16MUX~64MUX */
		0xa4,		/* a4 show use GDDRAM, a5 show no GDDRAM */
		0xd3, 0x00,	/* display effect, COM0~COM63 */
		0xd5, 0xf0,	/* osc div[3:0] 1~16 freq[7:4] */
		0xd9, 0x22,	/* precharge，osc [3:0] 1-15 osc, [7:4] 2-15 osc */
		0xda, 0x12, /* pin map */
		0xdb, 0x20,	/* Vcomh 0x00 ~0.65xVcc, 0x20 ~0.77xVcc, 0x30 ~0.83xVcc */
		0x8d, 0x14,	/* charge pump 0x14 enable */
		0xaf		/* display mode */
};

// 原点地址:page0,col0
uint8_t OriginData[] = {0xb0, 0x00, 0x10};

// 帧缓存
uint8_t SSD1306FramBuffer[8][128];


////////////////////////////////////////////////////////////////////////////////

uint8_t ssd1306_init(I2C_HandleTypeDef* phI2c) {
	pI2CHandle = phI2c;
	osDelay(500);
	HAL_I2C_Mem_Write(pI2CHandle, ADDR, WCOM, I2C_MEMADD_SIZE_8BIT, InitData, sizeof(InitData), 1000);
	HAL_I2C_Mem_Write(pI2CHandle, ADDR, WCOM, I2C_MEMADD_SIZE_8BIT, OriginData, sizeof(OriginData), 1000);
	return 0;
}

uint8_t ssd1306_show(uint8_t* pPos, uint16_t posSize, uint8_t* pData, uint16_t dataSize) {
	HAL_I2C_Mem_Write(pI2CHandle, ADDR, WCOM, I2C_MEMADD_SIZE_8BIT, pPos, posSize, 1000);
	HAL_I2C_Mem_Write(pI2CHandle, ADDR, WDATA, I2C_MEMADD_SIZE_8BIT, pData, dataSize, 1000);
	return 0;
}
