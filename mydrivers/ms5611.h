#ifndef __MS5611_H
#define __MS5611_H

#include "stm32f4xx_hal.h"
#include "function_ptr.h"

#define MS5611OPEN      1

typedef struct{
  	i2c_write_buffer_fptr i2c_write_buffer;
	i2c_read_buffer_fptr i2c_read_buffer;
	i2c_err_reset_fptr i2c_reset;
	delay_ms_fptr delay_ms;
    uint8_t dev_addr;
	uint8_t I2C_OK;
    uint8_t I2C_TIME_OUT;
	uint16_t i2c_error_count;
    uint8_t osr;            //常量
    uint8_t call_cycle;     //常量
    uint8_t adc_time;       //常量
    
    uint16_t call_time;     //调用时间
    uint8_t step;           //调用状态
}MS5611_Dev;

// MS5611, Standard address 0x77
#define MS5611_ADDR                 0xEE
// Autodetect: turn off BMP085 while initializing ms5611 and check PROM crc to confirm device
#define BMP085_OFF                  digitalLo(BARO_GPIO, BARO_PIN);
#define BMP085_ON                   digitalHi(BARO_GPIO, BARO_PIN);

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

extern MS5611_Dev MS5611;
extern float MS5611_Height;
void MS5611_Init(MS5611_Dev * dev);
void MS5611_Read(MS5611_Dev *dev);
#endif
