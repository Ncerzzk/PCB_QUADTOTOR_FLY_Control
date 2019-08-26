#ifndef __MPU9250_H
#define __MPU9250_H

#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "function_ptr.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x01(500Hz)
//采样频率=陀螺仪输出频率/(1+SMPLRT_DIV )    陀螺仪输出频率当低通滤波不开的时候为8KHZ，当低通开的时候为1K

#define	MPU_CONFIG			0x1A	//低通滤波频率，典型值：0x01(188Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define INT_PIN_CFG		0x37
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I	    0x75	//IIC地址寄存器(默认数值0x68，只读)

#define GYRO_Range_Configure 0x8   //0x10
#define GYRO_Range 500

#define ACCEL_Range_Configure 0x1  
#define ACCEL_Range 2*9.8f



#define MPU_Address   0xD0
#define MAG_Address 	0x18          //0x0C左移一位

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define MAG_CONTROL 	0x0A
#define MAG_CONTROL2    0x0B

#define MAG_ST1             0x02
#define MAG_ST2             0x09

#define MAG_ASAX			0x10
#define MAG_ASAY			0x11
#define MAG_ASAZ			0x12

#define MAG_WIA             0x00




typedef enum{
	RANGE250=0x00,    //+-250
	RANGE500=0x08,
	RANGE1000=0x10,
	RANGE2000=0x18
}GYRO_RANGE;

typedef enum{
	RANGE2G=0x00,
	RANGE4G=0x08,
	RANGE8G=0x10,
	RANGE16G=0x18
}ACCEL_RANGE;

typedef enum{
	NOHPF=0x00,
	_5HZ=0x01,
	_2_5HZ=0x02,  //2.5HZ
	_1_25HZ=0x03,	//1.25HZ
	_0_63HZ=0x04 	//0.63HZ
}ACCEL_HPF;

typedef struct{
	GYRO_RANGE gyro_range_setting;
	ACCEL_RANGE accel_range_setting;
	uint8_t accel_high_pass_filter_setting;
	float gyro_range;
	int16_t accel_range;
    float mag_range;
}MPU_Setting;

typedef struct{
	i2c_write_buffer_fptr i2c_write_buffer;
	i2c_read_buffer_fptr i2c_read_buffer;
	i2c_err_reset_fptr i2c_reset;
	delay_ms_fptr delay_ms;
    delay_us_fptr delay_us;
    
	uint8_t dev_addr;  //8位地址，即7位地址左移一位后的结果
	uint8_t dev_mag_addr; //磁力计地址
	uint8_t dev_ID;
	uint8_t I2C_OK;
//    uint8_t I2C_TIME_OUT;
	uint16_t i2c_error_count;
	MPU_Setting *setting;
}MPU_Dev;

void MPU9250_Init(MPU_Dev * dev);

extern MPU_Dev MPU9250;

void MPU_Read6500(MPU_Dev *dev,int16_t *,int16_t *);
void MPU_ReadM_Mag(MPU_Dev *dev,int16_t mag[]);
#endif

