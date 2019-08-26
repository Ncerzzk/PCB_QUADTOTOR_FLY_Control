#include "stm32f4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "mpu9250.h"
#include "nrf24l01.h"
#include "command.h"
#include "base.h"

/*
这个文件用来运行一些测试
*/

static void Test_Uprintf(){
  uprintf("Test_Uprintf!\r\n");
}

/*
测试MPU9250，包括（6500和磁力计），同时还能顺便测试I2C通信是否正常。
*/
static void Test_MPU9250(){
  uint8_t temp;
  I2C_Read_Buffer(MPU_Address,WHO_AM_I,&temp,1);
  uprintf("Test_MPU9250,mpu6500_id is %x,it must be 0x71 \r\n",temp);
  
  I2C_Read_Buffer(MAG_Address, MAG_WIA,&temp,1);
  uprintf("Test_MPU9250,AKM_id is %x,it must be 0x48 \r\n",temp);
}
