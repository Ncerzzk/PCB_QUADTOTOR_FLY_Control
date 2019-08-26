#include "mpu9250.h"
#include "usart.h"
#include "i2c.h"





MPU_Dev MPU9250;
MPU_Setting MPU9250_Setting;

/*
下面几个个函数需用户自己实现。
*/
extern I2C_HandleTypeDef hi2c3;
uint8_t I2C_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
uint8_t I2C_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len);
void I2C_Reset(void);
extern void Delay_Us(uint32_t nus);



void mpu_error_deal(MPU_Dev *dev);
inline void mpu_error_deal(MPU_Dev *dev);

static void mpu_write_byte(MPU_Dev *dev,uint8_t write_addr,uint8_t data){
	if(dev->i2c_write_buffer(dev->dev_addr,write_addr,&data,1)!=dev->I2C_OK){
		mpu_error_deal(dev);
	}
}

inline void mpu_error_deal(MPU_Dev *dev){
		dev->i2c_error_count++;
		if(dev->i2c_error_count>20){
			dev->i2c_reset();
			dev->i2c_error_count=0;
		}
}

static uint8_t mpu_read_byte(MPU_Dev *dev,uint8_t read_addr){
	uint8_t temp=0;
	if(dev->i2c_read_buffer(dev->dev_addr,read_addr,&temp,1)!=dev->I2C_OK){
		mpu_error_deal(dev);
	}
	return temp;
}

inline void I2C_Error_Check(MPU_Dev *dev,uint8_t i2c_result){
  if(i2c_result!=dev->I2C_OK){
    mpu_error_deal(dev);
  }
}
void MPU_Read6500(MPU_Dev *dev,int16_t ac[],int16_t gy[]){
  
	uint8_t temp[6];
    uint8_t i2c_result;
    
    i2c_result=dev->i2c_read_buffer(dev->dev_addr,ACCEL_XOUT_H,temp,6);
    I2C_Error_Check(dev,i2c_result);
	ac[0]=(temp[0]<<8)|temp[1];
	ac[1]=(temp[2]<<8)|temp[3];
	ac[2]=(temp[4]<<8)|temp[5];
    
    i2c_result=dev->i2c_read_buffer(dev->dev_addr,GYRO_XOUT_H,temp,6);
    I2C_Error_Check(dev,i2c_result);
	gy[0]=(temp[0]<<8)|temp[1];
	gy[1]=(temp[2]<<8)|temp[3];
	gy[2]=(temp[4]<<8)|temp[5];	
}

void MPU_ReadM_Mag(MPU_Dev *dev,int16_t mag[]){
	uint8_t temp[6]; 
    uint8_t ST_data;
    uint8_t i2c_result;
	i2c_result=dev->i2c_read_buffer(dev->dev_mag_addr,MAG_XOUT_L,temp,6);
    
    I2C_Error_Check(dev,i2c_result);
    /*
    连续测量模式，读取完数据之后，必须读一下ST2这个寄存器作为读取完的标志。
    */
    i2c_result=dev->i2c_read_buffer(dev->dev_mag_addr,MAG_ST2,&ST_data,1);
    I2C_Error_Check(dev,i2c_result);
    
    /*
    磁力计的正方向需要修改。MPU9250上，磁力计的X、Y与加速度计的反了，因此，磁力计的X应作为Y
    磁力计的Y应作为X，否则在角度融合时会出现错误。
    磁力计的Z轴正方向与加速度计相反，因此加了个负号。
    
    */
    mag[1]=(int16_t)((temp[1]<<8)|temp[0]);
    mag[0]=(int16_t)((temp[3]<<8)|temp[2]);
    mag[2]=-(int16_t)((temp[5]<<8)|temp[4]);

}


void MPU9250_Init(MPU_Dev * dev){
  
    uint8_t data=0x01;
  
	dev->dev_addr=MPU_Address;
	dev->i2c_read_buffer=I2C_Read_Buffer;
	dev->i2c_write_buffer=I2C_Write_Buffer;
	dev->delay_ms=HAL_Delay;
	
	dev->I2C_OK=0x00;
    //dev->I2C_TIME_OUT=HAL_TIMEOUT;
	dev->i2c_reset=I2C_Reset;
    dev->delay_us=Delay_Us;
	//dev->data=&MPU9250_Data;
	dev->setting=&MPU9250_Setting;
	
	dev->dev_mag_addr=MAG_Address;
	
	dev->setting->accel_range_setting=RANGE2G;
	dev->setting->accel_high_pass_filter_setting=_5HZ;
	dev->setting->gyro_range_setting=RANGE500;
	
	MPU_INIT:
	
	dev->delay_ms(50);
	

	mpu_write_byte(dev,PWR_MGMT_1,0x00);
	mpu_write_byte(dev,SMPLRT_DIV, 0x00);
	mpu_write_byte(dev,MPU_CONFIG, 0x02);  //之前延时为20ms(0x06，现在为3ms左右 0x02)

	mpu_write_byte(dev,GYRO_CONFIG, dev->setting->gyro_range_setting);   //
	mpu_write_byte(dev,ACCEL_CONFIG, dev->setting->accel_range_setting | dev->setting->accel_high_pass_filter_setting); 
	
    dev->setting->mag_range=4912.0/100.0f; 
    
    mpu_write_byte(dev,INT_PIN_CFG,0x02);    //MPU6500 开启路过模式
    
    dev->delay_ms(50);
    
    data=0x01;
    dev->i2c_write_buffer(dev->dev_mag_addr,MAG_CONTROL2,&data,1);   //soft reset AK8963
    
    data=0x16;
    dev->i2c_write_buffer(dev->dev_mag_addr,MAG_CONTROL,&data,1);   //设置，16位输出，连续模式(100hz)
    
    
	switch(dev->setting->gyro_range_setting){
		case RANGE250:
			dev->setting->gyro_range=250.0f/57.3f;
			break;
		case RANGE500:
			dev->setting->gyro_range=500.0f/57.3f;
			break;
		case RANGE1000:
			dev->setting->gyro_range=1000.0f/57.3f;
			break;
		case RANGE2000:
			dev->setting->gyro_range=2000.0f/57.3f;
			break;
	}
	
	switch(dev->setting->accel_range_setting){
		case RANGE2G:
			dev->setting->accel_range=2;
			break;
		case RANGE4G:
			dev->setting->accel_range=4;
			break;	
		case RANGE8G:
			dev->setting->accel_range=8;
			break;	
		case RANGE16G:
			dev->setting->accel_range=16;
			break;		
	}
	
	
	
	dev->dev_ID=mpu_read_byte(dev,WHO_AM_I);
	
	if(dev->dev_ID!=0x71){
		dev->i2c_reset();
        uprintf("mpu9250 init failed!\r\n");
		goto MPU_INIT;
	}

}