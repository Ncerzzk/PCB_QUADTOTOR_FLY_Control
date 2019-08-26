#include "ms5611.h"
#include "base.h"

static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM

MS5611_Dev MS5611;
float MS5611_Height;

extern I2C_HandleTypeDef hi2c3;
uint8_t I2C_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
uint8_t I2C_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len);
void I2C_Reset(void);
void HAL_Delay(__IO uint32_t Delay);

static uint16_t ms5611_prom(MS5611_Dev * dev,int8_t coef_num);

void MS5611_Init(MS5611_Dev * dev){
  dev->i2c_read_buffer=I2C_Read_Buffer;
  dev->i2c_write_buffer=I2C_Write_Buffer;
  dev->i2c_reset=I2C_Reset;
  dev->delay_ms=HAL_Delay;
  dev->call_cycle=2;         //Read函数调用周期，单位ms
  
  dev->dev_addr=MS5611_ADDR;
  dev->I2C_OK=0x00;
  
  dev->osr=CMD_ADC_4096;
  
  switch(dev->osr){
  case CMD_ADC_4096:
    dev->adc_time=10;
    break;
  case CMD_ADC_2048:
    dev->adc_time=5;
    break;
  case CMD_ADC_1024:
    dev->adc_time=3;
    break;
  case CMD_ADC_512:
    dev->adc_time=2;
    break;
  case CMD_ADC_256:
    dev->adc_time=1;
    break;
  }
  
  dev->call_time=0;
  dev->step=0;
  for (int i = 0; i < PROM_NB; ++i){
    ms5611_c[i] = ms5611_prom(dev,i);
  }
}

static uint16_t ms5611_prom(MS5611_Dev * dev,int8_t coef_num)
{
  uint8_t rxbuf[2] = { 0, 0 };
  dev->i2c_read_buffer(dev->dev_addr,CMD_PROM_RD+coef_num * 2,rxbuf,2);
  return rxbuf[0] << 8 | rxbuf[1];
}

static void ms5611_error_deal(MS5611_Dev *dev){
  dev->i2c_error_count++;
  if(dev->i2c_error_count>20){
    dev->i2c_reset();
    dev->i2c_error_count=0;
  }  
}

inline void I2C_Error_Check(MS5611_Dev *dev,uint8_t i2c_result){
  if(i2c_result!=dev->I2C_OK){
    ms5611_error_deal(dev);
  }
}

static void ms5611_write_byte(MS5611_Dev *dev,uint8_t write_addr,uint8_t data){
  uint8_t i2c_result;
  i2c_result=dev->i2c_write_buffer(dev->dev_addr,write_addr,&data,1);
  I2C_Error_Check(dev,i2c_result);
}


 /*
 * 开始采集温度.
 */
static void ms5611_start_ut(MS5611_Dev *dev){
  ms5611_write_byte(dev,CMD_ADC_CONV + CMD_ADC_D2 + dev->osr,1);
}
 /*
 * 开始采集气压.
 */
static void ms5611_start_up(MS5611_Dev *dev)
{
  ms5611_write_byte(dev,CMD_ADC_CONV + CMD_ADC_D1 + dev->osr,1);
}

/*
* 读取 ms5611 采集的 温度/气压(24bit)
*/
static uint32_t ms5611_read_adc(MS5611_Dev *dev)
{
    uint8_t rxbuf[3];
    dev->i2c_read_buffer(dev->dev_addr,CMD_ADC_READ,rxbuf,3);
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

static void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int32_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);
   
    //temp = 3000;
    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
    
    //uprintf("p:%d   t:%d\r\n",press,temp);
}


void uprintf(char *fmt, ...);
Kal_Struct kal_height={1,0,0.01,0.15,0,1};

extern void Get_Height(float *vz,float refer_height,int call_time,float *height);
extern float Velocity[3];
extern float Height;
extern History_Buffer Height_HB;
extern void Height_Control();

float MS5611_Height_Window[2];

Window_Filter_Struct MS5611_Height_WFS={MS5611_Height_Window,2,0};

Butter_Parameter MS_Butter_Pram={\
  {1,1.1429805025399007  ,  0.41280159809618855},\
    {0.63894552515902237,1.2778910503180447 ,0.63894552515902237}};

Butter_BufferData MS_Butter_Data;
  
void MS5611_Read(MS5611_Dev *dev){
  int32_t pressure,temperature;
  
#if MS5611OPEN==1
  if(dev->call_cycle>=dev->adc_time){
    dev->step++;
  }else{
    dev->call_time+=dev->call_cycle;
    if(dev->call_time>dev->adc_time){
      dev->call_time=0;
      dev->step++;
    }else{
      return ;
    }
  }
  
  switch(dev->step){
  case 1: 
    ms5611_ut=ms5611_read_adc(dev);
    ms5611_calculate(&pressure,&temperature);
    MS5611_Height=(101325-pressure)*10.0f;
    //MS5611_Height=Limit_Dealt_Filter((101325-pressure)*10.0f,&MS5611_Height_WFS,20);
    //MS5611_Height=LPButterworth(MS5611_Height,&MS_Butter_Data,&MS_Butter_Pram);
    Height_Control();
    ms5611_start_up(dev);
    break;
  case 2:
    ms5611_up=ms5611_read_adc(dev);
    ms5611_start_ut(dev);    
    dev->step=0;
    break;
  }
#endif
}

void MS5611_test(){
  MS5611_Dev*dev=&MS5611;
  int32_t t,p;
  ms5611_start_ut(dev); 
  HAL_Delay(8);
  ms5611_ut=ms5611_read_adc(dev);
  ms5611_start_up(dev);
  HAL_Delay(8);
  ms5611_up=ms5611_read_adc(dev);
  ms5611_calculate(&p,&t);
}
