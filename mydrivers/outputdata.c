 

#include "outputdata.h"
#include "string.h"


int OutData[4] = {0};

extern UART_HandleTypeDef huart2;

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}


void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
	
  HAL_UART_Transmit(&huart6,databuf,10,100);

}
  
void Out_My_Data(float d1,float d2,float d3,float d4){
  char temp[19]={0xFF,0x00,16};
  
  memcpy(temp+3,&d1,sizeof(d1));
  memcpy(temp+7,&d2,sizeof(d1));
  memcpy(temp+11,&d3,sizeof(d1));
  memcpy(temp+15,&d4,sizeof(d1));
  
  HAL_UART_Transmit(&huart6,temp,19,100);
}