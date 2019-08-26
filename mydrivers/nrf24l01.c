#include "nrf24l01.h"

#define SENDER 1
#define REVEICER 2
#define ROLE   REVEICER

#if ROLE==SENDER
uint8_t  RX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x30}; 
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x40}; 
#else
uint8_t  RX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x40}; 
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x30}; 
#endif

uint8_t  NRF_TX_Data[TX_PLOAD_WIDTH]={1,2,3,4,5,6,7,8,9,10};
uint8_t  NRF_RX_Data[RX_PLOAD_WIDTH]={0};


static void nrf_read_bytes(NRF_Dev *dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size);
static void nrf_write_bytes(NRF_Dev *dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size);
static void nrf_write_reg(NRF_Dev * dev,uint8_t reg,uint8_t tx);
static uint8_t nrf_read_reg(NRF_Dev * dev,uint8_t reg);

NRF_Dev NRF24l01;
static void NRF_Set_GPIO(NRF_GPIO io,NRF_GPIO_Level level);
static void NRF_Read_Write(uint8_t * txData,uint8_t * rxData,uint16_t len);
static void NRF_Analize_Message(uint8_t * data,uint16_t len);
static void NRF_Write(uint8_t * txData,uint16_t len);
static void NRF_Read(uint8_t * rxData,uint16_t len);

extern void Delay_Us(uint32_t nus);

void NRF_Init(NRF_Dev * dev){
  dev->set_gpio=NRF_Set_GPIO;
  dev->delay_ms=HAL_Delay;
  dev->spi_write_read=NRF_Read_Write;
  dev->tx_data=NRF_TX_Data;
  dev->rx_data=NRF_RX_Data; 
  dev->tx_len=TX_PLOAD_WIDTH;
  dev->rx_len=RX_PLOAD_WIDTH;
  dev->rx_analize=NRF_Analize_Message;
  dev->spi_write=NRF_Write;
  dev->spi_read=NRF_Read;
  dev->delay_us=Delay_Us;
  
  nrf_write_reg(dev,NRF_CONFIG,0x02);
  nrf_write_reg(dev,EN_AA,0X00); //禁止自动ACK
  nrf_write_reg(dev,SETUP_RETR,0x00); //禁止重发
  nrf_write_reg(dev,RF_CH,0); //2.4ghz
  nrf_write_reg(dev,RX_PW_P0,dev->rx_len); 
  nrf_write_reg(dev,RF_SETUP,0x07); //2mhz 0db
  nrf_write_reg(dev,EN_RXADDR,0x01);//允许通道1接收
  nrf_write_reg(dev,NRF_CONFIG,0x03);  
  
  nrf_write_bytes(dev,TX_ADDR,TX_ADDRESS,5);
  nrf_write_bytes(dev,RX_ADDR_P0,RX_ADDRESS,5);
  

  
  dev->set_gpio(CE,LOW);
  //dev->set_gpio(CE,HIGH);
}


static void NRF_Set_GPIO(NRF_GPIO io,NRF_GPIO_Level level){
  if(io==CSN){
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,level);
  }else if(io==CE){
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,level);
  } 
}
extern SPI_HandleTypeDef hspi2;

static void NRF_Write(uint8_t * txData,uint16_t len){
  HAL_SPI_Transmit(&hspi2,txData,len,50);
}
static void NRF_Read(uint8_t * rxData,uint16_t len){
  HAL_SPI_Receive(&hspi2,rxData,len,50);
}



static void NRF_Read_Write(uint8_t * txData,uint8_t * rxData,uint16_t len){
  HAL_SPI_TransmitReceive(&hspi2,txData,rxData,len,50);
}

float nrf_command,nrf_thrust,nrf_direction,nrf_ele,nrf_aile;
uint32_t nrf_watch_dog;
extern void uprintf(char *fmt, ...);

float TO_PERCENT(uint16_t data){
  return data/65535.0f*100;
}

extern void RC_Set_Target(float thrust,float direction,float ele,float aile);
static void NRF_Analize_Message(uint8_t * data,uint16_t len){
  //10 bytes
  //0-1 command
  //2-3 thrust
  //4-5 direction
  //6-7 ele
  //8-9 aile
  
  nrf_command=TO_PERCENT(data[0]<<8|data[1]);
  nrf_thrust=TO_PERCENT(data[2]<<8|data[3]);
  nrf_direction=TO_PERCENT(data[4]<<8|data[5]);
  nrf_ele=TO_PERCENT(data[6]<<8|data[7]);
  nrf_aile=TO_PERCENT(data[8]<<8|data[9]);
  
  //uprintf("t:%f d:%f p:%f r:%f\r\n",nrf_thrust,nrf_direction,nrf_ele,nrf_aile);
  RC_Set_Target(nrf_thrust,nrf_direction,nrf_ele,nrf_aile);
  nrf_watch_dog=0;
  //uprintf("%f\r\n",nrf_thrust);
}



void nrf_receive2(NRF_Dev *dev){
	uint8_t t[32];
	int i=0;
    
  if(dev->mode==NRF_RX){
    dev->mode=NRF_WAIT;
    if(nrf_read_reg(dev,STATUS)<0x40)         //未接收到数据
      return ;  
    nrf_write_reg(dev,STATUS,0xFF); 
	dev->set_gpio(CSN,LOW);
	dev->delay_us(10);
	t[0]=R_RX_PAYLOAD;
    dev->spi_write_read(t,t,1);
	for(i=0;i<RX_PLOAD_WIDTH;i++){
		t[i]=NOP;
	}
    dev->spi_write_read(t,dev->rx_data,dev->rx_len);
    
    dev->set_gpio(CE,LOW);
	dev->set_gpio(CSN,HIGH);
    dev->rx_analize(dev->rx_data,dev->rx_len);  
    nrf_write_reg(dev,FLUSH_RX,0);
    nrf_write_reg(dev,NRF_CONFIG,NRF_WAIT);
  }else{   //mode==wait
    dev->mode=NRF_RX;
    nrf_write_reg(dev,FLUSH_RX,0xFF);
    dev->set_gpio(CE,HIGH);
	nrf_write_reg(dev,NRF_CONFIG,NRF_RX);
  }
}

void nrf_receive(NRF_Dev * dev){
	uint8_t t[32];
	int i=0;
    nrf_write_reg(dev,FLUSH_RX,0xFF);
    dev->set_gpio(CE,HIGH);
	nrf_write_reg(dev,NRF_CONFIG,0x03);
	dev->delay_ms(1);
	if(nrf_read_reg(dev,STATUS)<0x40)         //未接收到数据
		return ;
    nrf_write_reg(dev,STATUS,0xFF); 
	dev->set_gpio(CSN,LOW);
	dev->delay_ms(1);
	t[0]=R_RX_PAYLOAD;
    dev->spi_write_read(t,t,1);
	for(i=0;i<RX_PLOAD_WIDTH;i++){
		t[i]=NOP;
	}
    dev->spi_write_read(t,dev->rx_data,dev->rx_len);
    
    dev->set_gpio(CE,LOW);
	dev->set_gpio(CSN,HIGH);
    dev->rx_analize(dev->rx_data,dev->rx_len);  
    nrf_write_reg(dev,FLUSH_RX,0); 
}
void nrf_send_message(NRF_Dev * dev){
	uint8_t r[32],t,rt;
    
    if(dev->tx_len>32)
      return ;
    
	nrf_write_reg(dev,NRF_CONFIG,0x02);  
	t=W_RX_PAYLOAD;
    dev->set_gpio(CSN,LOW);
    dev->spi_write_read(&t,&rt,1);
    
    dev->spi_write_read(dev->tx_data,r,dev->tx_len);
    
	dev->set_gpio(CSN,HIGH);
	dev->set_gpio(CE,HIGH);
	dev->delay_ms(1);
	do{
		t=nrf_read_reg(dev,STATUS);
        if((t&0x10)==0x10){
          nrf_write_reg(dev,STATUS,0x10);
        }
	}while((t&0x20)==0);
	nrf_write_reg(dev,STATUS,0x20);
	dev->set_gpio(CE,LOW);
	nrf_write_reg(dev,NRF_CONFIG,0x03);
}


static void nrf_write_reg(NRF_Dev * dev,uint8_t reg,uint8_t tx){
	uint8_t r,t;
	REDO:
	dev->set_gpio(CSN,LOW);
    //dev->delay_us(1);
	t=W_REGISTER+reg;
    dev->spi_write(&t,1);
	t=tx;
	//dev->delay_us(10);
	dev->spi_write(&t,1);
	dev->set_gpio(CSN,HIGH);
	//dev->delay_ms(1);
	if(reg==STATUS||reg==FLUSH_RX)
      return ;
	if(nrf_read_reg(dev,reg)!=tx){
		goto REDO;
	}
}

static void nrf_write_bytes(NRF_Dev * dev,uint8_t reg,uint8_t * data,uint8_t size){
	uint8_t t,r,rx_buffer[10];
	int i;
	if(size>10)
		return ;
	INIT:
    dev->set_gpio(CSN,LOW);
    //dev->delay_ms(1);
	t=W_REGISTER+reg;
    dev->spi_write_read(&t,&r,1);
    dev->spi_write_read(data,rx_buffer,size);
	dev->set_gpio(CSN,HIGH);
	//dev->delay_ms(1);
	
	nrf_read_bytes(dev,reg,rx_buffer,size);
	for(i=0;i<size;++i){
		if(rx_buffer[i]!=data[i]){
			goto INIT;
		}
	}
	
}

static void nrf_read_bytes(NRF_Dev * dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size){
	uint8_t r;
    uint8_t t[10]={0xFF};
	if(size>10)
		return ;
    dev->set_gpio(CSN,LOW);
	t[0]=reg;
    dev->spi_write_read(t,&r,1);
	t[0]=0xFF;
    dev->spi_write_read(t,rx_buffer,size);
    dev->set_gpio(CSN,HIGH);
}

static uint8_t nrf_read_reg(NRF_Dev * dev,uint8_t reg){
	uint8_t t,r; 
    t=reg;
	dev->set_gpio(CSN,LOW);
    dev->spi_write(&t,1);  
	t=NOP;
    dev->spi_read(&r,1);
	dev->set_gpio(CSN,HIGH);
	return r;
}

