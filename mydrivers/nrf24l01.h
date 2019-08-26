#ifndef NRF24L01
#define NRF24L01
#include "stm32f4xx_hal.h"
#include "function_ptr.h"

typedef enum{
  CSN,
  CE
}NRF_GPIO;

typedef enum{
  LOW=0,
  HIGH=1
}NRF_GPIO_Level;


typedef void (*nrf_com_fptr)(uint8_t * txData,uint8_t * rxData,uint16_t len);
typedef void (*nrf_read_or_write_fptr)(uint8_t * Data,uint16_t len);
typedef void (*nrf_delay_ms_fptr)(uint32_t t);
typedef void (*nrf_set_gpio_fptr)(NRF_GPIO io,NRF_GPIO_Level level);
typedef void (*nrf_rx_analize_fptr)(uint8_t * data,uint16_t len);
 
typedef enum{
  NRF_TX=2,
  NRF_RX=3,
  NRF_WAIT=0
}NRF_Mode;

typedef struct {
  nrf_com_fptr spi_write_read;
  nrf_read_or_write_fptr spi_write;
  nrf_read_or_write_fptr spi_read;
  delay_ms_fptr delay_ms;
  delay_us_fptr  delay_us;
  nrf_set_gpio_fptr set_gpio;
  nrf_rx_analize_fptr rx_analize;
  uint8_t * tx_data;
  uint16_t tx_len;
  uint8_t * rx_data;
  uint16_t rx_len;
  NRF_Mode mode;
}NRF_Dev;


#define R_REGISTER 0
#define W_REGISTER 0x20
#define R_RX_PAYLOAD 0x61
#define W_RX_PAYLOAD 0xa0
#define FLUSH_TX 0xe1
#define FLUSH_RX 0xe2
#define REUSE_TX_PL 0xe3
#define NOP 0xff

#define TX_ADR_WIDTH    5    // 地址宽度
#define RX_ADR_WIDTH    5    // 
#define TX_PLOAD_WIDTH  10  //  发送数据长度
#define RX_PLOAD_WIDTH  10   // 接受数据长度

#define NRF_CONFIG      0x00  // 配置收发状态，CRC校验模式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器

#define MAX_TX          0x10  //达到�?大发送次数中�?
#define TX_OK           0x20  //TX发�?�完成中�?
#define RX_OK           0x40  //接收到数据中�?

#define OBSERVE_TX      0x08  // 发�?�监测功�?
#define CD              0x09  // 地址�?�?
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
void tx_(uint8_t *data);
void rx_(uint8_t *data);
void NRF_Init(NRF_Dev * dev);
void nrf_receive(NRF_Dev * dev);
void nrf_send_message(NRF_Dev * dev);
void nrf_receive2(NRF_Dev *dev);
#endif
