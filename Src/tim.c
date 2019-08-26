/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "mpu9250.h"
#include "angle.h"
#include "control.h"
#include "ms5611.h"
#include "nrf24l01.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
void uprintf(char *fmt, ...);

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}
/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_TIM_Base_Stop(&htim6);

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1999;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_NVIC_SetPriority(TIM7_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  HAL_TIM_Base_Start_IT(&htim7);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void Delay_Us(uint32_t nus){
	
	uint16_t cnt=0;
	HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_SetCounter(&htim6,1);
	
	while(cnt<nus){
		cnt=__HAL_TIM_GetCounter(&htim6);
	}
	HAL_TIM_Base_Stop(&htim6);
}


extern float Get_Attitude_Data(int16_t raw_data,float range,int16_t offset);
extern float Accel[3],Angle_Speed[3],Angle[3],Mag[3];
extern float Accel_E[3]; //天地坐标系下的加速度
extern float Velocity[3];
extern float Height;
extern char Motor_Open_Flag;

extern float ace_sub;

int count_2ms=0;
extern NRF_Dev NRF24l01;
extern uint32_t nrf_watch_dog;
extern void set_stop(int arg_num,char **s,float * args);

extern char NRF_Flag;




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  int16_t ac[3],gy[3],mag[3];
  
  int i;
  if(htim->Instance==TIM7){
    
    count_2ms++;
    
    
    if(count_2ms%2==0){
      if(NRF_Flag){
        nrf_receive2(&NRF24l01);
        nrf_watch_dog++;
        if(nrf_watch_dog>500){
          set_stop(0,0,0);
          nrf_watch_dog=0;
        }
      }else{//蓝牙调试模式
       // base_duty=40;
      }
    }
    
    
    if(count_2ms>5){
      MPU_ReadM_Mag(&MPU9250,mag);
      for(i=0;i<3;++i){
        Mag[i]=mag[i]*0.15f/100.0f;
      }
      Scale_Mag(Mag);
      count_2ms=0;
    }

    MPU_Read6500(&MPU9250,ac,gy);
    for(i=0;i<3;++i){
      Accel[i]=Get_Attitude_Data(ac[i],MPU9250.setting->accel_range,0);
      Angle_Speed[i]=Get_Attitude_Data(gy[i],MPU9250.setting->gyro_range,Gyro_Offset[i]);
    }
    /* 旧飞机
    Accel[0]=Get_Scale_Data(Accel[0],-0.0057510772772645f,0.999396442718185f);
    Accel[1]=Get_Scale_Data(Accel[1],0.00776779781191153f,1.01650084564392f);
    Accel[2]=Get_Scale_Data(Accel[2],-0.00625840169339553f,0.984361130707526f);
*/
    Accel[0]=Get_Scale_Data(Accel[0],-0.0200311354256406f,1.01977103002458f);
    Accel[1]=Get_Scale_Data(Accel[1],-0.000421249210775063f,1.011318642755862f);
    Accel[2]=Get_Scale_Data(Accel[2],0.0411504643712059f,0.969637306791105f);
    
    AHR_Update(Accel,Angle_Speed,Mag,Angle,Accel_E,!Is_Flying());
    //IMU_Update(Accel,Angle_Speed,Angle,Accel_E);
    Free_Falling_Detect(Accel_E,2);
    
    Fly_Control();
    //Get_Velocity(Accel_E,Velocity,2,ace_sub);
    
    Get_Position(Accel_E,Velocity,2,MS5611_Height,&Height);
    
    MS5611_Read(&MS5611);
    
  }
}

void Set_Speed(int CHn,float speed);
void set_speed(int arg_num,char ** string_prams,float * float_prams){
  uint8_t Chn;
  if(arg_num!=0x0002){ 
    uprintf("error arg_num!\r\n");
    return ;
  }
  Chn=(uint8_t)float_prams[0];
  Set_Speed(Chn,float_prams[1]);
  uprintf("OK,set ch%d = %f\r\n",Chn,float_prams[1]);
  
}



void Set_Speed(int CHn,float speed){
  speed=speed>0?speed:0;
  speed=speed>98?98:speed;
  uint32_t ccr=(int)(speed/100.0f*TIM3_PERIOD);
  *(&(TIM3->CCR1)+(CHn-1))=ccr>0?ccr:0;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
