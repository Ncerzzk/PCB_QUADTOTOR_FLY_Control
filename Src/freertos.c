/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "mpu9250.h"
#include "angle.h"
#include "usart.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId PositionTaskHandle;
osThreadId AnalizeUsartTaskHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartPositionTask(void const * a);
void StartAnalizeUsartTask(void const *a);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  
  osThreadDef(positionTask, StartPositionTask, osPriorityRealtime, 0, 256);
  PositionTaskHandle = osThreadCreate(osThread(positionTask),NULL);
  
  osThreadDef(analizeTask, StartAnalizeUsartTask, osPriorityNormal, 0, 128);
  AnalizeUsartTaskHandle=osThreadCreate(osThread(analizeTask),NULL);
  
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void StartPositionTask(void const * a){
  uint32_t previous_wake_time;
  int16_t ac_raw[3],gy_raw[3],offset_ac[3],offset_gy[3];
  float ac[3],gy[3],attitude[3];
  int i;
  for(;;){
    MPU_Read6500(&MPU9250,ac_raw,gy_raw);
    for(i=0;i<3;++i){
      ac[i]=Get_Attitude_Data(ac_raw[i],MPU9250.setting->accel_range,offset_ac[i]);
      gy[i]=Get_Attitude_Data(gy_raw[i],MPU9250.setting->gyro_range,offset_gy[i]);
    }
    IMU_Update(ac,gy,attitude);
    osDelayUntil(&previous_wake_time,2);
  }
}

/*陀螺仪零偏更新任务*/
#define GyroOffsetUpdateBit 0x01    //BIT0
void StartGyroAdjustTask(void const * a){
  int i=0,j=0;
  int32_t sum[3];
  int16_t ac_raw[3],gy_raw[3],offset_gy[3];
  for(;;){
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY); //等待任务标志，平时是阻塞状态
    
    vTaskSuspend(PositionTaskHandle);   //得到任务标志后，挂起姿态解算任务
    sum[0]=0;sum[1]=0,sum[2]=0;
	for(i=0;i<100;++i){
		MPU_Read6500(&MPU9250,ac_raw,gy_raw);
        for(j=0;j<3;++j){
          sum[j]+=gy_raw[j];
        }
		HAL_Delay(1);
	}
	for(j=0;j<3;++j){
      offset_gy[j]=sum[j]/100;
    }
    //发送零偏队列 
    xTaskNotify(PositionTaskHandle,GyroOffsetUpdateBit,eSetBits);//发送更新零偏标志
    
    
    vTaskResume(PositionTaskHandle);//恢复姿态解算任务
    
  }
}

/**/
#define Pram_Size 10
#define FLASH_Start 0x080A0000
void StartWritePramTask(void const * a){
  uint32_t SectorError;
  uint32_t temp;
  float Prams[10];
  int i;
  FLASH_EraseInitTypeDef EraseInitStruct;
  for(;;){	
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);//等待任务通知
    
    //获取Prams
	HAL_FLASH_Unlock();
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = 9;   //FLASH_Start 当前位于第九页
    EraseInitStruct.NbSectors = 1;
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) 
    { 
		uprintf("erase flash fail!\r\n");
		HAL_FLASH_Lock();
		return ;
    }
	
	for(i=0;i<Pram_Size;++i){
		temp=*((uint32_t *)(Prams+i));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_Start+i*4,temp);
		uprintf("write Pram[%d] Ok!\r\n",i);
	}
	HAL_FLASH_Lock();
	uprintf("Write OK!\r\n");    
  }
}

void LoadPramTask(void const *a){
  
}

extern char buffer_rx[20];
extern char buffer_rx_temp;
void StartAnalizeUsartTask(void const *a){
  
  for(;;){
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    analize(buffer_rx);
    HAL_UART_Receive_IT(&huart2,&buffer_rx_temp,1);
  }
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
