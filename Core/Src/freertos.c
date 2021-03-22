/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for x1Task */
osThreadId_t x1TaskHandle;
const osThreadAttr_t x1Task_attributes = {
  .name = "x1Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for x2Task */
osThreadId_t x2TaskHandle;
const osThreadAttr_t x2Task_attributes = {
  .name = "x2Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for x3Task */
osThreadId_t x3TaskHandle;
const osThreadAttr_t x3Task_attributes = {
  .name = "x3Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for xa1 */
osSemaphoreId_t xa1Handle;
const osSemaphoreAttr_t xa1_attributes = {
  .name = "xa1"
};
/* Definitions for xa2 */
osSemaphoreId_t xa2Handle;
const osSemaphoreAttr_t xa2_attributes = {
  .name = "xa2"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xa1 */
  xa1Handle = osSemaphoreNew(1, 1, &xa1_attributes);

  /* creation of xa2 */
  xa2Handle = osSemaphoreNew(1, 1, &xa2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of x1Task */
  x1TaskHandle = osThreadNew(StartTask01, NULL, &x1Task_attributes);

  /* creation of x2Task */
  x2TaskHandle = osThreadNew(StartTask02, NULL, &x2Task_attributes);

  /* creation of x3Task */
  x3TaskHandle = osThreadNew(StartTask03, NULL, &x3Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the x1Task thread.
  * @param  argument: Not used
  * @retval None
  */
	int s=0;
unsigned char MasterSend[64];
unsigned char MasterPkgIndex=0;
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
	A7131_WriteReg( PLL1_REG, 155 ); 
		int timeoutSend=0;
		int timeoutReceive=0;	
	int wantWrite=1;
  /* Infinite loop */
  for(;;)
  {
		if(wantWrite){
			wantWrite=0;
			MasterSend[0]=MasterPkgIndex;
      WriteThing(MasterSend);
		}
			StrobeCmd( CMD_TX );
			if(osSemaphoreAcquire(xa1Handle,5)==osOK){
				timeoutSend=0;
			}else{
				timeoutSend=1;
			}
			if(timeoutSend==1){
				continue;
			}
			StrobeCmd(CMD_RX);
			if(osSemaphoreAcquire(xa1Handle,5)==osOK){
				wantWrite=1;
				timeoutReceive=0;
			}else{
				timeoutReceive=1;
			}
			if(timeoutReceive==1){
				StrobeCmd(CMD_STBY);
				continue;
			}
			if((A7131_ReadReg(MODE_REG)&0x20)==0)
			{
				ReadFIFO(64);
				if(RfBuf[0]==MasterPkgIndex){
						MasterPkgIndex++;
					s++;
					wantWrite=1;
				}else{
					wantWrite=0;
				}
			}
			
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the x2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
		int ReceiveTimeout=0;
		int SendTimeout=0;
	A7131_WriteReg2( PLL1_REG, 155 );    
  /* Infinite loop */
  for(;;)
  {
			StrobeCmd2(CMD_RX);
			if(osSemaphoreAcquire(xa2Handle,20)==osOK){
				ReceiveTimeout=0;
			}else{
				ReceiveTimeout=1;
			}
			if(ReceiveTimeout==1){
				StrobeCmd2(CMD_STBY);
				continue;
			}
			if((A7131_ReadReg2(MODE_REG)&0x20)==0)
			{
					ReadFIFO2(64);
					StrobeCmd2(CMD_STBY);
					WriteThing2(RfBuf2);
					StrobeCmd2(CMD_TX);
					osSemaphoreAcquire(xa2Handle,20);
			}
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the x3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for(;;)
  {
		printf("sdfds %d\n",s);
		s=0;
    osDelay(1000);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if (GPIO_Pin&GPIO_PIN_1) {
			osSemaphoreRelease(xa2Handle);
		}
    if (GPIO_Pin&GPIO_PIN_4) {
			osSemaphoreRelease(xa1Handle);
		}
	
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
