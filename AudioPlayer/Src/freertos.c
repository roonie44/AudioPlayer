/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "stm32l4xx_hal.h"
#include "mxconstants.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "fatfs.h"
#include "sai.h"
#include "i2c.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId appTaskHandle;
osThreadId usbTaskHandle;
osThreadId audioTaskHandle;
osThreadId displayTaskHandle;
osMessageQId joystickInputQueueHandle;
osMutexId qspiMutexHandle;

/* USER CODE BEGIN Variables */

#define AUDIO_BUFFER_SIZE          (1024 * 8)       /* Size (in bytes) of the buffer containing the PCM samples */
#define WAV_FILE_HEADER_SIZE        44              /* Size (in bytes) of a WAV header */

typedef enum {
  AUDIOPLAYER_STATE_STOPPED = 0,
  AUDIOPLAYER_STATE_RUNNING,
  AUDIOPLAYER_STATE_PAUSED
} AudioPlayer_StateTypeDef;

FIL                       File;
FATFS                     FatFs;
uint8_t                   Buffer[AUDIO_BUFFER_SIZE];    // Buffer containig the PCM samples to play  
uint32_t                  ActualPosition;               // Actual read offset
AudioPlayer_StateTypeDef  AppState = AUDIOPLAYER_STATE_STOPPED;
  
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void appTaskBody(void const * argument);
void usbTaskBody(void const * argument);
void audioTaskBody(void const * argument);
void displayTaskBody(void const * argument);

extern void MX_FATFS_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void Error_Handler(void);

static void AudioPlayer_Start(void);
static void AudioPlayer_Pause(void);
static void AudioPlayer_Stop(void);
static void AudioPlayer_Resume(void);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of qspiMutex */
  osMutexDef(qspiMutex);
  qspiMutexHandle = osMutexCreate(osMutex(qspiMutex));

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
  /* definition and creation of appTask */
  osThreadDef(appTask, appTaskBody, osPriorityNormal, 0, 4096);
  appTaskHandle = osThreadCreate(osThread(appTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, usbTaskBody, osPriorityHigh, 0, 256);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* definition and creation of audioTask */
  osThreadDef(audioTask, audioTaskBody, osPriorityNormal, 0, 1024);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, displayTaskBody, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of joystickInputQueue */
  osMessageQDef(joystickInputQueue, 1, uint16_t);
  joystickInputQueueHandle = osMessageCreate(osMessageQ(joystickInputQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* appTaskBody function */
void appTaskBody(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();

  /* USER CODE BEGIN appTaskBody */
  
  osEvent event;
  
  /* Wait for the qspiMutex */
  osMutexWait(qspiMutexHandle, osWaitForever);

  /* Register the file system object to the FatFs module */
  if(f_mount(&FatFs, (TCHAR const*)USER_Path, 1) != FR_OK)
  {
    /* FatFs Initialization Error */
    if (f_mkfs((TCHAR const*)USER_Path, 0, 128) != FR_OK)
    {
      Error_Handler();
    }
    else {
      /* Second trial to register the file system object */
      if(f_mount(&FatFs, (TCHAR const*)USER_Path, 1) != FR_OK)
      {
        Error_Handler();
      }
    }
  }
  
  /* FatFS file write test */
  if(f_open(&File, "FATFSOK", FA_CREATE_NEW | FA_WRITE) == FR_OK)
  {
    f_printf(&File, "FatFS is working properly.\n");
    f_close(&File);
  }

  /* Release the qspiMutex */
  osMutexRelease(qspiMutexHandle);
    
  /* Initialize audio driver */
  if(CS43L22_ID != AUDIO_ReadID(AUDIO_I2C_ADDRESS))
  {
    Error_Handler();
  }
  if(0 != AUDIO_Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 60, AUDIO_FREQUENCY_44K))
  {
    Error_Handler();
  }
    
  /* Infinite loop */
  for(;;)
  {
    /* Wait for the joystick button press */
    event = osMessageGet(joystickInputQueueHandle, osWaitForever);
    
    if (event.status == osEventMessage)
    {
      switch(AppState)
      {
      case AUDIOPLAYER_STATE_STOPPED:
        switch(event.value.v)
        {
        case JOY_RIGHT_Pin:
          AudioPlayer_Start();
          break;
        default:
          break;
        }
        break;
      case AUDIOPLAYER_STATE_RUNNING:
        switch(event.value.v)
        {
        case JOY_CENTER_Pin:
          AudioPlayer_Pause();
          break;
        case JOY_LEFT_Pin:
          AudioPlayer_Stop();
          break;
        default:
          break;
        }
        break;
      case AUDIOPLAYER_STATE_PAUSED:
        switch(event.value.v)
        {
        case JOY_RIGHT_Pin:
          AudioPlayer_Resume();
          break;
        case JOY_LEFT_Pin:
          AudioPlayer_Stop();
          break;
        default:
          break;
        }
        break;
      default:
        break;
      }
    }
  }
  /* USER CODE END appTaskBody */
}

/* usbTaskBody function */
void usbTaskBody(void const * argument)
{
  /* USER CODE BEGIN usbTaskBody */
  uint32_t USB_VBUS_counter = 0;
  
  /* Infinite loop */
  for(;;)
  {
    USB_VBUS_counter = 0;
    /* USB_VBUS availability check */
    while (USB_VBUS_counter < 5)
    {
      osDelay(10);
      if (HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) != GPIO_PIN_RESET)
      {
        USB_VBUS_counter++;
      }
      else {
        break;
      }
    }
    if(USB_VBUS_counter >= 5)
    {
      /* STOP the audio player, if it is running */
      if (AppState != AUDIOPLAYER_STATE_STOPPED)
      {
        AudioPlayer_Stop();
      }
      
      /* Wait for the qspiMutex */
      osMutexWait(qspiMutexHandle, osWaitForever);
      
      /* Initialize USB peripheral */
      MX_USB_DEVICE_Init();
      
      /* RED LED ON */
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      
      /* USB_VBUS availability check */
      while (USB_VBUS_counter)
      {
        /* Wait 100ms, then check the USB_VBUS availability */
        osDelay(100);
        
        if (HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) == GPIO_PIN_RESET)
        {
          USB_VBUS_counter--;
        }
        else {
          USB_VBUS_counter = 5;
        }
      }
      
      /* Deinitialize USB peripheral */
      USBD_DeInit(&hUsbDeviceFS);
            
      /* Release the qspiMutex */
      osMutexRelease(qspiMutexHandle);
      
      /* RED LED OFF */
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
    /* Every 1s we will check if USB_VBUS is available */
    osDelay(1000);
  }
  /* USER CODE END usbTaskBody */
}
  
/* audioTaskBody function */
void audioTaskBody(void const * argument)
{
  /* USER CODE BEGIN audioTaskBody */
  uint32_t bufferOffset = 0;
  uint32_t bytesRead = 0;
  osEvent event;
      
  /* Pause this task execution */
  osThreadSuspend(NULL);
  
  /* Infinite loop */
  for(;;)
  {
    event = osSignalWait(A | B, osWaitForever);
    if( event.status == osEventSignal )
    {
      switch(event.value.v)
      {
      case B:
        bufferOffset = AUDIO_BUFFER_SIZE/2;
        break;
      case A:
        bufferOffset = 0;
        break;
      default:
        /* We were not fast enough to deliver new data => clean the buffer => speed up SYSCLK */
        break;
      }
      
      /* Wait for the qspiMutex */
      osMutexWait(qspiMutexHandle, osWaitForever);

      /* Fill in the buffer with new data */
      if (f_read(&File, (uint8_t *)(Buffer + bufferOffset), AUDIO_BUFFER_SIZE/2, &bytesRead) != FR_OK)
      {
        Error_Handler();
      }

      /* Some processing of the audio data could be added here */    
      
      /* GREEN LED ON */
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      
      ActualPosition += bytesRead;
      
      /* Check the end of the file */
      if((ActualPosition + (AUDIO_BUFFER_SIZE/2)) > File.fsize)
      {
        if (f_rewind(&File) != FR_OK)
        {
          Error_Handler();
        }
            
        if (f_lseek(&File, WAV_FILE_HEADER_SIZE) != FR_OK)
        {
          Error_Handler();
        }
        
        /* Skip the WAV header - this we don't want to listen to :) */
        ActualPosition = WAV_FILE_HEADER_SIZE;
      }
      
      /* Release the qspiMutex */
      osMutexRelease(qspiMutexHandle);
    }
    else {
      /* Don't care now (not implemented) */
    }
  }
  /* USER CODE END audioTaskBody */
}

/* displayTaskBody function */
void displayTaskBody(void const * argument)
{
  /* USER CODE BEGIN displayTaskBody */
  uint32_t timeout = 100;
  uint32_t prevAppState = -1;
  
  /* Infinite loop */
  for(;;)
  {
    if (prevAppState != AppState)
    {
      switch(AppState)
      {
      case AUDIOPLAYER_STATE_STOPPED:
        LCD_GLASS_Clear();
        LCD_GLASS_DisplayString("STOPPD");
        break;
      case AUDIOPLAYER_STATE_RUNNING:
        LCD_GLASS_Clear();
        LCD_GLASS_DisplayString("PLAYIN");
        break;
      case AUDIOPLAYER_STATE_PAUSED:
        LCD_GLASS_Clear();
        LCD_GLASS_DisplayString("PAUSED");
        break;
      default:
        Error_Handler();
        break;
      }
    }
    else {
      /* Add some animations or display content changes depending on the App State */
    }
  
    prevAppState = AppState;
  
    osDelay(timeout);
  }
  /* USER CODE END displayTaskBody */
}

/* USER CODE BEGIN Application */

/**
  * @brief Application Error Handler
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
    /* Error management behavior to be added here */
  }
}

/**
  * @brief Tx Transfer completed callbacks.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_TxCpltCallback could be implemented in the user file
   */ 
  osSignalSet(audioTaskHandle, B);
}

/**
  * @brief Tx Transfer Half completed callbacks
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_TxHalfCpltCallback could be implenetd in the user file
   */ 
  osSignalSet(audioTaskHandle, A);
}

/**
  * @brief  GPIO EXTI Callback function
  *         Handle USB VBUS detection upon External interrupt
  * @param  GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case JOY_CENTER_Pin:
    osMessagePut(joystickInputQueueHandle, JOY_CENTER_Pin, 0);
    break;
  case JOY_LEFT_Pin:
    osMessagePut(joystickInputQueueHandle, JOY_LEFT_Pin, 0);
    break;
  case JOY_RIGHT_Pin:
    osMessagePut(joystickInputQueueHandle, JOY_RIGHT_Pin, 0);
    break;
  case JOY_UP_Pin:
    osMessagePut(joystickInputQueueHandle, JOY_UP_Pin, 0);
    break;
  case JOY_DOWN_Pin:
    osMessagePut(joystickInputQueueHandle, JOY_DOWN_Pin, 0);
    break;
  default:
    Error_Handler();
    break;
  }
}

/**
  * @brief AudioPlayer Start method
  * @param  None
  * @retval None
  */
static void AudioPlayer_Start(void)
{
  /* Wake-Up external audio Codec and enable output */
  if(0 != AUDIO_Play(AUDIO_I2C_ADDRESS, NULL, 0))
  {
    Error_Handler();
  }
  
  /* Wait for the qspiMutex */
  osMutexWait(qspiMutexHandle, osWaitForever);
  
  /* Create and Open a new text file object with write access */
  if(f_open(&File, (TCHAR const*)"audio.wav", FA_READ) != FR_OK)
  {
    Error_Handler();
  }
  
  /* Skip the WAV header - this we don't want to listen to :) */
  ActualPosition = WAV_FILE_HEADER_SIZE;
  
  /* Move file pointer to appropriate address - keep in mind that the file consists of many blocks in sectors */
  if (f_lseek(&File, WAV_FILE_HEADER_SIZE) != FR_OK)
  {
    Error_Handler();
  }
  
  /* Release the qspiMutex */
  osMutexRelease(qspiMutexHandle);
  
  /* Start the SAI DMA transfer from the Buffer */
  if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Buffer, AUDIO_BUFFER_SIZE/2))
  {
    Error_Handler();
  }
  
  /* Change the application state */
  AppState = AUDIOPLAYER_STATE_RUNNING;

  /* Resume audioTask */
  osThreadResume(audioTaskHandle);
}

/**
  * @brief AudioPlayer Resume method
  * @param  None
  * @retval None
  */
static void AudioPlayer_Resume(void)
{
  /* Resume DMA transfer */
  if (HAL_SAI_DMAResume(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Resume the playback */
  if (AUDIO_Resume(AUDIO_I2C_ADDRESS) != 0)
  {
    Error_Handler();
  }

  AppState = AUDIOPLAYER_STATE_RUNNING;   
  
  /* Resume audioTask */
  osThreadResume(audioTaskHandle);
}

/**
  * @brief AudioPlayer Pause method
  * @param  None
  * @retval None
  */
static void AudioPlayer_Pause(void)
{
  /* Stop the playback */
  if (AUDIO_Pause(AUDIO_I2C_ADDRESS) != 0)
  {
    Error_Handler();
  }
  /* Stop DMA transfer */
  if (HAL_SAI_DMAPause(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Wait for the qspiMutex */
  osMutexWait(qspiMutexHandle, osWaitForever);
  
  /* Pause the audioTask */
  osThreadSuspend(audioTaskHandle);
  
  /* Release the qspiMutex */
  osMutexRelease(qspiMutexHandle);
  
  /* Turn OFF the PLAY signaling LED */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  
  AppState = AUDIOPLAYER_STATE_PAUSED;
}

/**
  * @brief AudioPlayer Stop method
  * @param  None
  * @retval None
  */
static void AudioPlayer_Stop(void)
{
  /* Stop the playback */
  if (AUDIO_Stop(AUDIO_I2C_ADDRESS, CODEC_PDWN_SW) != 0)
  {
    Error_Handler();
  }
  /* Stop DMA transfer */
  if (HAL_SAI_DMAStop(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Wait for the qspiMutex */
  osMutexWait(qspiMutexHandle, osWaitForever);
  
  /* Pause the audioTask */
  osThreadSuspend(audioTaskHandle);
  
  /* Close the Audio file */
  if(f_close(&File) != FR_OK)
  {
    Error_Handler();
  }
  
  /* Release the qspiMutex */
  osMutexRelease(qspiMutexHandle);
  
  /* Turn OFF the PLAY signaling LED */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  
  AppState = AUDIOPLAYER_STATE_STOPPED;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
