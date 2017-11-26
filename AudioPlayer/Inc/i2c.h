/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "cs43l22.h"
   
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

#define AUDIO_I2C_ADDRESS                       ((uint16_t) 0x94)

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* High Layer codec functions */
uint32_t AUDIO_Init(uint16_t DeviceAddr, uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
void     AUDIO_DeInit(void);
uint32_t AUDIO_ReadID(uint16_t DeviceAddr);
uint32_t AUDIO_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size);
uint32_t AUDIO_Pause(uint16_t DeviceAddr);
uint32_t AUDIO_Resume(uint16_t DeviceAddr);
uint32_t AUDIO_Stop(uint16_t DeviceAddr, uint32_t Cmd);
uint32_t AUDIO_SetVolume(uint16_t DeviceAddr, uint8_t Volume);
uint32_t AUDIO_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq);
uint32_t AUDIO_SetMute(uint16_t DeviceAddr, uint32_t Cmd);
uint32_t AUDIO_SetOutputMode(uint16_t DeviceAddr, uint8_t Output);
uint32_t AUDIO_Reset(uint16_t DeviceAddr);

/* AUDIO IO functions */
void      AUDIO_IO_Init(void);
void      AUDIO_IO_DeInit(void);
void      AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t   AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
