/**
 ******************************************************************************
 * @file    audioProcessor.c
 * @author  AST Robotics Team
 * @version V0.0.1
 * @date    08-July-2014
 * @brief   This header file contains the functions prototypes for the voice-uP driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __AUDIOPROCESSOR_H
#define __AUDIOPROCESSOR_H

#ifdef __cplusplus
extern "C" {
#endif
  
  /* Includes ------------------------------------------------------------------*/
#include <stdint.h> 
  
  
 /** 
  * @brief  AudioProcessor initialization structure definition  
  */
  typedef struct
  {
    uint8_t SamplingFrequency;
    uint8_t DecimationFactor;
    uint8_t MicNumber;
    uint8_t MicGain[4];
    uint8_t LFCEnable;
    uint8_t IsHDMic;      
    uint8_t OutputPeripherals;  
    uint8_t USBMicOut;
    uint8_t I2SMicOut;
    uint8_t SPIMicOut;
    uint8_t I2SConfiguration;
    uint8_t SPIConfiguration;   
  }AUDIOPROCESSOR_InitTypeDef;

  
  /** 
  * @brief  AudioProcessor driver structure definition  
  */ 
  typedef struct
  {  
    void        (*Init)(AUDIOPROCESSOR_InitTypeDef *);
    void        (*PowerOff)(void);
    uint8_t     (*ReadID)(void);
    void        (*Reset)(void);
    void        (*Start)(void);
    void        (*Stop)(void);
    void        (*SetMicGain)(uint8_t, uint8_t);   
  }AUDIOPROCESSOR_DrvTypeDef;
  
  
  
  
  
#ifdef __cplusplus
}
#endif

#endif /* __AUDIOPROCESSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
