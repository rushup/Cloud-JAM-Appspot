/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  Central Labs
  * @version V1.1.0
  * @date    14-June-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32xx_it.h"

/** @addtogroup X_NUCLEO_CCA01M1_Applications
* @{
*/

/** @addtogroup Audio_Streaming
* @{
*/  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*             Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

#ifdef USE_STM32L4XX_NUCLEO
/**
  * @brief  This function handles DMA Stream interrupt request for First Device.
  * @param  None
  * @retval None
  */
void AUDIO_OUT1_IRQHandler(void)
{

  HAL_DMA_IRQHandler(hAudioOutSAI[0].hdmatx);
}

/**
  * @brief  This function handles DMA Stream interrupt request for Second Device.
  * @param  None
  * @retval None
  */
void AUDIO_OUT2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(hAudioOutSAI[1].hdmatx);
}

#else

/**
  * @brief  This function handles DMA Stream interrupt request for First Device.
  * @param  None
  * @retval None
  */
void AUDIO_OUT1_IRQHandler(void)
{

  HAL_DMA_IRQHandler(hAudioOutI2s[0].hdmatx);
}

/**
  * @brief  This function handles DMA Stream interrupt request for Second Device.
  * @param  None
  * @retval None
  */
void AUDIO_OUT2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(hAudioOutI2s[1].hdmatx);
}


#endif
/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}

/**
  * @brief  This function handles External lines 4 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
