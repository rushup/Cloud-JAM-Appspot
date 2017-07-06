 /**
  ******************************************************************************
  * @file    SynchronizationAgent.h
  * @author  Central LAB
  * @version V1.0.0
  * @date    01-April-2016
  * @brief   Header file for SynchronizationAgent.c 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

#ifndef __SYNCHRONIZATION_AGENT_H
#define __SYNCHRONIZATION_AGENT_H

#include <stdint.h>
#include "azure_c_shared_utility/httpapi.h"


#define REGISTRATIONAGENT_DEFAULT_ENDPOINT_IPADDRESS         "stm32ode-register.azurewebsites.net"
#define REGISTRATIONAGENT_DEFAULT_ENDPOINT_TCPPORT           443
#define AZURE_OFFSET_WEBSERVICE_ANSWER_BODY                        8  // FOR "Token":"
#define AZURE_CONNSTRING_TOKEN_SIZE                     56             // TO BE CHECK MAX SIZE TOKEN FOR AZURE TOKEN. ALWAYS 44?           
#define AZURE_FIXED_SIZE_CONNSTRING                     104            // Included margin
#define REGISTRATIONAGENT_DEFAULT_IOT_HUB                "STM32IoTHub.azure-devices.net"



typedef enum
{ 
  REG_SUCCESS           = 0,
  REG_ERROR             = 1,
  REG_TIMEOUT           = 2
} NTP_Status_t;

//int RegistrationAgentStartDefaultConfig(void);
int retrieve_connection_string( HTTP_HANDLE httpServiceHandle, char* connectionString);
int RegistrationAgentStart(HTTP_HANDLE httpServiceHandle, const char* ipAddress,short tcpPort, char* connectionString);

#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
