   /**
  ******************************************************************************
  * @file    IBM_Bluemix_Config.h
  * @author  Central LAB
  * @version V1.0.0
  * @date    17-Oct-2015
  * @brief   Main program body
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


#ifndef __IBM_Bluemix_Config_H
#define __IBM_Bluemix_Config_H

#include "stdio.h"
#include "stdint.h"
#include "MQTTClient.h"

// 1. Enable configuration for Xively Cloud
// 0. Disable configuration for Xively Cloud
#define XIVELY_ENABLED 1

typedef enum {
  QUICKSTART = 0,
  REGISTERED = 1,
} ibm_mode_t;

enum { C_ACCEPTED = 0, C_REFUSED_PROT_V = 1, C_REFUSED_ID = 2, C_REFUSED_SERVER = 3, C_REFUSED_UID_PWD = 4, C_REFUSED_UNAUTHORIZED = 5 };


typedef struct mqtt_vars
{
        uint8_t pub_topic[128];
        uint8_t sub_topic[128];
        uint8_t clientid[64]; 
        enum QoS qos;
	uint8_t username[64];
	uint8_t password[64]; // feedid?
	uint8_t hostname[64];
        uint8_t device_type[64];
        uint8_t org_id[64];        
	uint32_t port;
        uint8_t protocol; // //t -> tcp , s-> secure tcp, c-> secure tcp + certificates
        ibm_mode_t ibm_mode; // QUICKSTART, REGISTERED 
} MQTT_vars;



/* MQTT IBM Functions */
void Config_MQTT_IBM ( MQTT_vars *mqtt_ibm_setup, uint8_t *macadd ); 
void Compose_Quickstart_URL ( uint8_t *url_ibm, uint8_t *macadd ); 


#endif 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
