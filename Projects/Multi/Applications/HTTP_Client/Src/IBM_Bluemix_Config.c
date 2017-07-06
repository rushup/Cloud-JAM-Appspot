 /**
  ******************************************************************************
  * @file    IBM_Bluemix_Config.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    17-Jan-2016
  * @brief   Configuration file for connection with IBM Bluemix Cloud
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

#include "IBM_Bluemix_Config.h"

/** @addtogroup FP-CLD-BLUEMIX1
  * @{
  */


/**
  * @brief  Compose URL for Quickstart visualization of sensors data
  * @param  url_ibm : output buffer containing the URL for data visualization
  *         macadd : buffer containing MAC address
  * @retval None
  */
 void Compose_Quickstart_URL ( uint8_t *url_ibm, uint8_t *macadd ) 
{
     strcpy((char *)url_ibm,"quickstart.internetofthings.ibmcloud.com/#/device/");
     strcat((char *)url_ibm,(char *)macadd);
     strcat((char *)url_ibm,"/sensor/");  
   
     return;
}

/**
  * @brief  Configure MQTT parameters according to QUICKSTART/REGISTERED mode. REGISTERED mode requires custom information and a Bluemix account
  * @param  mqtt_ibm_setup : handler of parameters for connection with IBM MQTT broker; 
  *         macadd : mac address of Wi-Fi
  * @retval None
  */
void Config_MQTT_IBM ( MQTT_vars *mqtt_ibm_setup,  uint8_t *macadd ) 
{
#if XIVELY_ENABLED > 0
     /* Default Configuration for QUICKSTART. REGISTERED mode requires account on Bluemix */
    mqtt_ibm_setup->ibm_mode = REGISTERED;
#else
    mqtt_ibm_setup->ibm_mode = QUICKSTART;
#endif        
   
    /* Quickstart visualization */  
    if ( mqtt_ibm_setup->ibm_mode == QUICKSTART )
    {
        strcpy((char*)mqtt_ibm_setup->pub_topic, "iot-2/evt/status/fmt/json");
        strcpy((char*)mqtt_ibm_setup->sub_topic, "");  
        strcpy((char*)mqtt_ibm_setup->clientid,"d:quickstart:LED-controller:");
        strcat((char*)mqtt_ibm_setup->clientid,"emulate-linkit");
        mqtt_ibm_setup->qos = QOS0;
        strcpy((char*)mqtt_ibm_setup->username,"");
        strcpy((char*)mqtt_ibm_setup->password,"");  
        strcpy((char*)mqtt_ibm_setup->hostname,"quickstart.messaging.internetofthings.ibmcloud.com");
        strcpy((char*)mqtt_ibm_setup->device_type,"");
        strcpy((char*)mqtt_ibm_setup->org_id,"");    
        mqtt_ibm_setup->port = 1883; //TLS 8883
        mqtt_ibm_setup->protocol = 't'; // TLS no certificates
    }
    else if ( mqtt_ibm_setup->ibm_mode == REGISTERED )
    {
#if XIVELY_ENABLED > 0
       /* XIVELY  */
	   /*
        strcpy((char*)mqtt_ibm_setup->pub_topic, "xi/blue/v1/aca1ee2d-c46a-4d7c-974c-971b4a9451c5/d/ce99f67b-445b-4078-8aed-5d2b9be24d8e/Sensors");
        strcpy((char*)mqtt_ibm_setup->sub_topic, "xi/blue/v1/aca1ee2d-c46a-4d7c-974c-971b4a9451c5/d/ce99f67b-445b-4078-8aed-5d2b9be24d8e/Control");  
        mqtt_ibm_setup->qos = QOS0;
        strcpy((char*)mqtt_ibm_setup->username,"ce99f67b-445b-4078-8aed-5d2b9be24d8e"); 
        strcpy((char*)mqtt_ibm_setup->password,"88mNodV1ko9ThyJ2HP1PoUKxnC1kGjKmOsdFJAZLxUQ=");
        strcpy((char*)mqtt_ibm_setup->hostname,"broker.xively.com");
        strcpy((char *)mqtt_ibm_setup->clientid, "ce99f67b-445b-4078-8aed-5d2b9be24d8e");
        mqtt_ibm_setup->port = 8883;
        mqtt_ibm_setup->protocol = 's'; 
		*/
        strcpy((char*)mqtt_ibm_setup->pub_topic, "iot-2/evt/nucleo_event/fmt/json");
        strcpy((char*)mqtt_ibm_setup->sub_topic, "iot-2/cmd/nucleo_cmd/fmt/json");  
        mqtt_ibm_setup->qos = QOS0;
        strcpy((char*)mqtt_ibm_setup->username,"use-token-auth"); 
        strcpy((char*)mqtt_ibm_setup->password,"i7TG0X(05wdcr)dDlx");
		    strcpy((char*)mqtt_ibm_setup->hostname,"bslx9u.messaging.internetofthings.ibmcloud.com");
        strcpy((char*)mqtt_ibm_setup->device_type,"Nucleo-board");
        strcpy((char*)mqtt_ibm_setup->org_id,"bslx9u");  
        strcpy((char *)mqtt_ibm_setup->clientid, "d:");
        strcat((char *)mqtt_ibm_setup->clientid, (char *)mqtt_ibm_setup->org_id);
        strcat((char *)mqtt_ibm_setup->clientid,":");
        strcat((char *)mqtt_ibm_setup->clientid,(char *)mqtt_ibm_setup->device_type);
        strcat((char *)mqtt_ibm_setup->clientid,":"); 
        strcat((char*)mqtt_ibm_setup->clientid,"STM32-F4-L4");
        mqtt_ibm_setup->port = 1883;
        mqtt_ibm_setup->protocol = 't'; 
		
        
#else      
        strcpy((char*)mqtt_ibm_setup->pub_topic, "iot-2/evt/status/fmt/json");
        strcpy((char*)mqtt_ibm_setup->sub_topic, "iot-2/cmd/+/fmt/string");  
        mqtt_ibm_setup->qos = QOS0;
        strcpy((char*)mqtt_ibm_setup->username,"use-token-auth"); 
        strcpy((char*)mqtt_ibm_setup->password,"uUURNRbeQQaX+Fvi&8");
        strcpy((char*)mqtt_ibm_setup->hostname,"1w8a05.messaging.internetofthings.ibmcloud.com");
        strcpy((char*)mqtt_ibm_setup->device_type,"stm32_nucleo");
        strcpy((char*)mqtt_ibm_setup->org_id,"1w8a05");  
        strcpy((char *)mqtt_ibm_setup->clientid, "d:");
        strcat((char *)mqtt_ibm_setup->clientid, (char *)mqtt_ibm_setup->org_id);
        strcat((char *)mqtt_ibm_setup->clientid,":");
        strcat((char *)mqtt_ibm_setup->clientid,(char *)mqtt_ibm_setup->device_type);
        strcat((char *)mqtt_ibm_setup->clientid,":"); 
        strcat((char*)mqtt_ibm_setup->clientid,(char *)macadd);
 
        mqtt_ibm_setup->port = 8883; //TLS
        mqtt_ibm_setup->protocol = 's'; // TLS no certificates  
#endif
    }

    return;
}



/**
 * @}
 */
