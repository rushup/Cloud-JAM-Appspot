 /**
  ******************************************************************************
  * @file    SynchronizationAgent.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    01-April-2016
  * @brief   Functions to retrieve data from NTP server 
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

#include "InternetInterface.h"
#include "RegistrationAgent.h"
#include "azure_c_shared_utility/threadapi.h"
#include "TimingSystem.h"
//#include "agenttime.h"
#include "AzureIOTSDKConfig.h"
#include "wifi_interface.h"


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


/** @addtogroup MIDDLEWARES
* @{
*/ 


/** @defgroup  TIMING_ADAPTER_MODULE
  * @brief Interface between Microsoft Azure IoT SDK and Wi-Fi board
  * @{
  */

#define SIZE_BUFFER_ANSWERSERVICE  1024
#define CONVERSION_EPOCHFACTOR  2208988800ul
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#define TEMP_BUFFER_SIZE 4096
#define HTTP_LINE_END   "\r\n"
#define TOKEN_KEY_ID ((uint32_t)0x12481632)
#define TOKEN_FLASH_ADD ((uint32_t)0x08060000)
#define TOKEN_DATA_SIZE ((32+32+6+6)>>2)
#define TOKEN_FLASH_SECTOR FLASH_SECTOR_7
uint32_t tokenData[TOKEN_DATA_SIZE];

#define ENABLE_REGAGENT_DEBUG 0



    
typedef enum
{ 
  AGENT_SUCCESS           = 1,
  AGENT_ERROR           = 0 
} Agent_Status_t;
    

typedef struct _TTCPConnectionCredential_t
{
  char* pIPAddress;
  short tcpPort;
  HTTP_HANDLE tcpHandle;
}TTCPConnectionCredential;

//static THREAD_HANDLE                    threadHandle;
static TTCPConnectionCredential         tcpConnectionCredential;

//static int                              ThreadRegistration(void * pointerArg);
int RegistrationAgent(void * pointerArg, char *connectionString);
static TTCPConnectionCredential*        CreateTCPConnectionCredential(const char* ipAddress,short tcpPort, HTTP_HANDLE handle);
static void                             ReleaseTCPConnectionCredential(TTCPConnectionCredential* pTCPConnectionCredential);

extern bool Get_MAC_Add (char *macadd);
extern WiFi_Status_t GET_Configuration_Value(char* sVar_name,uint32_t *aValue);

static Agent_Status_t Get_UUID_Nucleo(char *UUID_Nucleo);
static Agent_Status_t SendWebServiceMessage(HTTP_HANDLE handle, const char* ipAddress, char *MAC_Address, char *UUID_Nucleo, const char* MessageType);
static Agent_Status_t ParseWebServiceMessage (  uint8_t* bufferServiceAnswer, size_t sizeEffectiveRead, char *connectionString, char *MAC_Address );



//void RegistrationAgentStartDefaultConfig(void)
//{
//  RegistrationAgentStart(REGISTRATIONAGENT_DEFAULT_ENDPOINT_IPADDRESS,REGISTRATIONAGENT_DEFAULT_ENDPOINT_TCPPORT);
//}



/**
 * @brief  Connect with NTP server 
* @param  uint8_t* ipAddress : IP address for NTP server
* @param  short tcpPort : TCP port number
* @retval void
 */
int RegistrationAgentStart(HTTP_HANDLE httpServiceHandle, const char* ipAddress, short tcpPort, char *connectionString)
{
  TTCPConnectionCredential* pTCPConnectionCredential;
  
  pTCPConnectionCredential = CreateTCPConnectionCredential(ipAddress, tcpPort, httpServiceHandle);
  
  // ThreadAPI_Create(&threadHandle,ThreadRegistration,pTCPConnectionCredential);
  return RegistrationAgent(pTCPConnectionCredential, connectionString);
}

/**
 * @brief  Thread managing connection with NTP server  
* @param  void * pointerArg : arguments for thread
* @param  short tcpPort : TCP port number
* @retval int value for success(1)/failure(0)
 */
int RegistrationAgent(void * pointerArg, char *connectionString)
{
  size_t        sizeEffectiveRead;
  uint8_t       bufferServiceAnswer[SIZE_BUFFER_ANSWERSERVICE];  
  uint8_t       flagSuccessOperation;
  uint8_t       MAC_Address[16];
  uint8_t       UUID_Nucleo[32];
  

  TTCPConnectionCredential* pTCPConnectionCredential;
  
  pTCPConnectionCredential = (TTCPConnectionCredential*)pointerArg;
 
  if(pointerArg==0 || pTCPConnectionCredential->pIPAddress==0)
    return 0;
  
  memset(bufferServiceAnswer,0,sizeof(bufferServiceAnswer));
  
  flagSuccessOperation = 0;
  
  printf("Registration Agent Launched\r\n");
  
  if(InternetInterfaceStart()==INTERNETINTERFACERESULT_SUCCESS)
  {
    if(InternetInterfaceSocketOpen(pTCPConnectionCredential->tcpHandle,pTCPConnectionCredential->pIPAddress, pTCPConnectionCredential->tcpPort, SOCKETCONNECTIONTYPE_TLS)==SOCKETSTATUS_CONNECTED)
    {      
      // Get MAC
      if ( !(Get_MAC_Add ((char *) MAC_Address)) ){
           printf("Failed o retrieve MAC. \r\n");
      }
      // Get UUID
      else if ( Get_UUID_Nucleo((char *) UUID_Nucleo) == AGENT_ERROR ) {
           printf("\r\n[Registration][E].Failed o retrieve Nucleo UUID. \r\n");
      }        
      // Compose and send GET-Credentials message  
      // Add delay inside 
      else if ( SendWebServiceMessage(pTCPConnectionCredential->tcpHandle, pTCPConnectionCredential->pIPAddress, (char *)MAC_Address, (char *)UUID_Nucleo, "GET") == AGENT_ERROR ){
              printf("\r\n[Registration][E]. Failed to send GET message \r\n");
      }
      // Retrieve and parse answer for GET-Credentials message
      else if(InternetInterfaceSocketReceive(pTCPConnectionCredential->tcpHandle, bufferServiceAnswer, SIZE_BUFFER_ANSWERSERVICE, &sizeEffectiveRead)==SOCKETOPERATIONRESULT_SUCCESS){
              if ( ParseWebServiceMessage ( bufferServiceAnswer, sizeEffectiveRead, connectionString, (char *) MAC_Address ) == AGENT_ERROR ){
                // Compose and send POST-Register message 
                if ( SendWebServiceMessage(pTCPConnectionCredential->tcpHandle, pTCPConnectionCredential->pIPAddress, (char *)MAC_Address, (char *)UUID_Nucleo, "POST") == AGENT_ERROR ){
                     printf("\r\n[Registration][E]. Failed to send POST message \r\n");
                }   
                else {
                     // Retrieve and parse answer POST-Register message 
                     if(InternetInterfaceSocketReceive(pTCPConnectionCredential->tcpHandle, bufferServiceAnswer, SIZE_BUFFER_ANSWERSERVICE, &sizeEffectiveRead)==SOCKETOPERATIONRESULT_SUCCESS){
                              if ( ParseWebServiceMessage ( bufferServiceAnswer, sizeEffectiveRead, connectionString, (char *) MAC_Address ) == AGENT_ERROR ){
                                printf("\r\n[Registration][E]. Failed to parse POST response  \r\n");
                              } 
                              else { 
                                 flagSuccessOperation = 1;
                              }   
                     }
                     else {
                       printf("\r\n[Registration][E]. Failed to retrieve POST message response from Azure service.  \r\n");
                       if ( sizeEffectiveRead > 0 )
                         printf("[Registration][E]. Receiver buffer overflow.  \r\n");
                       else 
                         printf("[Registration][E]. No data received.  \r\n");
                     }
                  }  
              }
              else {
                printf("\r\n[Registration]. Successfully parsed GET response \r\n");
                flagSuccessOperation = 1;
              } 
         }
      else {
           printf("\r\n[Registration][E]. Failed retrieve GET message response from Azure service.   \r\n");
           if ( sizeEffectiveRead > 0 )
              printf("[Registration][E]. Receiver buffer overflow.  \r\n");
           else 
              printf("[Registration][E]. No data received.  \r\n");

      }   
      InternetInterfaceSocketClose(pTCPConnectionCredential->tcpHandle);
    }
    else {      
      printf("\r\n[Registration][E].Failed connect with Azure service.   \r\n");
      
    }
  }
  
  ReleaseTCPConnectionCredential(pTCPConnectionCredential); 
  
  if(flagSuccessOperation == 1){    
     printf("\r\n[Registration]. Device Regsitration to Microsoft Azure Successfully Completed.   \r\n");
   #if ENABLE_REGAGENT_DEBUG>0
       printf("[Registration]. Connection string is : %s \r\n", connectionString);
   #endif   
     return 0;
  }
  else {
     printf("\r\n[Registration][E]. Device Registration to Microsoft Azure Failed.   \r\n");
     return 1;
  }

}

TTCPConnectionCredential* CreateTCPConnectionCredential(const char* ipAddress, short tcpPort, HTTP_HANDLE handle)
{
  memset(&tcpConnectionCredential,0,sizeof(TTCPConnectionCredential));
  
  tcpConnectionCredential.pIPAddress = (char*)malloc((size_t)(strlen(ipAddress)+1));
  memset(tcpConnectionCredential.pIPAddress,0,strlen(ipAddress)+1);
  
  strcpy(tcpConnectionCredential.pIPAddress,ipAddress);
  tcpConnectionCredential.tcpPort = tcpPort;
  tcpConnectionCredential.tcpHandle = handle;
  
  return &tcpConnectionCredential;
}


Agent_Status_t SendWebServiceMessage(HTTP_HANDLE httpServiceHandle, const char* ipAddress, char *MAC_Address, char *UUID_Nucleo, const char* MessageType)
{  
   // Compose message 
   // messageType + pIPAddress + UUID + MAC   
   // Send message 
   char    buf[TEMP_BUFFER_SIZE];
   //char    relativePath[32];
   int     ret;
   char    postContent[AZURE_CONNSTRING_TOKEN_SIZE];
 
   if (strcmp(MessageType,"GET")==0) {
          printf("\r\n[Registration]. Retrieving connection string... \r\n"); 
          if ((ret = snprintf(buf, sizeof(buf), "%s %s%s HTTP/1.1\r\n", "GET", "/api/tokens/",MAC_Address)) < 0 || ret >= sizeof(buf)){
              return AGENT_ERROR;
         }
   }
   else if (strcmp(MessageType,"POST")==0) {   
          printf("\r\n[Registration]. Registering device and retrieving connection string.... \r\n"); 
#if APPLICATION_SCENARIO > AZURE_ENDLESS_LOOP_BINARY       
          if ((ret = snprintf(buf, sizeof(buf), "%s %s HTTP/1.1\r\n", "POST", "/api/tokens/?demo=true")) < 0 || ret >= sizeof(buf)){
#else
          if ((ret = snprintf(buf, sizeof(buf), "%s %s HTTP/1.1\r\n", "POST", "/api/tokens/")) < 0 || ret >= sizeof(buf)){            
#endif
            return AGENT_ERROR;
        }
        if ((ret = snprintf(postContent, sizeof(postContent), "\"%s+%s\"", UUID_Nucleo, MAC_Address)) < 0 || ret >= sizeof(postContent)){
              return AGENT_ERROR;
        }
   }
   else
     return AGENT_ERROR;
   
#if ENABLE_REGAGENT_DEBUG>0  
   printf("\r\n[Registration]. Sending request: %s \r\n", MessageType); 
   printf("\r\n[Registration]. Request string: %s \r\n", buf); 
#endif   
   
   // Send 
   if (InternetInterfaceSocketSend(httpServiceHandle, buf, strlen(buf))!=SOCKETOPERATIONRESULT_SUCCESS){
              return AGENT_ERROR;
   }
   else {
            ThreadAPI_Sleep(200);
   }
   // Add Header fields
   // EQ. Replace with string in define 
   if ((ret = snprintf(buf, sizeof(buf), "Host: %s\r\n", REGISTRATIONAGENT_DEFAULT_ENDPOINT_IPADDRESS)) < 0 || ret >= sizeof(buf)){
              return AGENT_ERROR;
   }
#if ENABLE_REGAGENT_DEBUG>0  
   printf("\r\n[Registration]. Host string: %s \r\n", buf); 
#endif   

   // Send 
   if (InternetInterfaceSocketSend(httpServiceHandle, buf, strlen(buf))!=SOCKETOPERATIONRESULT_SUCCESS){
              return AGENT_ERROR;
   }
   else {
       ThreadAPI_Sleep(200);
   }
   if (strcmp(MessageType,"POST")==0) { 
        // Add content lenght 
       if ((ret = snprintf(buf, sizeof(buf), "Content-Length: %d\r\n", strlen(postContent))) < 0 || ret >= sizeof(buf)){
                  return AGENT_ERROR;
       }
       if (InternetInterfaceSocketSend(httpServiceHandle, buf, strlen(buf))!=SOCKETOPERATIONRESULT_SUCCESS){
                  return AGENT_ERROR;
        }
        else {
                ThreadAPI_Sleep(200);
        }
         // add content-type
        if ((ret = snprintf(buf, sizeof(buf), "Content-Type: application/json\r\n")) < 0 || ret >= sizeof(buf)){
              return AGENT_ERROR;
        }
        // Send 
         if (InternetInterfaceSocketSend(httpServiceHandle, buf, strlen(buf))!=SOCKETOPERATIONRESULT_SUCCESS){
                    return AGENT_ERROR;
         }
         else {
                ThreadAPI_Sleep(200);
        }
   }
    // Send end line 
    if ( InternetInterfaceSocketSend(httpServiceHandle,HTTP_LINE_END, strlen(HTTP_LINE_END) )!=SOCKETOPERATIONRESULT_SUCCESS){
              return AGENT_ERROR;
         }
    else {
            ThreadAPI_Sleep(200);
    }

    if (strcmp(MessageType,"POST")==0) {   
        if (InternetInterfaceSocketSend(httpServiceHandle, postContent, strlen(postContent))!=SOCKETOPERATIONRESULT_SUCCESS){
              return AGENT_ERROR;
        }
        else {
                ThreadAPI_Sleep(200);
        }
            // Send end line 
        if ( InternetInterfaceSocketSend(httpServiceHandle,HTTP_LINE_END, strlen(HTTP_LINE_END) )!=SOCKETOPERATIONRESULT_SUCCESS){
                  return AGENT_ERROR;
             }
        else {
                ThreadAPI_Sleep(200);
        }
    }        
  
   return AGENT_SUCCESS;
}

Agent_Status_t ParseWebServiceMessage (  uint8_t* bufferServiceAnswer, size_t sizeEffectiveRead, char *connectionString, char * MAC_Address )
{
  // Parse answer 
  // check error or connection string 
  // if connection string found --> copy in FLASH memory. 
  char * pConnString;
  char connToken[AZURE_CONNSTRING_TOKEN_SIZE];
  int i = 0;
 
#if ENABLE_REGAGENT_DEBUG>0  
   printf("\r\n[Registration]. Received response %s \r\n", bufferServiceAnswer);
#endif  
   
  if(strstr((char *)(bufferServiceAnswer), "not found") != NULL) {
    return AGENT_ERROR;
  }
  else if (strstr((char *)(bufferServiceAnswer), "disabled") != NULL) {
    printf ("\r\n[Registration]. Device is disabled. \r\n \r\n");
    return AGENT_ERROR;
  }
  else {
    pConnString = strstr((char *)(bufferServiceAnswer), "Token");
    
    // "HostName=sensor2cloud-st.azure-devices.net;DeviceId=0080E1B7C43D;SharedAccessKey=oHWzJaHFsTd64cndbSV9LLxiOfOgShSR0NldUjKhKt4="
    // "HostName=sensor2cloud-st.azure-devices.net;DeviceId=0080E1B7C43D;SharedAccessKey=oHWzJaHFsTd64cndbSV9LLxiOfOgShSR0NldUjKhKt4="
    // REGISTRATIONAGENT_DEFAULT_ENDPOINT_IPADDRESS
    
    
    if ( pConnString != NULL){
          pConnString += AZURE_OFFSET_WEBSERVICE_ANSWER_BODY;
          while ( *pConnString != '\"' ){
                 connToken[i] = *pConnString;
                 pConnString++;
                 i++;
          }
          //connToken[i] = *pConnString;
          connToken[i] = '\0';
          sprintf(connectionString,"HostName=%s;DeviceId=%s;SharedAccessKey=%s",REGISTRATIONAGENT_DEFAULT_IOT_HUB, MAC_Address, connToken );
#if ENABLE_REGAGENT_DEBUG>0  
          printf("[Registration]. Token is is %s \r\n", connToken);
          printf("[Registration]. Connection string is %s \r\n", connectionString);
#endif  
          
          return AGENT_SUCCESS;     
        }
    else 
      return AGENT_ERROR; 
   }
        
}


static Agent_Status_t Get_UUID_Nucleo(char *UUID_Nucleo)
{
    uint32_t idPart1 = STM32_UUID[0];
    uint32_t idPart2 = STM32_UUID[1];
    uint32_t idPart3 = STM32_UUID[2];
    
    sprintf (UUID_Nucleo,"%.10d%.10d%.10d",idPart1,idPart2,idPart3);
#if ENABLE_REGAGENT_DEBUG>0      
    printf ("\r\n[Registration]. UUID_Nucleo: %s \r\n", UUID_Nucleo);
#endif    
    return AGENT_SUCCESS;
}

void ReleaseTCPConnectionCredential(TTCPConnectionCredential* pTCPConnectionCredential)
{
  free(tcpConnectionCredential.pIPAddress);
}


// Refer to HAL
#if 0

/**
  * @brief  Read Access Point parameters from FLASH
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
 static Agent_Status_t ReCallSSIDPasswordFromMemory(void)
{
  static Agent_Status_t status = AGENT_ERROR;
  
  uint32_t Address = TOKEN_FLASH_ADD;
  __IO uint32_t data32 = *(__IO uint32_t*) Address;
  if(data32 == TOKEN_KEY_ID){
    int32_t ReadIndex;

    for(ReadIndex=0;ReadIndex<TOKEN_DATA_SIZE;ReadIndex++){
      Address +=4;
      data32 = *(__IO uint32_t*) Address;
      tokenData[ReadIndex]=data32;
    }
    status = MODULE_SUCCESS;
  }
  else
    printf("\r\nFLASH Keyword not found.");
  
  return status;
}



/**
  * @brief  Save Access Point parameters to FLASH
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
 static Agent_Status_t SaveSSIDPasswordToMemory(void)
{
  System_Status_t status = AGENT_ERROR;

  /* Reset Before The data in Memory */
  status = ResetSSIDPasswordInMemory();

  if(status) {
    /* Store in Flash Memory */
    uint32_t Address = TOKEN_FLASH_ADD;
    int32_t WriteIndex;

   /* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();
   
   /* Write the Magic Number */
   {
     uint32_t MagicNumber = TOKEN_KEY_ID;
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,MagicNumber) == HAL_OK){
        Address = Address + 4;
      } else {
          printf("\r\nError while writing in FLASH");
          status = MODULE_ERROR;
        }
   }
   
    /* Write the Wifi */
    for(WriteIndex=0;WriteIndex<UNION_DATA_SIZE;WriteIndex++){
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,UnionWifiToken.Data[WriteIndex]) == HAL_OK){
        Address = Address + 4;
      } else {
          printf("\r\nError while writing in FLASH");
          status = MODULE_ERROR;
        }
      }
    

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
    
    printf("\n\rSaveSSIDPasswordToMemory OK");
  }
  else
    printf("\n\rError while resetting FLASH memory");
    
  return status;
}
#endif


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
