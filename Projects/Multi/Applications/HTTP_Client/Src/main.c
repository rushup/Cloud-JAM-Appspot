   /**
  ******************************************************************************
  * @file    main.c
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stm32f4xx_I2C.h"
#include "stm32f4xx_periph_conf.h"
#include "TLocalBuffer.h"


//#include "lib_TagType4.h"



/** @defgroup  
  * @brief Expample code for HTTP client
  * @{
  */

/* Extern ()      ------------------------------------------------------------*/
extern UART_HandleTypeDef UartMsgHandle;
extern char print_msg_buff[512];
extern WiFi_Status_t GET_Configuration_Value(char* sVar_name,uint32_t *aValue);
extern int uartReceiveChar(void);

/* Private define ------------------------------------------------------------*/
#define WIFI_SCAN_BUFFER_LIST           32
#define WIFI_SCAN_BUFFER_SIZE           512
#define APPLICATION_DEBUG_MSG           1
// Timeout for setting wifi parameters 
#define APP_TIMEOUT 5000
#define NDEF_WIFI 32
#define BLUEMSYS_FLASH_ADD ((uint32_t)0x08060000)
#define BLUEMSYS_FLASH_SECTOR FLASH_SECTOR_7
#define WIFI_CHECK_SSID_KEY ((uint32_t)0x12345678)
#define UNION_DATA_SIZE ((32+32+6+6)>>2)
#define USART_SPEED_MSG	460800
// HTTP DEFINES
// https://iotportal-148413.appspot.com/_ah/api/data/v1/save
#define TEST_SERVER_NAME "iotportal-148413.appspot.com"
#define TEST_SERVER_PORT 443
#define HTTP_BUFFER_SIZE 512
// To handle received data
#define APPLICATION_BUFFER_SIZE         1024    // Need to be the same as TLOCALBUFFER_SIZE
#define STM32_UUID_SIZE									32
#define HTTP_POST_CONTENT_SIZE          1024
#define HTTP_LINE_END   "\r\n"
#define STM32_UUID ((uint32_t *)0x1FFF7A10)   // STM32 Id

uint8_t receive_data[APPLICATION_BUFFER_SIZE];
uint32_t application_idx = 0;
static TLocalBuffer localBufferReading;
// MAC address
uint8_t DisplayName[32];




typedef union
{
  uint32_t Data[UNION_DATA_SIZE];
  sWifiTokenInfo TokenInfo;
}uWifiTokenInfo;
static uWifiTokenInfo UnionWifiToken;
#define WIFITOKEN UnionWifiToken.TokenInfo


typedef enum
{ 
  MODULE_SUCCESS           = 0,
  MODULE_ERROR           = -1 
} System_Status_t;


// https://httpbin.org/
typedef struct http_vars
{
  uint8_t hostname[128];
	uint32_t port_number;
	uint8_t socket_id;
  uint8_t protocol; // //t -> tcp , s-> secure tcp, c-> secure tcp + certificates
} http_endpoint;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static System_Status_t  Get_MAC_Add (char *macadd);
static System_Status_t ReCallSSIDPasswordFromMemory(void);
static System_Status_t ResetSSIDPasswordInMemory(void);
static System_Status_t SaveSSIDPasswordToMemory(void);
static System_Status_t ConfigAPSettings(void);
static System_Status_t wifi_get_AP_settings(void);
static System_Status_t InitNucleo(int i_usart_speed);
static System_Status_t InitSensors(void);
static System_Status_t ConfigWiFi(void);
static System_Status_t ConfigServer(http_endpoint *server);
static System_Status_t write_http_get (http_endpoint *server);
static System_Status_t write_http_post(http_endpoint *server);
static System_Status_t read_http_response(void);
static System_Status_t Get_UUID_Nucleo(char *UUID_Nucleo);
int wifi_socket_read (unsigned char* buff, int ch);
void prepare_json_pkt (uint8_t * buffer);



#if CONFIG_USE_NFC > 0
static System_Status_t InitNFC(void);
static System_Status_t ReadWifiTokenFromNFC(void);
static System_Status_t WriteURLtoNFC(char* UrlForNFC);
#endif

static void NotifyLEDOn();
static void NotifyLEDOff(void);
static void NotifyLEDBlink(unsigned int msPeriod);


// WiFi related variables 
typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_idle,
  wifi_state_connected,
  wifi_state_connecting,
  wifi_state_disconnected,
  wifi_state_error,
	wifi_state_socket_create,
  wifi_state_socket_close,
  wifi_state_reconnect,
	wifi_state_resuming,
	wifi_state_server_close,
  wifi_state_http_get,
  wifi_state_http_post,
  wifi_undefine_state       = 0xFF,  
} wifi_state_t;
wifi_state_t wifi_state;
wifi_config config;
wifi_scan net_scan[WIFI_SCAN_BUFFER_LIST];
uint8_t console_input[1], console_count=0;
wifi_bool set_AP_config = WIFI_FALSE, SSID_found = WIFI_FALSE;


#if 0
  /* Default configuration SSID/PWD */ 
  char *ssid = "ST";
  char *seckey = "demodemo";  
#else
  char *ssid =  NULL;
  char *seckey = NULL;
#endif
WiFi_Priv_Mode mode = WPA_Personal; 
TIM_IC_InitTypeDef       sConfig;
void TIM4_IRQHandler(void);

#if CONFIG_USE_MEMS > 0	
/* Sensors. Private variables ---------------------------------------------------------*/  
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;
#endif
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
 


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
	System_Status_t status_http = MODULE_ERROR;
  WiFi_Status_t status_spwf = WiFi_MODULE_SUCCESS;
  int8_t ret_code;
  uint32_t i;
	http_endpoint test_server;
	
  
   /* Init Nucleo Board */
  if (InitNucleo (USART_SPEED_MSG) !=  MODULE_SUCCESS ){
    printf("\n\rFailed to Initialize Nucleo board \n\r"); 
    return 0; 
  }   
  else
    printf("\n\rNucleo board initialized."); 

  printf("\n\r **TMP***. Build configuration:\n\r");
  printf("CONFIG_USE_NFC %d \n\r",CONFIG_USE_NFC);
  
  HAL_Delay(1000);
  
  printf("\n\r**************************************************************************\n\r");
  printf("***                HTTP Client for STM32 Nucleo                 ***\n\r");
  printf("**************************************************************************\n\r");

#if CONFIG_USE_NFC > 0  
  /* Init NFC expansion board */
  if (InitNFC () < 0 ){  
    printf("\n\rFailed to Initialize NFC board."); 
    return 0; 
  }   
  else
    printf("\n\rNFC expansion board initialized.");   
#else
    printf("\n\rNFC expansion board not used.");   
#endif  
    
  HAL_Delay(1000);

#if CONFIG_USE_MEMS > 0	
  /* Init sensors expansion board */
  if (InitSensors () < 0 ){  
    printf("\n\rFailed to Initialize Sensors board."); 
    return 0; 
  }   
  else
    printf("\n\rSensors board initialized.");   
#else 
    printf("\n\rNo sensor board. ");   
#endif  
	 
	
  HAL_Delay(1000);  
  
  /* Set WiFi AP parameters */  
  if ( ( ssid!=NULL ) && ( seckey!=NULL ) ){
    printf("\r\nUsing SSID and PWD written in the code. "); 
    
    memcpy(WIFITOKEN.NetworkSSID, ssid, strlen(ssid));
    memcpy(WIFITOKEN.NetworkKey, seckey, strlen(seckey));
    memcpy(WIFITOKEN.AuthenticationType, "WPA2", strlen("WPA2"));
  }
  else{
        printf("\r\nStarting configure procedure for SSID and PWD....  "); 
        HAL_Delay(3000);
          
        if (ConfigAPSettings() < 0 ){  
          printf("\n\rFailed to set AP settings."); 
          return 0; 
        }   
        else
          printf("\n\rAP settings set.");         
  }      
  
  /* WiFi Init                 */
  if (ConfigWiFi () < 0 ){  
    printf("\n\rFailed to Initialize WiFi."); 
    return 0; 
  }   
  else
    printf("\n\rWiFi initialized.");   

 

  /* Initialize WiFi module */
  wifi_state = wifi_state_idle;
  if(wifi_init(&config) != WiFi_MODULE_SUCCESS)
  {
    printf("\r\n[E].Error in wifi init \r\n");
    return 0;
  }    
  
  /*  Get MAC Address               */  
  if(Get_MAC_Add((char *)DisplayName) < 0){ 
         printf("\r\n[E]. Error while retrieving MAC address \r\n");  
         return 0; 
  }
  else{
    printf("\r\n [D]. WiFi MAC address: \r\n");
    printf((char *)DisplayName);
  }  

  
#if CONFIG_USE_NFC > 0  
  /* Write IBM quickstart URL to NFC */  
  if( WriteURLtoNFC((char *)url_ibm) < 0) { 
          printf("\r\n[E]. Error while writing URL to NFC \r\n");  
          return 0;    
  }
  else {
      printf("\r\n [D] IBM Quickstart URL (https://+)  \r\n");
      printf((char *)url_ibm); 
      printf("\r\n");
  }
#endif  
 
  if ( ConfigServer(&test_server) < 0 ){  
    printf("\n\rFailed to Config server "); 
    return 0; 
  }   
  else
    printf("\n\rServer configured."); 
	
  while (1)
  {
		 if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET) {
			 printf("\n\rApplication exit.\n\n"); 
			 //wifi_state = wifi_state_idle;
			 break;
		 }
		 
    switch (wifi_state) 
    {
    case wifi_state_reconnect:
          printf("\r\n [E] WiFi connection lost. Wait 10 sec then reconnect with parameters:  \r\n");
          printf((char *)WIFITOKEN.NetworkSSID);
          printf("\r\n");
          printf((char *)WIFITOKEN.NetworkKey);
          printf("\r\n");
          HAL_Delay(10000);

          status_spwf = wifi_connect(WIFITOKEN.NetworkSSID, WIFITOKEN.NetworkKey, mode);

          if(status_spwf != WiFi_MODULE_SUCCESS){
             printf("\r\n[E].Error cannot connect with WiFi \r\n");
             wifi_state = wifi_state_reconnect;
          } else{
               printf("\r\n [D] Reconnecting....  \r\n");
               printf((char *)WIFITOKEN.NetworkSSID);
               wifi_state = wifi_state_idle;
          }
          break;

     case wifi_state_reset:
        break;
		 
		 case wifi_state_connected:
        // Low power mode not used
        break;
      
     case wifi_state_ready: 
       HAL_Delay(20);
        status_spwf = wifi_network_scan(net_scan, WIFI_SCAN_BUFFER_LIST);
        if(status_spwf==WiFi_MODULE_SUCCESS)
        {   
          
          if (strcmp(WIFITOKEN.AuthenticationType, "NONE") == 0)
               mode = None;
          else if (strcmp(WIFITOKEN.AuthenticationType, "WEP") == 0)
               mode = WEP;
          else
               mode = WPA_Personal;
   
          for ( i=0; i<WIFI_SCAN_BUFFER_LIST; i++ )
            {
                if(( (char *) strstr((const char *)net_scan[i].ssid,(const char *)WIFITOKEN.NetworkSSID)) !=NULL) {    
                   SSID_found = WIFI_TRUE;
                   memcpy(WIFITOKEN.AuthenticationType, "WPA2", strlen("WPA2"));                                      
                   status_spwf = wifi_connect(WIFITOKEN.NetworkSSID, WIFITOKEN.NetworkKey, mode);
                   
                   if(status_spwf!=WiFi_MODULE_SUCCESS) {
                      printf("\r\n[E].Error cannot connect to WiFi network \r\n");
                      return 0;
                   }else{
                      printf("\r\n [D] Connected to network with SSID  \r\n");
                      printf((char *)WIFITOKEN.NetworkSSID);  
                   }
                   break;
                 }
            }
          
            if( (!SSID_found) ) 
            {
                  /* Can happen in crowdy environments */ 
                   printf("\r\n[E]. Error, given SSID not found! Trying to force connection with SSID: \r\n");
                   printf((char *)WIFITOKEN.NetworkSSID); 
                   status_spwf = wifi_connect(WIFITOKEN.NetworkSSID, WIFITOKEN.NetworkKey, mode);
                   if(status_spwf!=WiFi_MODULE_SUCCESS){
                      printf("\r\n[E].Error cannot connect with WiFi \r\n");
                      return 0;
                   } else{
                        printf("\r\n [D] Connected to network with SSID  \r\n");
                        printf((char *)WIFITOKEN.NetworkSSID);  
                   }
            }       
            memset(net_scan, 0x00, sizeof(net_scan));
        }
        else
            printf("\r\n[E]. Error, network AP not found! \r\n");
  
        wifi_state = wifi_state_idle; 
        break;

     
    case wifi_state_socket_create:
			printf("\r\n[D]. Open socket \r\n");  
			status_spwf = wifi_socket_client_open(test_server.hostname, test_server.port_number,  &test_server.protocol, &test_server.socket_id);
		  if(status_spwf==WiFi_MODULE_SUCCESS){
				 printf("\r\n[D]. Socket opened. Go to post \r\n");   
				 HAL_Delay(2000);
				 wifi_state= wifi_state_http_post;
			}
			else {
				 printf("\r\n[E].wifi_state_socket_create line %d \r\n", __LINE__ );
			   wifi_state= wifi_state_idle;
				 
			}
			break;

		   
     case wifi_state_http_get:		
			 	 printf("\r\n[D]. HTTP get \r\n");  
			   status_http = write_http_get(&test_server);	 
         if (status_http < 0)   
				 {		
					 printf("\r\n[E].wifi_state_http_get line %d \r\n", __LINE__ );
				   wifi_state= wifi_state_idle;
				 } 
				 else {
						 HAL_Delay(2000);
						 
						 printf("\r\n[D]. Read HTTP response \r\n");  
						 status_http = read_http_response();	
						 if (status_http < 0){     
							 printf("\r\n[E].wifi_state_http_get line %d \r\n", __LINE__ );
						 }		
						 wifi_state= wifi_state_http_post;
						 HAL_Delay(15000);
					 }		 
		     break;
		 
     case wifi_state_http_post:		
			   printf("\r\n[D]. HTTP post \r\n");  
			   status_http = write_http_post(&test_server);		      
         if (status_http < 0) {     
					 printf("\r\n[E].wifi_state_http_post line %d \r\n", __LINE__ );
					 wifi_state= wifi_state_idle;
				 }
				 else {
							 printf("\r\n[D]. Read HTTP response \r\n");  
							 status_http = read_http_response();		 
							 if (status_http < 0)     {
								 printf("\r\n[E].wifi_state_http_post line %d \r\n", __LINE__ );
								 wifi_state= wifi_state_idle;	
							 }

							 wifi_state= wifi_state_http_post;	 
							 HAL_Delay(10000);
				 }			 
         break;
 
   
    case wifi_state_idle:
        printf(".");
        HAL_Delay(500);
        break;  
        
    case wifi_state_disconnected:        
        NotifyLEDOff();
        break;
        
    default:
      break;
    }     
  }  
}

#if CONFIG_USE_NFC > 0  

/**
  * @brief  Read Access Point parameters from NFC
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ReadWifiTokenFromNFC(void) {
  System_Status_t status = MODULE_ERROR;
  uint32_t tickstart;

  tickstart = user_currentTimeGetTick();
  while ( (user_currentTimeGetTick() - tickstart ) < APP_TIMEOUT)  {
    if(TT4_ReadWifiToken(&WIFITOKEN) == SUCCESS)
      status = MODULE_SUCCESS;
  }
      
  return status;
}


/**
  * @brief  Write URL to NFC
  * @param  char* UrlForNFC: string containing URL to write in NFC
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t WriteURLtoNFC(char* UrlForNFC)
{
  sURI_Info URI;
  System_Status_t status = MODULE_ERROR;
  
   /* Write URI */
   strcpy(URI.protocol,URI_ID_0x03_STRING);   
   strcpy(URI.URI_Message,(char *)UrlForNFC);
   strcpy(URI.Information,"\0");
              
   if(TT4_WriteURI(&URI) == SUCCESS) {
          printf("Written URL in NFC\r\n");
          printf((char *)UrlForNFC);
          printf("\r\n");
          status = MODULE_SUCCESS;
   } 
 
  return status; 
}
#endif

/**
  * @brief  Save Access Point parameters to FLASH
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t SaveSSIDPasswordToMemory(void)
{
  System_Status_t status = MODULE_ERROR;

  /* Reset Before The data in Memory */
  status = ResetSSIDPasswordInMemory();

  if(status>=0) {
    /* Store in Flash Memory */
    uint32_t Address = BLUEMSYS_FLASH_ADD;
    int32_t WriteIndex;

   /* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();
   
   /* Write the Magic Number */
   {
     uint32_t MagicNumber = WIFI_CHECK_SSID_KEY;
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



/**
  * @brief  Erase Access Point parameters from FLASH
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ResetSSIDPasswordInMemory(void)
{
  /* Reset Calibration Values in FLASH */
  System_Status_t status = MODULE_ERROR;

  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = BLUEMSYS_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      printf("\n\rError while erasing FLASH memory");
  } else
      status = MODULE_SUCCESS;
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return status;
}


/**
  * @brief  Read Access Point parameters from FLASH
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ReCallSSIDPasswordFromMemory(void)
{
  System_Status_t status = MODULE_ERROR;
  
  uint32_t Address = BLUEMSYS_FLASH_ADD;
  __IO uint32_t data32 = *(__IO uint32_t*) Address;
  if(data32== WIFI_CHECK_SSID_KEY){
    int32_t ReadIndex;

    for(ReadIndex=0;ReadIndex<UNION_DATA_SIZE;ReadIndex++){
      Address +=4;
      data32 = *(__IO uint32_t*) Address;
      UnionWifiToken.Data[ReadIndex]=data32;
    }
    status = MODULE_SUCCESS;
  }
  else
    printf("\r\nFLASH Keyword not found.");
  
  return status;
}

/**
  * @brief  Read Access Point parameters from serial interface
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t wifi_get_AP_settings(void)
{
  System_Status_t status = MODULE_ERROR;
  uint8_t console_input[1], console_count=0;
  char console_ssid[NDEF_WIFI];
  char console_psk[NDEF_WIFI];
  WiFi_Priv_Mode mode; 
    
  
              printf("\r\nEnter the SSID:");
              fflush(stdout);

              console_count=0;
              console_count=scanf("%s",console_ssid);
              printf("\r\n");
              // FIXME : Why 39. NDEF is 32
              if(console_count==NDEF_WIFI) 
              {
                        printf("Exceeded number of ssid characters permitted");
                        return status;
               }    

              printf("Enter the password:");
              fflush(stdout);
              console_count=0;
              
              console_count=scanf("%s",console_psk);
              printf("\r\n");
              // FIXME : Why 19. NDEF is 32
              if(console_count==NDEF_WIFI) 
                    {
                        printf("Exceeded number of psk characters permitted");
                        return status;
                    }    
              printf("Enter the authentication mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal):"); 
              fflush(stdout);
              scanf("%s",console_input);
              printf("\r\n");
              //printf("entered =%s\r\n",console_input);
              switch(console_input[0])
              {
                case '0':
                  mode = None;
                  break;
                case '1':
                  mode = WEP;
                  break;
                case '2':
                  mode = WPA_Personal;
                  break;
                default:
                  printf("\r\nWrong Entry. Priv Mode is not compatible\n");
                  return status;              
              }
              
              memcpy(WIFITOKEN.NetworkSSID, console_ssid, strlen(console_ssid));
              memcpy(WIFITOKEN.NetworkKey, console_psk, strlen(console_psk));
              if (mode == None)
                memcpy(WIFITOKEN.AuthenticationType, "NONE", strlen("NONE"));
              else if (mode == WEP)
                memcpy(WIFITOKEN.AuthenticationType, "WEP", strlen("WEP"));
              else
                memcpy(WIFITOKEN.AuthenticationType, "WPA2", strlen("WPA2"));
              
              status = MODULE_SUCCESS;
              
              return status;
}


/**
  * @brief  Initialize Nucleo board
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitNucleo(int i_usart_speed)
{
   System_Status_t status = MODULE_ERROR;
  
   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* Configure the system clock */
   SystemClock_Config();

   /* configure the timers  */
   Timer_Config();

   UART_Configuration(115200);
   UART_Msg_Gpio_Init();
   USART_PRINT_MSG_Configuration(i_usart_speed);

   BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
   BSP_LED_Init(LED2);  // <hd>
   
  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    printf("\n\r I2C Init Error \n\r");
    return status;
  }
  else
    status = MODULE_SUCCESS;
   
   return status;
 }

#if CONFIG_USE_NFC > 0  
/**
  * @brief  Initialize NFC board
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitNFC(void)
{
   System_Status_t status = MODULE_ERROR;
  
    /* Initialize the M24SR component */
  while (TT4_Init() != SUCCESS){
    ;
  }
  status = MODULE_SUCCESS;
  return status;
}
#endif



static System_Status_t ConfigServer(http_endpoint *server)
{
   System_Status_t status = MODULE_ERROR;
   memset(server->hostname,0,strlen(TEST_SERVER_NAME)+1);
	 strcpy(server->hostname, TEST_SERVER_NAME);
	 
	 server->port_number = TEST_SERVER_PORT;
	
	 if (server->port_number == 443) 
		 server->protocol = 's';
	 else
     server->protocol = 't';
	 
	server->socket_id = 0;
	 
	status = MODULE_SUCCESS; 
	return status; 
}


/**
  * @brief  POST request
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t write_http_get (http_endpoint *server)
{
	System_Status_t status = MODULE_ERROR;
	int     ret;
	char    buf[HTTP_BUFFER_SIZE];
	// Add HTTP GET 
	if ((ret = snprintf(buf, sizeof(buf), "%s %s%s HTTP/1.1\r\n", "GET", "/api/tokens/",DisplayName)) < 0 || ret >= sizeof(buf)){
              return status;
         }
  // write line
	if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}
	// Add Hostname					 
	if ((ret = snprintf(buf, sizeof(buf), "Host: %s\r\n", server->hostname )) < 0 || ret >= sizeof(buf)){
              return status;
   }
  // write line
	if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}
  // write end of line line
	if	(wifi_socket_client_write(server->socket_id, strlen(HTTP_LINE_END), HTTP_LINE_END) != WiFi_MODULE_SUCCESS){
				return status;
	}
				
				 
				 
  status = MODULE_SUCCESS;
  return status;
}	


/**
  * @brief  POST request
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t write_http_post (http_endpoint *server)
{
	System_Status_t status = MODULE_ERROR;
	int     ret;
	char    buf[HTTP_BUFFER_SIZE];
	char    postContent[HTTP_POST_CONTENT_SIZE];


	
	//if ( Get_UUID_Nucleo((char *) UUID_Nucleo) == MODULE_ERROR ) {
  //         printf("\r\n[E].Failed o retrieve Nucleo UUID. \r\n");
	//				 return MODULE_ERROR;
  //    }    
  // add content
	
	prepare_json_pkt (postContent);

  printf ("Post content: \r\n");
  printf ("%s \r\n", postContent);

	
  //if ((ret = snprintf(postContent, sizeof(postContent), "\"%s+%s\"", UUID_Nucleo, DisplayName)) < 0 || ret >= sizeof(postContent)){
  //            return status;
  //      }			
	// add post
	// https://iotportal-148413.appspot.com/_ah/api/data/v1/save			
	if ((ret = snprintf(buf, sizeof(buf), "%s %s HTTP/1.1\r\n", "POST", "/_ah/api/data/v1/save")) < 0 || ret >= sizeof(buf)){
              return status;
         }
	if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}
	// Add Hostname					 
	if ((ret = snprintf(buf, sizeof(buf), "Host: %s\r\n", server->hostname )) < 0 || ret >= sizeof(buf)){
              return status;
   }
	if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}	
  // Add content lenght 
  if ((ret = snprintf(buf, sizeof(buf), "Content-Length: %d\r\n", strlen(postContent))) < 0 || ret >= sizeof(buf)){
                  return status;
       }	
  if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}	
  // add content-type
  if ((ret = snprintf(buf, sizeof(buf), "Content-Type: application/json\r\n")) < 0 || ret >= sizeof(buf)){
              return status;
        }				
  if	(wifi_socket_client_write(server->socket_id, strlen(buf), buf) != WiFi_MODULE_SUCCESS){
				return status;
	}	
	// write end of line for header
	if	(wifi_socket_client_write(server->socket_id, strlen(HTTP_LINE_END), HTTP_LINE_END) != WiFi_MODULE_SUCCESS){
				return status;
	}
	// write content
	//
	 printf ("Strlen Post content: \r\n");
   printf ("%d \r\n", strlen(postContent));

	
  if	(wifi_socket_client_write(server->socket_id, strlen(postContent), postContent) != WiFi_MODULE_SUCCESS){
				return status;
	}	
	// write end of line for content
	if	(wifi_socket_client_write(server->socket_id, strlen(HTTP_LINE_END), HTTP_LINE_END) != WiFi_MODULE_SUCCESS){
				return status;
	}
	

			 
  status = MODULE_SUCCESS;
  return status;
}	


static System_Status_t read_http_response(void)
{
	System_Status_t status = MODULE_ERROR;
	unsigned char data_chunk[APPLICATION_BUFFER_SIZE];
	int i_data_read = 0;

	printf ("Waiting HTTP response. \r\n");
	

	while (i_data_read == 0){
		  HAL_Delay(500);
		  printf (".");
			i_data_read = wifi_socket_read(data_chunk, APPLICATION_BUFFER_SIZE);	
		}

		printf ("Check received response \r\n");
  
		
		if ( (strstr(data_chunk, "500 Internal Server Error") != 0) || 
			(strstr(data_chunk, "501 Not Implemented") != 0) ||
		  (strstr(data_chunk, "502 Bad Gateway") != 0) ||
		  (strstr(data_chunk, "503 Service Unavailable") != 0) ||
		  (strstr(data_chunk, "504 Gateway Time-out") != 0) ||
		  (strstr(data_chunk, "505 HTTP Version Not Supported") != 0) 
		){
					printf ("\r\n Failed request \r\n");
			    return status;
		}
		
		// Custom for Innext;  "result": true,
		if ( (strstr(data_chunk, "\"result\": true") != 0) ){
							printf ("\r\n Response OK \r\n");
		}	else {
					printf ("\r\n Wrong response \r\n");
			    return status;			
		}
	 			
  status = MODULE_SUCCESS;
  return status;
}	


static System_Status_t Get_UUID_Nucleo(char *UUID_Nucleo)
{
    uint32_t idPart1 = STM32_UUID[0];
    uint32_t idPart2 = STM32_UUID[1];
    uint32_t idPart3 = STM32_UUID[2];
    
    sprintf (UUID_Nucleo,"%.10d%.10d%.10d",idPart1,idPart2,idPart3);

	
    return MODULE_SUCCESS;
}


#if CONFIG_USE_MEMS > 0	
/**
  * @brief  Initialize sensors board
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitSensors(void)
{
   System_Status_t status = MODULE_ERROR;
 
      /* Try to use LSM6DS3 DIL24 if present */
      if(BSP_ACCELERO_Init( LSM6DS3_X_0, &ACCELERO_handle )!=COMPONENT_OK){        
        /* otherwise try to use LSM6DS0 on board */
        if(BSP_ACCELERO_Init( LSM6DS0_X_0, &ACCELERO_handle )!=COMPONENT_OK){
          return status;
        }
      }

      /* Try to use LSM6DS3 if present, otherwise use LSM6DS0 */
      if(BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle )!=COMPONENT_OK){
          return status;
      }

      /* Force to use LIS3MDL */
      if(BSP_MAGNETO_Init( LIS3MDL_0, &MAGNETO_handle )!=COMPONENT_OK){
          return status;
      }

      /* Force to use HTS221 */
      if(BSP_HUMIDITY_Init( HTS221_H_0, &HUMIDITY_handle )!=COMPONENT_OK){
          return status;
      }

      /* Force to use HTS221 */
      if(BSP_TEMPERATURE_Init( HTS221_T_0, &TEMPERATURE_handle )!=COMPONENT_OK){
          return status;
      }

      /* Try to use LPS25HB DIL24 if present, otherwise use LPS25HB on board */
      if(BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle )!=COMPONENT_OK){
          return status;
      }
      
      /*  Enable all the sensors */
      BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
      BSP_GYRO_Sensor_Enable( GYRO_handle );
      BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
      BSP_HUMIDITY_Sensor_Enable( HUMIDITY_handle );
      BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
      BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );

  status = MODULE_SUCCESS;
  return status;      
    
}
#endif




#if CONFIG_USE_MEMS > 0	
 /**
  * @brief  Prepare JSON packet with sensors data
  * @param  buffer : buffer that will contain sensor data in JSON format 
  * @retval None
  */
 void prepare_json_pkt (uint8_t * buffer)
{
	    // 737
      int32_t intPart, decPart;
      char tempbuff[40];
      float PRESSURE_Value;
      float HUMIDITY_Value;
      float TEMPERATURE_Value;
      SensorAxes_t ACC_Value;
      SensorAxes_t GYR_Value;
      SensorAxes_t MAG_Value;
    

      strcpy((char *)buffer,"{\"deviceHttpKey\":\"__HTTP_KEY__\",");
      strcat((char *)buffer,"\"deviceHttpSecret\":\"__HTTP_SECRET__\",");
			strcat((char *)buffer,"\"data\":[{\"dataType\":\"temp\",\"value\":");
      BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle,(float *)&TEMPERATURE_Value);
      MCR_BLUEMS_F2I_2D(TEMPERATURE_Value, intPart, decPart);
      sprintf(tempbuff, "\"%lu.%lu\"",intPart, decPart);
      strcat((char *)buffer,tempbuff);

      strcat((char *)buffer,"},{");
			strcat((char *)buffer,"\"dataType\":\"hum\",\"value\":");
      BSP_HUMIDITY_Get_Hum(HUMIDITY_handle,(float *)&HUMIDITY_Value);
      MCR_BLUEMS_F2I_2D(HUMIDITY_Value, intPart, decPart);
      sprintf(tempbuff, "\"%lu.%lu\"",intPart, decPart);
      strcat(  (char *)buffer,tempbuff);
      
      //BSP_PRESSURE_Get_Press(PRESSURE_handle,(float *)&PRESSURE_Value);
      //MCR_BLUEMS_F2I_2D(PRESSURE_Value, intPart, decPart);
      //sprintf(tempbuff, ",\"A_Pressure\":%lu.%lu",intPart, decPart );
      //strcat((char *)buffer,tempbuff);
      
      strcat((char *)buffer,"},{");
			strcat((char *)buffer,"\"dataType\":\"accx\",\"value\":");			
      BSP_ACCELERO_Get_Axes(ACCELERO_handle,&ACC_Value);
      //sprintf(tempbuff, ",\"Acc_X\":%d",ACC_Value.AXIS_X);
			sprintf(tempbuff, "\"%d\"",ACC_Value.AXIS_X);
      strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"Acc_Y\":%d",ACC_Value.AXIS_Y);
      //strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"Acc_Z\":%d",ACC_Value.AXIS_Z);
      //strcat((char *)buffer,tempbuff);
      strcat((char *)buffer,"},{");
			strcat((char *)buffer,"\"dataType\":\"accy\",\"value\":");			
      //BSP_ACCELERO_Get_Axes(ACCELERO_handle,&ACC_Value);
      //sprintf(tempbuff, ",\"Acc_X\":%d",ACC_Value.AXIS_X);
			sprintf(tempbuff, "\"%d\"",ACC_Value.AXIS_Y);
      strcat((char *)buffer,tempbuff);
      
      //BSP_GYRO_Get_Axes(GYRO_handle,&GYR_Value);
      //sprintf(tempbuff, ",\"GYR_X\":%d",GYR_Value.AXIS_X);
      //strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"GYR_Y\":%d",GYR_Value.AXIS_Y);
      //strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"GYR_Z\":%d",GYR_Value.AXIS_Z);
      //strcat((char *)buffer,tempbuff);
     
      //BSP_MAGNETO_Get_Axes(MAGNETO_handle,&MAG_Value);
      //sprintf(tempbuff, ",\"MAG_X\":%d",MAG_Value.AXIS_X);
      //strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"MAG_Y\":%d",MAG_Value.AXIS_Y);
      //strcat((char *)buffer,tempbuff);
      //sprintf(tempbuff, ",\"MAG_Z\":%d",MAG_Value.AXIS_Z);
      //strcat((char *)buffer,tempbuff);
      
      //strcat((char *)buffer,"}}");
      strcat((char *)buffer,"}]}");
      
      return;
}
#endif



/**
  * @brief  GET MAC Address from WiFi
  * @param  char* macadd : string containing MAC address
  * @retval None
  */
static System_Status_t Get_MAC_Add (char *macadd)
{
  uint8_t macaddstart[32];
  System_Status_t status = MODULE_ERROR; // 0 success
  int i,j;
       
  if (GET_Configuration_Value("nv_wifi_macaddr",(uint32_t *)macaddstart) != WiFi_MODULE_SUCCESS)
  {
      printf("Error retrieving MAC address \r\n");  
      return status; 
  }
  else
      status = MODULE_SUCCESS;
 
  macaddstart[17]='\0';
  printf("MAC orig: \r\n");  
  printf((char *)macaddstart);
  printf("\r\n");
  
  if (status == MODULE_SUCCESS)
  {  
        for(i=1,j=0;i<17;i++){
          if(macaddstart[i]!=':'){
            macadd[j]=macaddstart[i];
            j++;  
          } 
        }
        macadd[j]='\0';
  }
  
  return status;
}


/**
  * @brief  Configure Access Point parmaters (SSID, PWD, Authenticaation) :
  * @brief  1) Read from FLASH
  * @brief  2) If not available in FLASH or when User Button is pressed : read from NFC
  * @brief  3) If not available in FLASH or when User Button is pressed and if nothing is written in NFC : read from serial terminal
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ConfigAPSettings(void)
{
   System_Status_t status = MODULE_ERROR;
   bool   b_set_AP_pars = false;
#if CONFIG_USE_NFC > 0  
   uint8_t URL_ST[64];
#endif

   printf("\r\nKeep pressed user button to set Wi-Fi Access Point parameters (SSID and PWD) "); 
   printf("\r\nfrom NFC or via serial terminal. Otherwise parameters saved to FLASH will be used.");
   HAL_Delay(4000);
   
   /* User Button Pressed --> set parameters */
   if(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET) {
          /* Read from FLASH */
          if(ReCallSSIDPasswordFromMemory() >= 0) {
              printf("\n\rRead from FLASH:\n\r");
              printf("\tSSID =%s\n\r",WIFITOKEN.NetworkSSID);
              printf("\tKey  =%s\n\r",WIFITOKEN.NetworkKey);
              printf("\tAuthentication  =%s\n\r",WIFITOKEN.AuthenticationType);

              status = MODULE_SUCCESS;
          }
          else
          { 
              printf("\n\rNo data written in FLASH.");
              b_set_AP_pars = true;
          }   
    }  
    else
       b_set_AP_pars = true;

    if (b_set_AP_pars)
    {
            // Blink LED
            /*
            printf("\r\nLED ON....\n\r");
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
            BSP_LED_On(LED2);
            HAL_Delay(3000);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            BSP_LED_Off(LED2);
            */
     #if CONFIG_USE_NFC > 0  
            printf("\r\nReading from NFC....\n\r");
            /* Read from FLASH. 5 sec timeout */
            status = ReadWifiTokenFromNFC();
     #else
            status = MODULE_ERROR;
     #endif       
      
            if(status==MODULE_SUCCESS)
            {
     #if CONFIG_USE_NFC > 0  
                printf("Read from NFC:\n\r");
                printf("\tSSID =%s\n\r",WIFITOKEN.NetworkSSID);
                printf("\tKey  =%s\n\r",WIFITOKEN.NetworkKey);
                printf("\tAuthentication  =%s\n\r",WIFITOKEN.AuthenticationType);
                                
                /* Prevent to read wrong SSID/PWD from NFC next time */
                strcpy((char *)URL_ST, "www.st.com/stm32ode");
                status = WriteURLtoNFC((char *)URL_ST);  
                if(status!=MODULE_SUCCESS)
                { 
                      printf("\r\n[E]. Error while writing URL to NFC \r\n");  
                      return status;    
                }
      #else
                printf("\r\n[E]. Error in ConfigAPSettings. \r\n");
      #endif    
                       
            }     
            else {
              printf("No Wi-Fi parameters from NFC. \n\r"); 
              printf("\n\rRead parameters from serial terminal."); 

              /* Read from Serial.  */
              status = wifi_get_AP_settings();        
              if(status!=MODULE_SUCCESS)
              {
                    printf("\r\n[E]. Error in AP Settings\r\n");
                    return status;
              }
              
                          
              /* Prevent to read wrong SSID/PWD from NFC next time */
           #if CONFIG_USE_NFC > 0 
              strcpy((char *)URL_ST, "www.st.com/stm32ode");
              status = WriteURLtoNFC((char *)URL_ST);  
              if(status!=MODULE_SUCCESS)
              { 
                      printf("\r\n[E]. Error while writing URL to NFC \r\n");  
                      return status;    
              }         
            #endif  
            }
            /* Save to FLASH */  
             status = SaveSSIDPasswordToMemory();
             if(status!=MODULE_SUCCESS)
             {
                    printf("\r\n[E]. Error in AP Settings\r\n");
                    return status;
             }

        }
   
  return status;
}




/**
  * @brief  Initialize WiFi board
  * @param  None
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ConfigWiFi(void)
{
  System_Status_t status = MODULE_ERROR;

  /* Config WiFi : disable low power mode */
  config.power=wifi_active;
  config.power_level=high;
  config.dhcp=on;//use DHCP IP address
  config.web_server=WIFI_TRUE;  
  
  status = MODULE_SUCCESS;
  return status;
}  
  

/*
** 			WiFi Callbacks
**
*/



/**
  * @brief  Wi-Fi callback activated when Wi-Fi is on 
  * @param  None  
  * @retval None
  */
void ind_wifi_on()
{
  printf("\r\n[D]. Wi-Fi on \r\n");
  wifi_state = wifi_state_ready;
}

/**
  * @brief  Wi-Fi callback activated when Wi-Fi is connected to AP 
  * @param  None  
  * @retval None
  */
void ind_wifi_connected()
{
  wifi_state = wifi_state_socket_create;
}

/**
  * @brief  Wi-Fi callback activated Wi-Fi is resuming from sleep mode 
  * @param  None  
  * @retval None
  */
void ind_wifi_resuming()
{
  printf("\r\n [E]. Wifi resuming from sleep user callback... \r\n");
	wifi_state = wifi_state_ready;
}

/**
  * @brief  Wi-Fi callback activated in case of connection error
  * @param  None  
  * @retval None
  */
void ind_wifi_connection_error(WiFi_Status_t WiFi_DE_AUTH)
{
  printf("\r\n [E]. WiFi connection error. Go to idle. \r\n");

  wifi_state = wifi_state_idle;
}

/**
  * @brief  Wi-Fi callback activated when remote server is closed  
  * @param  socket_closed_id : socket identifier  
  * @retval None
  */
void ind_wifi_socket_client_remote_server_closed(uint8_t * socket_closed_id)
{
   printf("\r\n[E]. Remote disconnection from server.  \r\n");

	 wifi_state = wifi_state_reconnect;
}

/**
  * @brief  Wifi callback for data received  
  * @param  data_ptr : pointer to buffer to be filled with data
  *         message_size : message size (equal to chunk_size if less than APPLICATION_BUFFER_SIZE bytes) 
  *         chunk_size : chunk size (used in case message size is larger than APPLICATION_BUFFER_SIZE)
  * @retval None
  */
void ind_wifi_socket_data_received(uint8_t socket_id,uint8_t * data_ptr, uint32_t message_size, uint32_t chunck_size)
{

//	printf("\r\n[D] ind_wifi_socket_data_received. %s \r\n", data_ptr );

	
  if ( message_size > APPLICATION_BUFFER_SIZE )
 {
     printf("\r\n[E] Error ind_wifi_socket_data_received. Exceeded APPLICATION_BUFFER_SIZE \r\n");
     return;
 }

 if ( message_size == chunck_size){
    memcpy(receive_data, data_ptr, chunck_size);       
 }
 else
 {     
     memcpy(&receive_data[application_idx], data_ptr, chunck_size);
     application_idx += chunck_size;
    
     if (application_idx == message_size){              
       application_idx = 0;      
     }
     else
       return;
  }

  LocalBufferPushBuffer(&localBufferReading,(char *)&receive_data,message_size);
}



/**
  * @brief  Wrapping function to read data from an buffer queue, which is filled by WiFi callbacks 
  * @param  net : Network structure 
  *         i : buffer to fill with data read
  *         ch : number of bytes to read
  *         timeout : timeout for writing to socket 
  * @retval sizeReceived : number of bytes read
  */
int wifi_socket_read (unsigned char* buff, int ch){
    
  int sizeReceived;
 
  sizeReceived = LocalBufferGetSizeBuffer(&localBufferReading);
  
  if(sizeReceived > 0) 
  {    
    if(sizeReceived >= ch)
    {
      LocalBufferPopBuffer(&localBufferReading, buff, ch);
      return ch;
    }
    else
    {  
      LocalBufferPopBuffer(&localBufferReading, buff, sizeReceived);
      return sizeReceived;
    }     
  }
  return 0;
 }




/**
* @brief  Retrieve SysTick to increment counter
* @param  None
* @retval tick value
*/
uint32_t user_currentTimeGetTick()
{
   return HAL_GetTick();
}  


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI)
 *            SYSCLK(Hz)                     = 84000000
 *            HCLK(Hz)                       = 84000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 16000000
 *            PLL_M                          = 16
 *            PLL_N                          = 336
 *            PLL_P                          = 4
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale2 mode
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 

     clocked below the maximum system frequency, to update the voltage scaling value 

     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
   
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}



 /**
  * @brief  Turn on notificaion LED (LED2)
  * @param  None
  * @retval None
  */
static void NotifyLEDOn(void)
{
 BSP_LED_On(LED2);
}

 /**
  * @brief  Turn off notificaion LED (LED2)
  * @param  None
  * @retval None
  */
static void NotifyLEDOff(void)
{
 BSP_LED_Off(LED2);
}

 /**
  * @brief  Turn on notificaion LED (LED2)
  * @param  msPeriod time delay in milli seconds
  * @retval None
  */
static void NotifyLEDBlink(unsigned int msPeriod) 
{
   BSP_LED_Off(LED2);
   HAL_Delay(msPeriod);
   BSP_LED_On(LED2);
}




/**
 * @}
 */
 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
