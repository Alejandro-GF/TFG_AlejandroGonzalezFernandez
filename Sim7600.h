/*
 * Sim7600.h
 *
 *  Created on: 3 mar. 2021
 *      Author: gf_al
 */

#include "stm32f4xx_hal.h"
//#include <stdio.h>
#include "printf.h"
#include "string.h"
#include <stdlib.h>

#define SIM7600_TIMEOUT_COMANDO  9000
#define SIM7600_TIMEOUT_INTERNET 150000
#define SIM7600_TIMEOUT_RESPONSE 1000     //Tiempo máximo de espera para HAL_UART_Recieve //TODO Intentar cambiar a HAL_UART_RECIEVE_IT
#define SIM7600_DATA_TIMEOUT 	10000	 //Tiempo que va a esperar la Sim7600 para recibir los datos que se le indique
#define RESPONSE_SIZE		    255
#define RESPONSE_ATTEMPTS       10		//Número de veces que se va a enviar el mismo comando para recibir una respuesta


/*----------------- ESTADOS DE USART -----------------*/ //Macros para el estado resultante del HAL
#define TRANSMIT_TIMEOUT		1
#define TRANSMIT_BUSY			2
#define TRANSMIT_ERROR			3

#define WRONG_RESPONSE          10
#define COMMAND_SENT			11
#define EXPECTED_RESPONSE       12

#define BAD_COMM     			13		//La comunicación con el módulo Sim7600.c no es buena
#define BAD_RESET				14		//No se ha podido completar el reset
#define BAD_PIN					15		//La Sim7600 espera un PIN
#define APN_ERROR				16
#define USER_ERROR				17
#define PASSWORD_ERROR			18
#define NETWORK_BEARING_ERROR	19
#define BAD_CONNECTION_SETTING  20
#define UNKNOWN_OPERATOR		21
#define HTTP_SERVICE_INIT_ERROR 22
#define SET_URL_ERROR			23
#define SET_CONTENT_ERROR		24
#define SET_USER_AGENT_ERROR	25
#define SET_ACCEPT_ERROR		39
#define HTTP_DATA_ERROR			26
#define CFTRANRX_DATA_ERROR		40
#define POST_ERROR				27
#define BAD_SERVER_RESPONSE		28      //O no hubo respuesta o hubo un error
#define TERM_HTTP_ERROR			29
#define PS_STATUS_ERROR			30
#define	NTP_INIT_ERROR			31
#define SLOW_CLOCK_EN_ERROR		32
#define BAD_SLEEP   			33
#define SLOW_CLOCK_DIS_ERROR	34
#define BAD_WAKE_UP             35
#define INTERNAL_TIME_ERROR		36
#define OPERATOR_CONNECTED_ERROR 37
#define CSQ_UPDATE_ERROR		38
#define MODULE_INFO_ERROR		39
#define GPS_INFO_ERROR			40


#define PS_STATUS_SUCCESS		49
#define RESET_SUCCESS			50      //Se completó bien el reset
#define COMPROB_OK				51		//La comprobación inicial es correcta
#define CONNECTION_SUCCESS		52
#define NETWORK_BEARING_SUCCESS 53
#define HTTP_INIT_SUCCESS		54
#define DATA_POST_SUCCESS		55
#define SERVER_RESPONSE_SUCCESS 56
#define TERM_HTTP_SUCCESS		57
#define NTP_SUCCESS				58
#define SLEEP_SUCCESS		    59
#define WAKE_UP_SUCCESS			60
#define INTERNAL_TIME_SUCCESS   61
#define IMAGE_POST_SUCCESS		62
#define OPERATOR_CONNECTED_SUCCESS 63
#define CSQ_UPDATE_SUCCESS      64
#define SIMCOM7070     			 65
#define SIMCOM7600     			 66
#define GPS_SUCCESS				67


/*----------------- TIPO POST -----------------*/ //Macros para definir el tipo de POST
#define DATOS					201
#define FOTO					202








/*----------------- PROTOTIPOS -----------------*/

uint8_t Sim7600_ComandoRaw	   (uint8_t* comando, uint8_t* respuesta);
uint8_t Sim7600_ComandoConResp  (uint8_t* comando, uint8_t* resp_esperada, uint8_t* resp_recibida);
uint8_t Sim7600_Comprobacion    ();
uint8_t Sim7600_Reset		   ();
uint8_t Sim7600_SleepModeIn	   ();
uint8_t Sim7600_Wake_Up   	   ();
uint8_t Sim7600_Init_Conexion   (uint8_t operadora);
uint8_t Sim7600_Init_HTTP(uint8_t tipo_http,uint8_t* url);
uint8_t Sim7600_Term_HTTP	   ();
uint8_t Sim7600_Data_Post 	   (char* data, int data_length);
uint8_t Sim7600_Resp_Servidor   (uint8_t* respuesta);
uint8_t Sim7600_HoraNTP		   ();
uint8_t Sim7600_HoraInterna     (uint8_t* buffer);
uint8_t Sim7600_UpdateCSQ	   (uint8_t* buffer);
uint8_t Sim7600_GPS			(uint8_t* buffer);


//BORRADOR
uint8_t Sim7600_PostImagen(char* filename, uint8_t* stringImg, int sizeImg);
uint8_t Sim7600_operadora(uint8_t* respuesta);

void    Sim7600_AT_Response_Parser(uint8_t* respuesta, uint8_t* header, uint8_t* buffer, uint8_t erase_until);
void    Sim7600_Status_Handler    (uint8_t status);
