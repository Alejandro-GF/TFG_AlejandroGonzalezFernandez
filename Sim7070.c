/*
 * Sim7070.c
 *
 *  Created on: 3 mar. 2021
 *      Author: gf_al
 */
#include "Sim7070.h"

extern UART_HandleTypeDef huart2; //UART handler para la SIM en cuestión
extern char depurado_txt[2000]; //Cadena de depurado_txt a guardar en la SD



uint8_t Sim7070_ComandoRaw(uint8_t* comando, uint8_t* respuesta){

	HAL_StatusTypeDef ret;

	ret = HAL_UART_Transmit(&huart2, comando, strlen(comando), SIM7070_TIMEOUT_COMANDO);

	if(ret == HAL_OK); //Nada
	else if(ret == HAL_BUSY)
		return TRANSMIT_BUSY;
	else if(ret == HAL_ERROR)
		return TRANSMIT_ERROR;
	else if(ret == HAL_TIMEOUT)
		return TRANSMIT_TIMEOUT;


	if (respuesta != NULL) //Solo hacer un Receive si el buffer es distinto a NULL
		HAL_UART_Receive(&huart2, respuesta, RESPONSE_SIZE, SIM7070_TIMEOUT_RESPONSE);

	return COMMAND_SENT;
}
uint8_t Sim7070_ComandoConResp(uint8_t* comando, uint8_t* resp_esperada, uint8_t* resp_recibida){

	uint8_t* needle; // Variable que indica si se ha encontrado la "aguja en el pajar"
	uint8_t  i   	        = 0;
	uint8_t  j				= 0;
	uint8_t  resp_temp[255] = {0}; //Se usa un resp_temp porque resp_recibida puede ser NULL y la comprobación de la respuesta no se haría bien
	int		 status         = 0;

	do
	{
		status = Sim7070_ComandoRaw(comando, resp_temp);
		needle = strstr(resp_temp, resp_esperada); // Buscar resp_esperada(aguja) en temp(pajar)
		i++;
	}
	while (needle == NULL && i < RESPONSE_ATTEMPTS);

	//Copiar buffer temporal en buffer de salida solo si este último existe
	if(resp_recibida != NULL)
	{
		for (j = 0; j < 255 && resp_temp[j] != '\0' ; j++)
		{
			resp_recibida[j] = resp_temp[j];
		}
	}
	if (status == TRANSMIT_TIMEOUT || status == TRANSMIT_BUSY || status == TRANSMIT_ERROR);
	   //Los errores de USART tienen prioridad

	else if (i >= RESPONSE_ATTEMPTS)
	{
		status = WRONG_RESPONSE;
	}
	else
		status = EXPECTED_RESPONSE;

	return status;
}

uint8_t Sim7070_Comprobacion(){

	uint8_t status;

	status = Sim7070_ComandoConResp ("AT\r\n", "OK", NULL);  //Si se desea ver la respuesta, cambiar NULL
																   //por una variable temporal
	if (status != EXPECTED_RESPONSE)
	{
		return BAD_COMM;
	}

	status = Sim7070_ComandoConResp ("AT+CPIN?\r\n", "READY", NULL);

	if (status != EXPECTED_RESPONSE)
	{
		return BAD_PIN;
	}

	return COMPROB_OK;

}


uint8_t Sim7070_Reset(){

	uint8_t status;

	status = Sim7070_ComandoConResp ("AT+CFUN=1,1\r\n", "OK", NULL);

	if (status != EXPECTED_RESPONSE)
	{
		return BAD_RESET;
	}
	return RESET_SUCCESS;
}

uint8_t Sim7070_SleepModeIn(){//comprobar

	uint8_t status;


	status = Sim7070_ComandoConResp("AT+CFUN=0\r\n", "OK", NULL); //Configurar a funcionalidad mínima
	if(status != EXPECTED_RESPONSE)
		return BAD_SLEEP;

	return SLEEP_SUCCESS;

}
uint8_t Sim7070_Connect(){

	uint8_t status;
	uint8_t temp[255] = {0};

	status = Sim7070_ComandoConResp("AT+CREG=2\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return NETWORK_REG_ERROR;
	status = Sim7070_ComandoConResp("AT+CEREG=2\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return EPS_NETWORK_REG_ERROR;
	status = Sim7070_ComandoConResp("AT+CGREG=2\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return GPRS_NETWORK_REG_ERROR;
	status = Sim7070_ComandoConResp("AT+CFUN=4\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
				return PARTIAL_FUNC_MODE_ERROR;
	HAL_Delay(1000);
	status = Sim7070_ComandoConResp("AT+CFUN=1\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return FULL_FUNC_MODE_ERROR;
	status = Sim7070_ComandoConResp("AT+CNMP=51\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return MODE_SELECTION_ERROR;
	//status = Sim7070_ComandoConResp("AT+COPS?\r\n", "+COPS: 0,0,\"Movistar\",9", temp);
	//if(status != EXPECTED_RESPONSE)
				//	return OPERATOR_CONNECTED_ERROR;
	status = Sim7070_ComandoConResp("AT+CPSI?\r\n", "+CPSI: LTE NB-IOT", temp);
	if(status != EXPECTED_RESPONSE)
		return SYSTEM_INFO_ERROR;

return CONNECTION_SUCCESS;
}



uint8_t Sim7070_Wake_Up(){//comprobar

	uint8_t status;


	status = Sim7070_ComandoConResp("AT+CFUN=1\r\n", "OK", NULL); //Configurar a funcionalidad total
	if(status != EXPECTED_RESPONSE)
		return BAD_WAKE_UP;

	return WAKE_UP_SUCCESS;
}

uint8_t Sim7070_PS_status(){

	uint8_t status;

	status = Sim7070_ComandoConResp("AT+CGREG?\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
				return PS_STATUS_ERROR;

	return PS_STATUS_SUCCESS;
}

uint8_t Sim7070_Network_Bearing(){

	uint8_t status;

	status = Sim7070_ComandoConResp("AT+CGACT?\r\n", "+CGACT: 1,1", NULL);
	if(status != EXPECTED_RESPONSE)
				return NETWORK_BEARING_ERROR;

	return NETWORK_BEARING_SUCCESS;
}

uint8_t Sim7070_operadora(uint8_t* respuesta){

	uint8_t status;

	status = Sim7070_ComandoConResp("AT+COPS?\r\n", "+COPS: 0,0,\"Movistar\",9", respuesta); //Obtener la operadora conectada
		if(status != EXPECTED_RESPONSE)
			return OPERATOR_CONNECTED_ERROR;

		return OPERATOR_CONNECTED_SUCCESS;
}
uint8_t Sim7070_Init_HTTP(uint8_t* url,char* data, int data_length){

	uint8_t status;
	char 	HTTPURL1[100] = {0};
	char 	HTTPURL2[100] = {0};
	char 	HTTPData1[100] = {0};
	char 	HTTPData2[100] = {0};
	char	texto[200]= {0};

	status = Sim7070_ComandoConResp("AT+CNACT=0,1\r\n", "OK", NULL);
	e_sprintf(HTTPURL1, "AT+SHCONF=\"URL\",\"%s\"\r\n", url);

	//status = Sim7070_ComandoConResp(HTTPURL1, "OK", NULL);
	status = Sim7070_ComandoConResp("AT+SHCONF=\"URL\",\"http://www.riverview.es\"\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
				return SET_URL_ERROR;
	e_sprintf(HTTPData1, "AT+SHCONF=\"BODYLEN\",%d\r\n", data_length);
	e_sprintf(HTTPData2, "AT+SHBOD=%d,%d\r\n", data_length,10000);
	e_sprintf(HTTPURL2,"AT+SHREQ=\"%s\",3\r\n", url);
	status = Sim7070_ComandoConResp(HTTPData1, "OK", NULL); //Indicarle a la SIM cuántos datos debe enviar
	if(status != EXPECTED_RESPONSE)
			return SET_HTTP_CONF_ERROR;

	status = Sim7070_ComandoConResp("AT+SHCONF=\"HEADERLEN\",350\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_HTTP_CONF_ERROR;
	status = Sim7070_ComandoConResp("AT+SHCONF?\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
		return SET_HTTP_CONF_ERROR;
	status = Sim7070_ComandoConResp("AT+SHCONN\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return HTTP_DATA_ERROR;
	status = Sim7070_ComandoConResp("AT+SHSTATE?\r\n", "1", NULL);
	if(status != EXPECTED_RESPONSE)
					return PS_STATUS_ERROR;
	status = Sim7070_ComandoConResp("AT+SHCHEAD\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return PS_STATUS_ERROR;
	status = Sim7070_ComandoConResp("AT+SHAHEAD=\"Accept\",\"text/html, */*\"\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_CONTENT_ERROR;
	status = Sim7070_ComandoConResp("AT+SHAHEAD=\"User-Agent\",\"IOE Client\"\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_CONTENT_ERROR;
	status = Sim7070_ComandoConResp("AT+SHAHEAD=\"Content-Type\",\"application/json\"\r\n", "OK", NULL);
	//status = Sim7070_ComandoConResp("AT+SHAHEAD=\"Content-Type\",\"application /x-www-form-urlencoded\"\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_CONTENT_ERROR;
	status = Sim7070_ComandoConResp("AT+SHAHEAD=\"Connection\",\"keep-alive\"\r\n" , "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_CONTENT_ERROR;
	status = Sim7070_ComandoConResp("AT+SHAHEAD=\"Cache-control\",\"no-cache\"\r\n" , "OK", NULL);
	if(status != EXPECTED_RESPONSE)
					return SET_CONTENT_ERROR;
	status = Sim7070_ComandoRaw(HTTPData2, NULL);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart2, data, strlen(data), SIM7070_TIMEOUT_COMANDO); // Enviar los datos
	HAL_Delay(2000);


	status = Sim7070_ComandoConResp("AT+SHREQ=\"/clicnfish/sensors/5c766b7600a7895d784a6efa/addmeasurement\",3\r\n", "OK", texto); // POST request
	//status = Sim7070_ComandoConResp(HTTPURL2, "OK", texto);

	if(status != EXPECTED_RESPONSE)
		return POST_ERROR;

		return DATA_POST_SUCCESS;

}
uint8_t Sim7070_Term_HTTP(){

	uint8_t status;
	status = Sim7070_ComandoConResp("AT+SHDISC\r\n", "OK", NULL); //
	if(status != EXPECTED_RESPONSE)
		return TERM_HTTP_ERROR;

	return TERM_HTTP_SUCCESS;

}

uint8_t Sim7070_Resp_Servidor(uint8_t* Server_Values){

	uint8_t status;
	char 	HTTPURL[100] = {0};
	uint8_t temp[255] = {0};
	uint8_t test[255] = {0};
	status = Sim7070_ComandoConResp("AT+SHREAD=0,46\r\n", "OK", temp);
	if(status != EXPECTED_RESPONSE)
		status= BAD_SERVER_RESPONSE;

	Sim7070_AT_Response_Parser(temp, "+SHREAD:", Server_Values, '\n'); //Solo llamar al parser de las respuestas AT si el comando ha tenido éxito

	return SERVER_RESPONSE_SUCCESS;
}


/*
 * @brief Realizar el POST request con la imagen tomada
 * @param filename: nombre del archivo que se creará para enviar al servidor
 *        stringImg: puntero con el mapa de bits de la foto tomada
 *        sizeImg: tamaño de la imagen
 * @retv  status: código de estado de la operación (ver el .h)
 */

/*
* @brief Analizar respuesta AT y extraer la información relevante, que viene separada por comas
* @param respuesta: respuesta AT a analizar
* 		  header: cabecera AT que indica el tipo de respuesta (+HTTPREAD:, +CCLK:, etc)
* 		  buffer: buffer donde se va a guardar los datos extraídos en formato uint8_t (entero)
* 		  erase_until: Caracter anterior al inicio de la respuesta relevante
* @retv  NONE
* Sim7070_AT_Response_Parser(temp, "+HTTPREAD:", Server_Values, '\n')
*/
void Sim7070_AT_Response_Parser(uint8_t* respuesta, uint8_t* header, uint8_t* buffer, uint8_t erase_until){

	uint8_t* needle;
	uint8_t  temp[10] = {0};
	uint8_t  i 	= 0;
	uint8_t  j  = 0;
	uint8_t  k  = 0;

	needle = strstr(respuesta, header); //Buscar el header en la respuesta

	while(*needle != erase_until) //Ir moviendo el puntero hasta que encuentre el caracter anterior al inicio de la cadena
		needle++;

	if(*needle == '\0')
		return;

	needle++; //Luego moverlo un paso más para empezar a guardar los datos

	for(; j < 255; j++)
	{
		while(*needle != ',' && *needle != '\r') //Hasta que se encuentre una coma o con el fin de línea
		{
			temp[i] = *needle; //Ir guardando los datos en la variable temp
			i++;
			needle++;
		}

		buffer[j] = atoi(temp); //Convertir la lectura en int y guardarla en el buffer

		if(*needle == '\r') //Si se ha llegado a final de línea salir del bucle for
			break;

		for(k = 0; k < 10; k++) //Limpiar la variable temp
		{
			temp[k] = '\0';
		}

		i = 0; //Volver al inicio de temp
		needle++; //Ir al elemento siguiente a la coma

	}

}
uint8_t Sim7070_HoraNTP(){

	uint8_t status;

	status = Sim7070_ComandoConResp("AT+CNTP\r\n", "OK", NULL); //Iniciar NTP
	if(status != EXPECTED_RESPONSE)
		return NTP_INIT_ERROR;

	return NTP_SUCCESS;

}

uint8_t Sim7070_HoraInterna(uint8_t* buffer){

	uint8_t status;
	uint8_t temp[50] = {0};

	status = Sim7070_ComandoConResp("AT+CCLK?\r\n", "+CCLK:", temp); //Obtener la hora local
	if(status != EXPECTED_RESPONSE)
		return INTERNAL_TIME_ERROR;

	strncpy(buffer, temp + 19, 17); //Extraer la fecha y hora de la respuesta AT

	return INTERNAL_TIME_SUCCESS;
}

uint8_t Sim7070_UpdateCSQ(uint8_t* CSQ_Values){

	uint8_t status = 0;
	uint8_t temp[50] = {0};

	status = Sim7070_ComandoConResp("AT+CSQ\r\n", "+CSQ:", temp); //Obtener RSSI y BER
	if(status != EXPECTED_RESPONSE)
		return CSQ_UPDATE_ERROR;

	Sim7070_AT_Response_Parser(temp, "+CSQ:", CSQ_Values, ' '); //Extraer valores CSQ solo si ha habido éxito en la respuesta AT

	return CSQ_UPDATE_SUCCESS;
}

void Sim7070_Status_Handler(uint8_t status){

	char msg[200] = {0};

	switch(status)
	{

	/*-------------------------Errores-----------------------------*/
	case TRANSMIT_TIMEOUT:
		//e_sprintf(msg, "Sim7070: TRANSMIT_TIMEOUT Timeout al transmitir al USART\n\n");
		break;

	case TRANSMIT_BUSY:
		//e_sprintf(msg, "Sim7070: TRANSMIT_BUSY Canal ocupado al transmitir al USART\n\n");
		break;

	case TRANSMIT_ERROR:
		//e_sprintf(msg, "Sim7070: TRANSMIT_ERROR Error al transmitir al USART\n\n");
		break;

	case WRONG_RESPONSE:
		//e_sprintf(msg, "Sim7070: WRONG_RESPONSE Excedido numero de envios sin respuesta esperada\n\n");
		break;

	case BAD_COMM:
		e_sprintf(msg, "[Sim7070.c] BAD_COMM: Comunicacion fallida con SIM7070\n");
		break;

	case BAD_RESET:
		e_sprintf(msg, "[Sim7070.c] BAD_RESET: Reset fallido de la SIM7070\n");
		break;

	case BAD_PIN:
		e_sprintf(msg, "[Sim7070.c] BAD_PIN: La SIM7070 espera introduccion de un PIN\n");
		break;

	case NETWORK_REG_ERROR:
		e_sprintf(msg, "[Sim7070.c] NETWORK_REG_ERROR: Error al registrarse a la red\n");
		break;

	case EPS_NETWORK_REG_ERROR:
		e_sprintf(msg, "[Sim7070.c] EPS_NETWORK_REG_ERROR: Error al registrarse a la red EPS\n");
		break;

	case GPRS_NETWORK_REG_ERROR:
		e_sprintf(msg, "[Sim7070.c] GPRS_NETWORK_REG_ERROR: Error al registrarse a la red GPRS\n");
		break;

	case BAD_CONNECTION_SETTING:
		e_sprintf(msg, "[Sim7070.c] BAD_CONNECTION_SETTING: La configuracion de la conexion no se realizo correctamente\n");
		break;

	case UNKNOWN_OPERATOR:
		e_sprintf(msg, "[Sim7070.c] UNKNOWN_OPERATOR: La operadora ingresada no es correcta\n");
		break;

	case HTTP_SERVICE_INIT_ERROR:
		e_sprintf(msg, "[Sim7070.c] HTTP_SERVICE_INIT_ERROR: Error al inicializar servicio HTTP\n");
		break;

	case SET_URL_ERROR:
		e_sprintf(msg, "[Sim7070.c] SET_URL_ERROR: Error al poner el URL del servidor\n");
		break;

	case SET_CONTENT_ERROR:
		e_sprintf(msg, "[Sim7070.c] SET_CONTENT_ERROR: Error al poner el parametro CONTENT de HTTP\n");
		break;
	case SET_HTTP_CONF_ERROR:
		e_sprintf(msg, "[Sim7070.c] SET_HTTP_CONF_ERROR: Error al configurar el mensaje HTTP\n");
		break;

	case HTTP_DATA_ERROR:
		e_sprintf(msg, "[Sim7070.c] HTTP_DATA_ERROR: Error al preparar los datos para su envio al servidor\n");
		break;

	case POST_ERROR:
		e_sprintf(msg, "[Sim7070.c] POST_ERROR: Error al hacer un POST request\n");
		break;

	case BAD_SERVER_RESPONSE:
		e_sprintf(msg, "[Sim7070.c] BAD_SERVER_RESPONSE: Respuesta no valida del servidor\n");
		break;

	case TERM_HTTP_ERROR:
		e_sprintf(msg, "[Sim7070.c] TERM_HTTP_ERROR: Error al terminar el servicio HTTP\n");
		break;


	case NTP_INIT_ERROR:
		e_sprintf(msg, "[Sim7070.c] NTP_INIT_ERROR: Error al iniciar NTP\n");
		break;

	case SLOW_CLOCK_EN_ERROR:
		e_sprintf(msg, "[Sim7070.c] SLOW_CLOCK_EN_ERROR: Error al habilitar reloj en modo lento\n"
				       "[Sim7070.c] WARNING: Puede que el modulo SIM no este bien conectado. Envio al servidor deshabilitado\n\n");
		break;

	case BAD_SLEEP:
		e_sprintf(msg, "[Sim7070.c] BAD_SLEEP: Modo SLEEP de la Sim7070 fallido\n");
		break;

	case SLOW_CLOCK_DIS_ERROR:
		e_sprintf(msg, "[Sim7070.c] SLOW_CLOCK_DIS_ERROR: Error al deshabilitar reloj en modo lento\n");
		break;

	case BAD_WAKE_UP:
		e_sprintf(msg, "[Sim7070.c] BAD_WAKE_UP: Despertar de la Sim7070 fallido\n");
		break;

	case INTERNAL_TIME_ERROR:
		e_sprintf(msg, "[Sim7070.c] INTERNAL_TIME_ERROR: Error al obtener hora local\n");
		break;

	case OPERATOR_CONNECTED_ERROR:
		e_sprintf(msg, "[Sim7070.c] OPERATOR_CONNECTED_ERROR: Error al comprobar la conexion a la operadora\n");
		break;

	case PARTIAL_FUNC_MODE_ERROR:
		e_sprintf(msg, "[Sim7070.c] PARTIAL_FUNC_MODE_ERROR: Error al cambiar a modo de funcionamiento parcial\n");
		break;
	case FULL_FUNC_MODE_ERROR:
		e_sprintf(msg, "[Sim7070.c] FULL_FUNC_MODE_ERROR: Error al cambiar a modo de funcionamiento completo\n");
		break;
	case MODE_SELECTION_ERROR:
		e_sprintf(msg, "[Sim7070.c] MODE_SELECTION_ERROR: Error al cambiar el modo de red\n");
		break;
	case SYSTEM_INFO_ERROR:
		e_sprintf(msg, "[Sim7070.c] SYSTEM_INFO_ERROR: Error en la información del sistema recibida\n");
		break;
	case CSQ_UPDATE_ERROR:
		e_sprintf(msg, "[Sim7070.c] CSQ_UPDATE_ERROR: Error al obtener datos CSQ\n");
		break;



	/*-------------------------Estados buenos-----------------------------*/
	case RESET_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Reset completado con exito\n");
		break;

	case COMPROB_OK:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Comprobacion correcta de la SIM7070\n");
		break;

	case CONNECTION_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Conexion a la operadora realizada con exito\n");
		break;

	case HTTP_INIT_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Servicio HTTP inicializado con exito\n");
		break;

	case DATA_POST_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Datos enviados al servidor con exito\n");
		break;

	case SERVER_RESPONSE_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: El servidor ha enviado una respuesta con exito\n");
		break;

	case TERM_HTTP_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Servicio HTTP terminado con exito\n");
		break;

	case EXPECTED_RESPONSE:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Respuesta correcta al comando enviado\n");
		break;

	case NTP_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Protocolo NTP iniciado con exito\n");
		break;

	case SLEEP_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: SIM7070 puesta a dormir con exito\n");
		break;

	case WAKE_UP_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: SIM7070 despertada con exito\n");
		break;

	case INTERNAL_TIME_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Hora local obtenida con exito\n");
		break;

	case IMAGE_POST_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Imagen enviada con exito\n");
		break;

	case OPERATOR_CONNECTED_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Operadora conectada comprobada con exito\n");
		break;

	case CSQ_UPDATE_SUCCESS:
		e_sprintf(msg, "[Sim7070.c] SUCCESS: Datos CSQ obtenidos con exito\n");
		break;

	default:
		e_sprintf(msg, "[Sim7070.c] FATAL: Unknown result code!\n");

	} //switch

	printf(msg);
	//SD_print(depurado_txt, msg);

}

