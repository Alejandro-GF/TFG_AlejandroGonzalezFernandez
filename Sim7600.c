/*
 * Sim7600.c
 *
 *  Created on: 3 mar. 2021
 *      Author: gf_al
 */
#include "Sim7600.h"

extern UART_HandleTypeDef huart2; //UART handler para la SIM en cuestión
extern char depurado_txt[2000]; //Cadena de depurado_txt a guardar en la SD



uint8_t Sim7600_ComandoRaw(uint8_t* comando, uint8_t* respuesta){

	HAL_StatusTypeDef ret;

	ret = HAL_UART_Transmit(&huart2, comando, strlen(comando), SIM7600_TIMEOUT_COMANDO);

	if(ret == HAL_OK); //Nada
	else if(ret == HAL_BUSY)
		return TRANSMIT_BUSY;
	else if(ret == HAL_ERROR)
		return TRANSMIT_ERROR;
	else if(ret == HAL_TIMEOUT)
		return TRANSMIT_TIMEOUT;


	if (respuesta != NULL) //Solo hacer un Receive si el buffer es distinto a NULL
		ret=HAL_UART_Receive(&huart2, respuesta, RESPONSE_SIZE, SIM7600_TIMEOUT_RESPONSE);

	return COMMAND_SENT;
}
uint8_t Sim7600_ComandoConResp(uint8_t* comando, uint8_t* resp_esperada, uint8_t* resp_recibida){

	uint8_t* needle; // Variable que indica si se ha encontrado la "aguja en el pajar"
	uint8_t  i   	        = 0;
	uint8_t  j				= 0;
	uint8_t  resp_temp[255] = {0}; //Se usa un resp_temp porque resp_recibida puede ser NULL y la comprobación de la respuesta no se haría bien
	int		 status         = 0;

	do
	{
		status = Sim7600_ComandoRaw(comando, resp_temp);
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

uint8_t Sim7600_Comprobacion(){

	uint8_t status;

	status = Sim7600_ComandoConResp ("AT\r\n", "OK", NULL);  //Si se desea ver la respuesta, cambiar NULL
																   //por una variable temporal
	if (status != EXPECTED_RESPONSE)
	{
		return BAD_COMM;
	}

	status = Sim7600_ComandoConResp ("AT+CPIN?\r\n", "READY", NULL);

	if (status != EXPECTED_RESPONSE)
	{
		return BAD_PIN;
	}

	return COMPROB_OK;

}


uint8_t Sim7600_Reset(){

	uint8_t status;

	//status = Sim7600_ComandoConResp ("AT+CFUN=1,1\r\n", "OK", NULL);
	status=EXPECTED_RESPONSE;
	if (status != EXPECTED_RESPONSE)
	{
		return BAD_RESET;
	}
	return RESET_SUCCESS;
}
uint8_t CSQ(uint8_t* res){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+CSQ\r\n", "OK", res);

	if (status != EXPECTED_RESPONSE)
	{
		return WRONG_RESPONSE;
	}

	return EXPECTED_RESPONSE;

}
uint8_t Sim7600_SleepModeIn(){

	uint8_t status;

	/*status = Sim7600_ComandoConResp("AT+CSCLK=1\r\n", "OK", NULL); //Habilitar reloj lento
	if(status != EXPECTED_RESPONSE)
		return SLOW_CLOCK_EN_ERROR;
*/
	status = Sim7600_ComandoConResp("AT+CFUN=0\r\n", "OK", NULL); //Configurar a funcionalidad mínima
	if(status != EXPECTED_RESPONSE)
		return BAD_SLEEP;

	return SLEEP_SUCCESS;

}
uint8_t Sim7600_Wake_Up(){
	char test[100]= {0};
	uint8_t status;

	status = Sim7600_ComandoConResp("AT+CFUN=1\r\n", "OK", test); //Configurar a funcionalidad total
	if(status != EXPECTED_RESPONSE)
		return BAD_WAKE_UP;

	/*status = Sim7600_ComandoConResp("AT+CSCLK=0\r\n", "OK", test); //Deshabilitar reloj lento
	if(status != EXPECTED_RESPONSE)
		return SLOW_CLOCK_DIS_ERROR;
*/


	return WAKE_UP_SUCCESS;
}

uint8_t Sim7600_PS_status(){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+CGREG?\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
				return PS_STATUS_ERROR;

	return PS_STATUS_SUCCESS;
}

uint8_t Sim7600_Network_Bearing(){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+CGACT?\r\n", "+CGACT: 1,1", NULL);
	if(status != EXPECTED_RESPONSE)
				return NETWORK_BEARING_ERROR;

	return NETWORK_BEARING_SUCCESS;
}

uint8_t Sim7600_operadora(uint8_t* respuesta){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+COPS?\r\n", "+COPS: 0,0", respuesta); //Obtener la operadora conectada
		if(status != EXPECTED_RESPONSE)
			return OPERATOR_CONNECTED_ERROR;

		return OPERATOR_CONNECTED_SUCCESS;
}
uint8_t Sim7600_Init_HTTP(uint8_t tipo_http,uint8_t* url){

	uint8_t status;
	char 	HTTPURL[100] = {0};

	e_sprintf(HTTPURL, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
	status = Sim7600_ComandoConResp("AT+HTTPINIT\r\n","OK",NULL);
	if(status != EXPECTED_RESPONSE)
		{Sim7600_ComandoConResp("AT+HTTPTERM\r\n","OK", NULL);
		status = Sim7600_ComandoConResp("AT+HTTPINIT\r\n","OK", NULL);}
	if(status != EXPECTED_RESPONSE)
		return HTTP_SERVICE_INIT_ERROR;
	switch(tipo_http)
	{

	case DATOS:

		status = Sim7600_ComandoConResp(HTTPURL, "OK", NULL);
		//status = Sim7600_ComandoConResp("AT+HTTPPARA=\"URL\",\"www.riverview.es/clicnfish/sensors/5a4e61f9614687085658c171/addmeasurement\"\r\n", "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_URL_ERROR;

		status = Sim7600_ComandoConResp("AT+HTTPPARA=\"CONTENT\",\"application/json; boundary=abc123abc\"\r\n", "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_CONTENT_ERROR;

		break;


	case FOTO:

		status = Sim7600_ComandoConResp(HTTPURL, "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_URL_ERROR;

		status = Sim7600_ComandoConResp("AT+HTTPPARA=\"CONTENT\",\"multipart/form-data; boundary=abc123abc\"\r\n", "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_CONTENT_ERROR;
		status = Sim7600_ComandoConResp("AT+HTTPPARA=\"UA\",\"SIMCOM_MODULE\"\r\n", "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_USER_AGENT_ERROR;

		status = Sim7600_ComandoConResp("AT+HTTPPARA=\"ACCEPT\",\"*/*\"\r\n", "OK", NULL);
		if(status != EXPECTED_RESPONSE)
			return SET_ACCEPT_ERROR;



	}
	return HTTP_INIT_SUCCESS;

}
uint8_t Sim7600_Term_HTTP(){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+HTTPTERM\r\n", "OK", NULL); // TERMINAR conexión HTTP
	if(status != EXPECTED_RESPONSE)
		status = Sim7600_ComandoConResp("AT+HTTPTERM\r\n", "OK", NULL);
	if(status != EXPECTED_RESPONSE)
		return TERM_HTTP_ERROR;

	return TERM_HTTP_SUCCESS;

}
uint8_t Sim7600_Data_Post(char* data, int data_length){

	uint8_t status;
	char 	HTTPData[100] = {0};
	char test[100]={0};
	e_sprintf(HTTPData, "AT+HTTPDATA=%d,%d\r\n", data_length, SIM7600_DATA_TIMEOUT);

	status = Sim7600_ComandoConResp(HTTPData, "DOWNLOAD", NULL); //Indicarle a la SIM cuántos datos debe enviar
	if(status != EXPECTED_RESPONSE)
		return HTTP_DATA_ERROR;

	Sim7600_ComandoRaw(data, NULL); // Enviar los datos
	HAL_Delay(10000);
	status = Sim7600_ComandoConResp("AT+HTTPACTION=1\r\n", "OK", test);// POST request (0: GET, 1: POST, 2: HEAD)
	if(status != EXPECTED_RESPONSE)
		return POST_ERROR;

	return DATA_POST_SUCCESS;
}
uint8_t Sim7600_Resp_Servidor(uint8_t* Server_Values){

	uint8_t status;
	uint8_t temp[255] = {0};

	 status = Sim7600_ComandoConResp("AT+HTTPREAD=0,46\r\n", "OK", temp);
	if(status != EXPECTED_RESPONSE)
		return BAD_SERVER_RESPONSE;

	Sim7600_AT_Response_Parser(temp, "+HTTPREAD:", Server_Values, '\n'); //Solo llamar al parser de las respuestas AT si el comando ha tenido éxito

	return SERVER_RESPONSE_SUCCESS;
}


/*
 * @brief Realizar el POST request con la imagen tomada
 * @param filename: nombre del archivo que se creará para enviar al servidor
 *        stringImg: puntero con el mapa de bits de la foto tomada
 *        sizeImg: tamaño de la imagen
 * @retv  status: código de estado de la operación (ver el .h)
 */
uint8_t Sim7600_PostImagen(char* filename, uint8_t* stringImg, int sizeImg){

	char    HTTPData[255] = {0};
	char    HTTPPost[255] = {0};
	char    temp[255]	  = {0};
	int     command_size  = 0;
	int     t 			  = 0;
	int i=0;
	int     resto		  = 0;
	uint8_t status        = 0;
	uint8_t *p            = stringImg;
	uint8_t temp2[255]    = {0};
	char	test[200]= {0};
	char Postcam[255]	= {0};
	command_size  = myStrlen("--abc123abc\r\nContent-Type: image/jpeg\r\n\r\n\r\n\r\n--abc123abc--");

	e_sprintf(temp, "Content-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\n",filename);
	command_size += myStrlen(temp);
	command_size += sizeImg;


	/*e_sprintf(HTTPData, "AT+HTTPDATA=%d,%d\r\n", command_size, 60000);

	status = Sim7600_ComandoConResp(HTTPData, "DOWNLOAD", test); //AT+HTTPDATA
	if(status != EXPECTED_RESPONSE)
		return HTTP_DATA_ERROR;
*/
	e_sprintf(HTTPData, "AT+CFTRANRX=\"f:/%s\",%d\r\n",filename ,command_size);
	status = Sim7600_ComandoConResp(HTTPData, ">", NULL);
	if(status != EXPECTED_RESPONSE)
		return CFTRANRX_DATA_ERROR;
	//Envío de la imagen siguiendo la forma: Boundary, Headers, Body, Boundary (sacado directamente de Sim7600.c)
	status = Sim7600_ComandoRaw("--abc123abc\r\n", NULL); //Boundary
	if(status != COMMAND_SENT)
		return status;

	status = Sim7600_ComandoRaw(temp, NULL); //Header 1
	if(status != COMMAND_SENT)
		return status;

	status = Sim7600_ComandoRaw("Content-Type: image/jpeg\r\n\r\n", NULL); //Header 2
	if(status != COMMAND_SENT)
		return status;

	//Body (envío de foto en pedazos de 60kbi
	if (sizeImg >= 60000) //Porqué es 60k? No lo sé aún (Diego)
	{
		for (t = 0; t < sizeImg; t += 60000)
		{
			resto = sizeImg - t;
			if (resto <= 60000) break;
			HAL_UART_Transmit(&huart2, p,60000,SIM7600_TIMEOUT_INTERNET);	//Transimisión directa de la imagen (el timeout es distinto)
			p+=60000;
		}
			HAL_UART_Transmit(&huart2, p,resto,SIM7600_TIMEOUT_INTERNET);
	}
	else
	{
		HAL_UART_Transmit(&huart2, p,sizeImg,SIM7600_TIMEOUT_INTERNET);
	}
	//Boundary
	status = Sim7600_ComandoRaw("r\n\r\n--abc123abc--", NULL);
	if(status != COMMAND_SENT)
		return status;
	Delay(3000);
	e_sprintf(HTTPPost,"AT+HTTPPOSTFILE=\"%s\",1,1,1\r\n",filename);
	status = Sim7600_ComandoConResp(HTTPPost, "OK", NULL); // POST request (0: GET, 1: POST, 2: HEAD)
	if(status != EXPECTED_RESPONSE)
		return POST_ERROR;
	return IMAGE_POST_SUCCESS;
}



/*
* @brief Analizar respuesta AT y extraer la información relevante, que viene separada por comas
* @param respuesta: respuesta AT a analizar
* 		  header: cabecera AT que indica el tipo de respuesta (+HTTPREAD:, +CCLK:, etc)
* 		  buffer: buffer donde se va a guardar los datos extraídos en formato uint8_t (entero)
* 		  erase_until: Caracter anterior al inicio de la respuesta relevante
* @retv  NONE
* Sim7600_AT_Response_Parser(temp, "+HTTPREAD:", Server_Values, '\n')
*/
void Sim7600_AT_Response_Parser(uint8_t* respuesta, uint8_t* header, uint8_t* buffer, uint8_t erase_until){

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
uint8_t Sim7600_HoraNTP(){

	uint8_t status;

	status = Sim7600_ComandoConResp("AT+CNTP\r\n", "OK", NULL); //Iniciar NTP
	if(status != EXPECTED_RESPONSE)
		return NTP_INIT_ERROR;

	return NTP_SUCCESS;

}

uint8_t Sim7600_HoraInterna(uint8_t* buffer){

	uint8_t status;
	uint8_t temp[50] = {0};

	status = Sim7600_ComandoConResp("AT+CCLK?\r\n", "+CCLK:", temp); //Obtener la hora local
	if(status != EXPECTED_RESPONSE)
		return INTERNAL_TIME_ERROR;

	strncpy(buffer, temp + 19, 17); //Extraer la fecha y hora de la respuesta AT

	return INTERNAL_TIME_SUCCESS;
}

uint8_t Sim7600_GPS(uint8_t* buffer){

	uint8_t status;
	uint8_t temp[50] = {0};

	status = Sim7600_ComandoConResp("AT+CGPS=1,1\r\n", "OK:", NULL); //Empezar GPS
	status = Sim7600_ComandoConResp("AT+CGPSINFO\r\n", "+CGPSINFO:", temp);//Obtiene la locaclizacion del modulo
	if(status != EXPECTED_RESPONSE)
			return GPS_INFO_ERROR;
	strncpy(buffer, temp + 11, 35); //Extraer la fecha y hora de la respuesta AT

	return GPS_SUCCESS;
}
uint8_t Sim7600_UpdateCSQ(uint8_t* CSQ_Values){

	uint8_t status = 0;
	uint8_t temp[50] = {0};

	status = Sim7600_ComandoConResp("AT+CSQ\r\n", "+CSQ:", temp); //Obtener RSSI y BER
	if(status != EXPECTED_RESPONSE)
		return CSQ_UPDATE_ERROR;

	Sim7600_AT_Response_Parser(temp, "+CSQ:", CSQ_Values, ' '); //Extraer valores CSQ solo si ha habido éxito en la respuesta AT

	return CSQ_UPDATE_SUCCESS;
}
uint8_t Sim7600_Module_Info(){
	uint8_t status = 0;
	char	temp[200]= {0};
	status = Sim7600_ComandoConResp("AT+CGMM\r\n", "SIMCOM_SIM7600G", NULL);
	if(status != EXPECTED_RESPONSE)
		{
		status = Sim7600_ComandoConResp("AT+CGMM\r\n", "SIMCOM_SIM7070", NULL);
		if(status != EXPECTED_RESPONSE)
			return MODULE_INFO_ERROR;
		return SIMCOM7070;
		}
	return SIMCOM7600;
}


void Sim7600_Status_Handler(uint8_t status){

	char msg[200] = {0};

	switch(status)
	{

	/*-------------------------Errores-----------------------------*/
	case TRANSMIT_TIMEOUT:
		//e_sprintf(msg, "Sim7600: TRANSMIT_TIMEOUT Timeout al transmitir al USART\n\n");
		break;

	case TRANSMIT_BUSY:
		//e_sprintf(msg, "Sim7600: TRANSMIT_BUSY Canal ocupado al transmitir al USART\n\n");
		break;

	case TRANSMIT_ERROR:
		//e_sprintf(msg, "Sim7600: TRANSMIT_ERROR Error al transmitir al USART\n\n");
		break;

	case WRONG_RESPONSE:
		//e_sprintf(msg, "Sim7600: WRONG_RESPONSE Excedido numero de envios sin respuesta esperada\n\n");
		break;

	case BAD_COMM:
		e_sprintf(msg, "[Sim7600.c] BAD_COMM: Comunicacion fallida con SIM7600\n");
		break;

	case BAD_RESET:
		e_sprintf(msg, "[Sim7600.c] BAD_RESET: Reset fallido de la SIM7600\n");
		break;

	case BAD_PIN:
		e_sprintf(msg, "[Sim7600.c] BAD_PIN: La SIM7600 espera introduccion de un PIN\n");
		break;

	case PS_STATUS_ERROR:
		e_sprintf(msg, "[Sim7600.c] PS_STATUS_ERROR: Error en el estado del PS\n");
		break;

	case NETWORK_BEARING_ERROR:
		e_sprintf(msg, "[Sim7600.c] NETWORK_BEARING_ERROR: Error al comprobar el bearer\n");
		break;

	case PASSWORD_ERROR:
		e_sprintf(msg, "[Sim7600.c] PASSWORD_ERROR: Error al poner el password\n");
		break;

	case BAD_CONNECTION_SETTING:
		e_sprintf(msg, "[Sim7600.c] BAD_CONNECTION_SETTING: La configuracion de la conexion no se realizo correctamente\n");
		break;

	case UNKNOWN_OPERATOR:
		e_sprintf(msg, "[Sim7600.c] UNKNOWN_OPERATOR: La operadora ingresada no es correcta\n");
		break;

	case HTTP_SERVICE_INIT_ERROR:
		e_sprintf(msg, "[Sim7600.c] HTTP_SERVICE_INIT_ERROR: Error al inicializar servicio HTTP\n");
		break;

	case SET_URL_ERROR:
		e_sprintf(msg, "[Sim7600.c] SET_URL_ERROR: Error al poner el URL del servidor\n");
		break;

	case SET_CONTENT_ERROR:
		e_sprintf(msg, "[Sim7600.c] SET_CONTENT_ERROR: Error al poner el parametro CONTENT de HTTP\n");
		break;
	case SET_USER_AGENT_ERROR:
			e_sprintf(msg, "[Sim7600.c] SET_USER_AGENT_ERROR: Error al poner el parametro CONTENT de HTTP\n");
			break;
	case SET_ACCEPT_ERROR:
			e_sprintf(msg, "[Sim7600.c] SET_ACCEPT_ERROR: Error al poner el parametro CONTENT de HTTP\n");
			break;
	case HTTP_DATA_ERROR:
			e_sprintf(msg, "[Sim7600.c] HTTP_DATA_ERROR: Error al preparar los datos para su envio al servidor\n");
			break;
	case CFTRANRX_DATA_ERROR:
		e_sprintf(msg, "[Sim7600.c] CFTRANRX_DATA_ERROR: Error al preparar los datos para su envio al servidor\n");
		break;

	case POST_ERROR:
		e_sprintf(msg, "[Sim7600.c] POST_ERROR: Error al hacer un POST request\n");
		break;

	case BAD_SERVER_RESPONSE:
		e_sprintf(msg, "[Sim7600.c] BAD_SERVER_RESPONSE: Respuesta no valida del servidor\n");
		break;

	case TERM_HTTP_ERROR:
		e_sprintf(msg, "[Sim7600.c] TERM_HTTP_ERROR: Error al terminar el servicio HTTP\n");
		break;


	case NTP_INIT_ERROR:
		e_sprintf(msg, "[Sim7600.c] NTP_INIT_ERROR: Error al iniciar NTP\n");
		break;

	case SLOW_CLOCK_EN_ERROR:
		e_sprintf(msg, "[Sim7600.c] SLOW_CLOCK_EN_ERROR: Error al habilitar reloj en modo lento\n"
				       "[Sim7600.c] WARNING: Puede que el modulo SIM no este bien conectado. Envio al servidor deshabilitado\n\n");
		break;

	case BAD_SLEEP:
		e_sprintf(msg, "[Sim7600.c] BAD_SLEEP: Modo SLEEP de la Sim7600 fallido\n");
		break;

	case SLOW_CLOCK_DIS_ERROR:
		e_sprintf(msg, "[Sim7600.c] SLOW_CLOCK_DIS_ERROR: Error al deshabilitar reloj en modo lento\n");
		break;

	case BAD_WAKE_UP:
		e_sprintf(msg, "[Sim7600.c] BAD_WAKE_UP: Despertar de la Sim7600 fallido\n");
		break;

	case INTERNAL_TIME_ERROR:
		e_sprintf(msg, "[Sim7600.c] INTERNAL_TIME_ERROR: Error al obtener hora local\n");
		break;

	case OPERATOR_CONNECTED_ERROR:
		e_sprintf(msg, "[Sim7600.c] OPERATOR_CONNECTED_ERROR: Error al comprobar la conexion a la operadora\n");
		break;

	case CSQ_UPDATE_ERROR:
		e_sprintf(msg, "[Sim7600.c] CSQ_UPDATE_ERROR: Error al obtener datos CSQ\n");
		break;


	/*-------------------------Estados buenos-----------------------------*/
	case RESET_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Reset completado con exito\n");
		break;

	case COMPROB_OK:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Comprobacion correcta de la SIM7600\n");
		break;

	case CONNECTION_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Conexion a la operadora realizada con exito\n");
		break;

	case PS_STATUS_SUCCESS:
			e_sprintf(msg, "[Sim7600.c] SUCCESS: Conexion al PS realizada con exito\n");
			break;

	case NETWORK_BEARING_SUCCESS:
			e_sprintf(msg, "[Sim7600.c] SUCCESS: Conexion al bearer con exito\n");
			break;

	case HTTP_INIT_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Servicio HTTP inicializado con exito\n");
		break;

	case DATA_POST_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Datos enviados al servidor con exito\n");
		break;

	case SERVER_RESPONSE_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: El servidor ha enviado una respuesta con exito\n");
		break;

	case TERM_HTTP_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Servicio HTTP terminado con exito\n");
		break;

	case EXPECTED_RESPONSE:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Respuesta correcta al comando enviado\n");
		break;

	case NTP_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Protocolo NTP iniciado con exito\n");
		break;

	case SLEEP_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Modulo puesto a dormir con exito\n");
		break;

	case WAKE_UP_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: SIM7600 despertada con exito\n");
		break;

	case INTERNAL_TIME_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Hora local obtenida con exito\n");
		break;

	case IMAGE_POST_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Imagen enviada con exito\n");
		break;

	case OPERATOR_CONNECTED_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Operadora conectada comprobada con exito\n");
		break;

	case CSQ_UPDATE_SUCCESS:
		e_sprintf(msg, "[Sim7600.c] SUCCESS: Datos CSQ obtenidos con exito\n");
		break;

	default:
		e_sprintf(msg, "[Sim7600.c] FATAL: Unknown result code!\n");

	} //switch

	printf(msg);
	//SD_print(depurado_txt, msg);

}

