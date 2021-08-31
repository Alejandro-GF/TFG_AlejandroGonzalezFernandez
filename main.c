/**
  ******************************************************************************
  * @file    main.c
  * @author  AQUACORP S.L.
  * @version V1.1.2
  * @date    18-Enero-2020
  * @brief   Fichero principal de funcionamiento de las estaciones de medición de calidad de agua de AQUACORP
  ******************************************************************************
  * @attention
  *
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

/* Includes: Libraries included in this code*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "conf.h"

#include "Utilities/leds.h"
#include "Utilities/dram.h"
#include "Utilities/stm324x9i_eval_camera.h"
#include "Utilities/stm32f429i_discovery_sdram.h"
#include "Utilities/ov5642.h"
#include "Utilities/sdcard.h"
#include "Utilities/Sim7600.h"
#include "Utilities/Sim7070.h"
#include "Utilities/clock.h"
#include "Utilities/DS18B20.h"
#include "Utilities/DS18B20b.h"
#include "Utilities/DS2764.h"

#include "stdio.h"
#include "stdlib.h"

#include "Utilities/printf.h"
#include "Utilities/mlp115a1.h"

#include "Utilities/PCF8523.h"
#include "Utilities/eeprom.h"
#include "string.h"
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DCMI_HandleTypeDef hdcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

WWDG_HandleTypeDef hwwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// Variables de RTC
PCF8523_date fecha;
PCF8523_time hora;
uint8_t update_ext_rtc = 0;



// Variables de imagen
#define CAMERA_FRAME_BUFFER (__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR) //Lugar de memoria donde se guarda la foto (DRAM externa)

char filename [15]  ; //Nombre de la imagen
int  sizeImg; //Tamaño de la imagen
uint8_t *stringImg; //Buffer de la imagen

extern uint32_t recv_size ;
extern uint8_t camera_error;


// Variables de gestor de batería
extern struct DS2764 DS2764_dev;


// Variables de SD
char depurado_txt [2000] = {0}; //A escribir en depurado_txt.txt
char datos_txt    [2000] = {0}; //A escribir en datos.txt

//Variables de Sim7600 y de comunicación con servidor
unsigned char datatime[25] = {0};
unsigned char operad2[255] = {0};
uint8_t CSQ_Values [2] = {0};
char    JSON_Datos [1000] = {0};
uint8_t Server_Response[100] = {0};
uint8_t Server_Values[13] = {0};
uint8_t log_present[100] = {0}; //Log que resulte pertinente enviar al servidor
uint16_t espera_cobertura;
uint8_t  rssi;
uint8_t  ber;
int Sim7600_status = 0;
int Sim7070_status = 0;
int Sim_status_aux = 0;


//Variables de configuración
uint8_t numero_tlf[9];
uint8_t update_EEPROM;
struct s_config_Data{
	unsigned short int hora_dormir				; //hora
	unsigned short int hora_despertar			; //hora
	unsigned short int min_intervalo_dia		; //minutos
	unsigned short int min_espera_cobertura		; //minutos
	unsigned short int modo_trabajo				; //Ver "Modos"
	unsigned short int ampliacion_intervalo_dia ;
	unsigned short int nivel_log				; //0= sin log, 1=log basico, 2=log detallado, 3=log basico+img, 4=log detallado+img
	unsigned short int nivel_sms				;
	//unsigned char numero_tlf[9]; //Lo guardé a parte porque sino se quedaba colgada (Nuria?)
} Config_Data;

/* Modos // Para operar a distancia
 * Modo 0 --> DEBUG (envío continuo de datos e imágenes)
 * Modo 1 --> Día: envío datos e imágenes   Noche: Dormir
 * Modo 2 --> Día: envío imágenes			Noche: Dormir
 */


//Variables de sensores
tDS18B20Dev  temper ;
tDS18B20bDev temperb;
int temp_val = 0;
int temp_sub = 0;
float presure = 0.0;
float level=0.0;
uint16_t AO_out[60] = {0};
uint16_t _turb;
uint16_t _cond;
uint16_t _piro;
uint16_t _ph;
uint16_t _o2;
char phmedia_str[100], phmedia12bit_str[100] = {"0"};
char o2media_str[100], o2media12bit_str[100] = {"0"};
char condmedia_str[100], condmedia12bit_str[100] = {"0"};
float accel[3];

//Otras variables
char temp[50] = {0};
uint8_t errorfatal   = 0;
uint8_t errornofatal = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void _Error_Handler(char *, int);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_DCMI_Init(void);
static void MX_FMC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI6_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RTC_HOUR            (void);
void assign_photo_memory_space();
void lectura_sensor_ADS (char* tension_media, char* tension_12_bit, uint8_t sensor);
void preparar_JSON();
void calcular_siesta();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	Delay(300); //Estabilizar el reloj para obtener un debug por pantalla óptimo
	printf("------------------------- Parte 1: INICIALIZACION -------------------------\n");


	printf("[Main.c] Activacion de la alimentacion de los sensores\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)  ; //ON ALIM SENSORES
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //ON ALIM PH
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //ON ALIM O2
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //ON ALIM COND

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET);

	MX_DMA_Init();
	MX_RTC_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_SPI6_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_ADC2_Init();

  /* USER CODE BEGIN 2 */

	clock_init();			//Inicializa el reloj
	leds_init();
    Delay (1000);

    printf("[Main.c] Activacion de la alimentacion del modulo\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Encendido del módem
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //Reset del módem
    Delay(2000);

    printf("[Main.c] Inicializacion y prueba de RAM externa\n");
    BSP_SDRAM_Init();  //Inicializa
	memory_test();	//Testea

	//COMENTARIO DE PRUEBA

 //Congiuración del RTC Externo
	PCF8523_Set_12_24(MODO_24h,&hi2c1); // Modo 24h: Formato de 24 horas
	PCF8523_Disable_CLKOUT(&hi2c1); 	// Para que el Clock no salga por INT1
	PCF8523_SET_POWERMODE(&hi2c1); 		// Powermode estandar


	//Actualización manual de fecha (ejecutar una sola vez y cambiar la fecha en la primera carga del código!!! - Diego)
	/*printf("[Main.c] WARNING: Actualizacion manual de fecha activada\n");
	fecha.Day = 15;
	fecha.Month = 02;
	fecha.Year = 2021;
	PCF8523_Set_Date(fecha, 0, &hi2c1);
*/

	/* Obtener fecha y hora */
	PCF8523_Get_Date(&fecha, 0, &hi2c1);
	PCF8523_Get_Time(&hora, 0, &hi2c1);

	uint8_t hora_inicio = hora.Hours	;
	uint8_t min_inicio  = hora.Minutes ;

	printf("[Main.c] Fecha actual:   %02d/%02d/%d\n", fecha.Day, fecha.Month, fecha.Year);
	printf("[Main.c] Hora de inicio: %02d:%02d:%02d\n", hora.Hours, hora.Minutes, hora.Seconds);

	EEPROM_Read_NBytes(numero_tlf, sizeof(numero_tlf), NUM_TLF_ADD,  &hi2c1);
	Delay(READ_TIME);

	EEPROM_Read_NBytes((uint8_t*)&Config_Data, sizeof(struct s_config_Data), CONFIG_DATA_ADD,  &hi2c1);
	Delay(READ_TIME);

	if ((Config_Data.hora_dormir == 0 && Config_Data.hora_despertar == 0)|| //Si dormir y despertar a la misma hora
		 Config_Data.hora_dormir > 23 || Config_Data.hora_despertar < 0  || //Horas no válidas de dormir o despertarse
		 Config_Data.modo_trabajo > 2 || Config_Data.modo_trabajo < 0)      //Modos de trabajo no válidos

	{
		printf("[Main.c] WARNING: Datos EEPROM no válidos, utilizando valores por defecto\n");

		Config_Data.hora_despertar			 = DEFAULT_WKUP				;
		Config_Data.hora_dormir				 = DEFAULT_SLEEP			;
		Config_Data.min_intervalo_dia		 = DEFAULT_WAIT_INTERVAL	;
		Config_Data.min_espera_cobertura	 = DEFAULT_WAIT_CONNECTION	;
		Config_Data.modo_trabajo			 = DEFAULT_MODE				;
		Config_Data.ampliacion_intervalo_dia = DEFAULT_AMPL_INTERVAL	;
		Config_Data.nivel_log			     = DEFAULT_LOG_LEVEL		;
		Config_Data.nivel_sms				 = DEFAULT_SMS_LEVEL		;
		EEPROM_Write_NBytes((uint8_t*)&Config_Data, sizeof(struct s_config_Data), 0x50,  &hi2c1);
		Delay(WRITE_TIME);

		printf("[Main.c] Configuracion por defecto: \n");
		printf("         Hora de despertar:     %d\n", Config_Data.hora_despertar);
		printf("         Hora de dormir:        %d\n", Config_Data.hora_dormir);
		printf("         Intervalo toma datos:  %d\n", Config_Data.min_intervalo_dia);
		printf("         Espera cobertura:      %d\n", Config_Data.min_espera_cobertura);
		printf("         Modo:                  %d\n", Config_Data.modo_trabajo);
		printf("         Tiempo ampliacion:     %d\n", Config_Data.ampliacion_intervalo_dia);
		printf("         Nivel log:             %d\n", Config_Data.nivel_log);
		printf("         Nivel SMS:             %d\n", Config_Data.nivel_sms);
		printf("         Numero telefono:       %s\n", numero_tlf);
	}

	else
	{
		printf("[Main.c] Datos de EEPROM correctos\n");
		printf("[Main.c] Configuracion guardada en EEPROM: \n");
		printf("         Hora de despertar:     %d\n", Config_Data.hora_despertar);
		printf("         Hora de dormir:        %d\n", Config_Data.hora_dormir);
		printf("         Intervalo toma datos:  %d\n", Config_Data.min_intervalo_dia);
		printf("         Espera cobertura:      %d\n", Config_Data.min_espera_cobertura);
		printf("         Modo:                  %d\n", Config_Data.modo_trabajo);
		printf("         Tiempo ampliacion:     %d\n", Config_Data.ampliacion_intervalo_dia);
		printf("         Nivel log:             %d\n", Config_Data.nivel_log);
		printf("         Nivel SMS:             %d\n", Config_Data.nivel_sms);
		printf("         Numero telefono:       %s\n", numero_tlf);
	}

	//COMIENZO DE DEPURADO EN SD
	printf("[Main.c] Inicio de depurado en tarjeta SD\n");

	SD_mount();
	SD_init(SD_HAS_LOG, SD_HAS_PHOTO);

	SD_print(depurado_txt, "\n\n---------- LOG %02d:%02d - %02d/%02d/%d ----------\n", hora_inicio, min_inicio, fecha.Day, fecha.Month, fecha.Year);

	PCF8523_Get_Time(&hora, 0, &hi2c1);



	/////* Doze *//////
	//Sueño ligero, para que el despertar con envío dat/img sea preciso
	/*if(MODO_DORMIR_NOCHE) //Si el modo actual permite dormir por la noche (ver conf.h)
	{
		if ((HM_fin < horadespertar) || (HM_fin > horadormir)) //Si la hora actual es más pronto que la hora de despertar o más tarde que la hora de dormir
		{
			printf("[Main.c] Hora de dormir\n");

			if (HM_fin > horadormir)  //Si la hora actual es más tarde que la hora de dormir
			{
				to_medianoche = MEDIANOCHE - HM_fin;
				sleepmin = to_medianoche + horadespertar - 180; //Despertamos a 3 horas(que lo mismo son 4 que 2)?? (Supongo que para luego calcular mejor los minutos de dormir que queden? -Diego)
			}

			else  //Si la hora actual es más pronto que la hora de despertar
			{
				sleepmin = horadespertar - HM_fin; //El ultimo sueño ligero es corto y debe despertarse muy preciso

				if(sleepmin > 90) //Si queda más de 1 hora y media para la hora de despertarse mejor dormirse 1 hora y media.
					sleepmin  = 60; //Sueño ligero 60 min
			}

			sleepseg = sleepmin * 60;

			if(sleepseg <= 0) sleepmin = 10, sleepseg = 600;

			Sim7600_SleepModeIn();
			HAL_SuspendTick(); //Disable tick interruption.

			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			printf("[Main.c]: Duerme %d min\n\n", sleepmin);

			if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepseg, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
			{
				Error_Handler();
			}

			HAL_GPIO_WritePin		(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)  ; //OFF ALIM PH
			HAL_GPIO_WritePin		(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)  ; //OFF ALIM O2
			HAL_GPIO_WritePin		(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)  ; //OFF ALIM COND
			HAL_GPIO_WritePin		(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //OFF ALIM SENSORES
			HAL_PWR_EnterSTANDBYMode()								   ; // A dormir

		}//((HM_fin < horadespertar) || (HM_fin > horadormir))
	}//((Config_Data.modo_trabajo != 5)||(Config_Data.modo_trabajo != 4)||(Config_Data.modo_trabajo != 3))*/


//////* Operation Mode *///////

	switch(Config_Data.modo_trabajo)
	{
	case MODO_DEBUG:
		printf("[Main.c] Modo DEBUG -> Envio continuo datos e imagenes\n\n");
		SD_print(depurado_txt, "[Main.c] Modo DEBUG -> Envio continuo datos e imagenes\n\n");
		break;

	case MODO_1:
		printf("[Main.c] Modo 1 -> Dia: Envio de datos e imagenes, Noche: Dormir\n\n");
		SD_print(depurado_txt, "[Main.c] Modo 1 -> Dia: Envio de datos e imagenes, Noche: Dormir\n\n");
		break;

	case MODO_2:
		printf("[Main.c] Modo 2:  Dia: Envio de imagenes, Noche: Dormir\n\n");
		SD_print(depurado_txt, "[Main.c] Modo 2:  Dia: Envio de imagenes, Noche: Dormir\n\n");
		break;

	default:
		printf("[Main.c] WARNING: Modo desconocido! Usando MODO_DEBUG\n");
		SD_print(depurado_txt, "[Main.c] WARNING: Modo desconocido! Usando MODO_DEBUG\n");
	}


 /*BATTERY MONITOR*/
  DS2764_dsInit();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("[Main.c] Bucle infinito\n");
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifdef BATMAN
  if (Config_Data.modo_trabajo != MODO_2)
	  {
 		  printf("\n------------------------- Parte 2: TOMA DE DATOS -------------------------\n");

		  /*TEMPERATURE SENSOR*/
		  DS18B20Init(&temper);

		  /*TEMPERATURE SENSOR SUB*/ //PORQUÉ HAY DOS LIBRERÍAS IDÉNTICAS PERO CON DISTINTO NOMBRE PARA UN MISMO TIPO DE SENSOR????????
		  DS18B20bInit(&temperb);


		  printf("[Main.c] Lectura de pH...\n");
		  lectura_sensor_ADS(phmedia_str, phmedia12bit_str, PH);
		  printf("		  MEDIDA DE PH: %s Voltios, %s Escala de 12 bit\n", phmedia_str, phmedia12bit_str);

		  printf("[Main.c] Lectura de oxigeno...\n");
		  lectura_sensor_ADS(o2media_str, o2media12bit_str, O2);
		  printf("		  MEDIDA DE O2: %s Voltios, %s Escala de 12 bit\n", o2media_str, o2media12bit_str);

		  printf("[Main.c] Lectura de conductividad...\n");
		  lectura_sensor_ADS(condmedia_str, condmedia12bit_str, COND);
		  printf("		  MEDIDA DE COND: %s Voltios, %s Escala de 12 bit\n", condmedia_str, condmedia12bit_str);

			//ADC
		  printf("[Main.c] Inicio de ADC\n");
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)AO_out, 28);

		  Delay(1000);

		  Delay(1000);
		  char temp[30];

		  printf("Linea 640: CPU temp => %d %d\n", AO_out[6],AO_out[13]);

		  //Conversion altura del agua
		  //level=((AO_out[7]*4)*0.000805);
		  //Lectura del ADC
		  //El primer 2 es por que 1V son 2 mWG, el segundo para pasar de la escala 2.5V a 5V, y el 0.000805V es por que es 1 ADC
		  //Da el nivel en metros
		  //printf("Linea 653: Nivel en Metros => %f\n",level);

		#ifdef DS18B20
		  DS18B20Reset(&temper);
		  Delay(1);
		  DS18B20ROMRead(&temper);

		  DS18B20Reset(&temper);
		  Delay(1);
		  //DS18B20ROMMatch(&temp);
		  DS18B20ROMSkip(&temper);
		  DS18B20TempConvert(&temper);
		  Delay(500);
		  DS18B20Reset(&temper);
		  Delay(1);
		  //DS18B20ROMMatch(&temp);
		  DS18B20ROMSkip(&temper);
		  DS18B20TempRead(&temper, &temp_val);

		  printf("[Main.c] Temperatura Ext => %d\n",temp_val);
		#endif
		  //TODO ES NECESARIO TODOS ESTOS PASOS PARA INICIAR LA SONDA DE TEMPERATURA SUB
		#ifdef DS18B20b
		  DS18B20bReset(&temperb);
		  Delay(1);
		  DS18B20bROMRead(&temperb);
		  DS18B20bReset(&temperb);
		  Delay(1);
		  //DS18B20bROMMatch(&tempb);
		  DS18B20bROMSkip(&temperb);
		  DS18B20bTempConvert(&temperb);
		  Delay(500);
		  DS18B20bReset(&temperb);
		  Delay(1);
		  //DS18B20bROMMatch(&tempb);
		  DS18B20bROMSkip(&temperb);
		  DS18B20bTempRead(&temperb, &temp_sub);

		  printf("[Main.c] Temperatura Sub => %d\n",temp_sub);
		#endif



	  }
#endif
	  printf("[Main.c] Lectura de valores del gestor de bateria\n");

	  myFtoa(DS2764_dev.mfCurrent, temp); //Se convierte de float a string porque el depurado_txt SWV da problemas para imprimir floats
	  printf("Linea 1076: Amp => %s\n", temp);

	  printf("Linea 1079: Corriente Acumulada => %d\n", DS2764_dev.miAccCurrent);

	  myFtoa(DS2764_dsGetBatteryCapacityPercent(), temp);
	  printf("Linea 1082: Porcentaje batería => %s\n", temp);

	  SD_print(depurado_txt, "[Main.c] Nivel de bateria: %s\n", temp);


	  //init();
	  //TODO ES NECESARIO TODOS ESTOS PASOS PARA INICIAR LA SONDA DE TEMPERATURA

	  Delay(1000);

	  printf("\n----------------------- Parte 3: ESTABLECIMIENTO MODULO DE COMUNICACIONES -------------------------\n");
	  if(Sim7600_Module_Info()==SIMCOM7600)
	  {
		printf("[Main.c]: Sim7600 detectado \n", datatime);
	  // DESPERTAR Sim7600
	  printf("[Main.c] Despertando Sim7600\n");
	  Sim7600_status = Sim7600_Wake_Up();
	  Sim7600_Status_Handler (Sim7600_status);

	  if (Sim7600_status != WAKE_UP_SUCCESS)
	  {
		  errorfatal = 1;
	  }

	  if(errorfatal == 0)
	  {
		  // REINICIAR Sim7600
		  printf("[Main.c] Reiniciando Sim7600\n");

		  Sim7600_status = Sim7600_Reset();
		  Sim7600_Status_Handler (Sim7600_status);
		  printf("[Main.c] Esperando Reinicio\n");
		  //Delay(30000);
		  printf("[Main.c] Reiniciado\n");

		  if (Sim7600_status == RESET_SUCCESS)
		  {
			  // COMPROBAR Sim7600
			  printf("[Main.c] Comprobando Sim7600\n");
			  Delay(1000); //DEBUG
			  Sim7600_status = Sim7600_Comprobacion();
			  Sim7600_Status_Handler (Sim7600_status);

			  if (Sim7600_status != COMPROB_OK)
			  {
				  if(errorfatal==0) errorfatal=2;
			  }

			  if(errorfatal == 0)
			  {
				  // CONECTANDO Sim7600
				  Delay(1000); //DEBUG
				  printf("[Main.c] Conectando Sim7600 a la operadora MOVISTAR y abriendo bearer\n");
				  Sim7600_status=Sim7600_PS_status();
				  Sim7600_Status_Handler (Sim7600_status);
				  Sim7600_status = Sim7600_Network_Bearing();
				  Sim7600_Status_Handler (Sim7600_status);

				 if (Sim7600_status != CONNECTION_SUCCESS )
				  {
					 	 //No se pone error_fatal = 1 porque a veces da falsos positivos de errores
				  }

			  } //errorfatal==0
	      } //Sim7600_status == RESET_SUCCESS
		  else
		  {
			  if(errorfatal == 0) errorfatal = 3;
	      }
	  	  }
	   }//errorfatal==0
	  else if(Sim7600_Module_Info()==SIMCOM7070)
	  	  {
	  		printf("[Main.c]: Sim7070 detectado \n", datatime);
	  	  // DESPERTAR Sim7600
	  	  printf("[Main.c] Despertando Sim7070\n");
	  	  Sim7070_status = Sim7070_Wake_Up();
	  	Sim7070_Status_Handler (Sim7070_status);

	  	  if (Sim7070_status != WAKE_UP_SUCCESS)
	  	  {
	  		  errorfatal = 1;
	  	  }

	  	  if(errorfatal == 0)
	  	  {
	  		  // REINICIAR Sim7600
	  		  printf("[Main.c] Reiniciando Sim7070\n");

	  		  Sim7070_status = Sim7070_Reset();
	  		  Sim7070_Status_Handler (Sim7070_status);
	  		  printf("[Main.c] Esperando Reinicio\n");
	  		  //Delay(30000);
	  		  printf("[Main.c] Reiniciado\n");

	  		  if (Sim7070_status == RESET_SUCCESS)
	  		  {
	  			  // COMPROBAR Sim7600
	  			  printf("[Main.c] Comprobando Sim7070\n");
	  			  Delay(1000); //DEBUG
	  			  Sim7070_status = Sim7070_Comprobacion();
	  			  Sim7070_Status_Handler (Sim7070_status);

	  			  if (Sim7070_status != COMPROB_OK)
	  			  {
	  				  if(errorfatal==0) errorfatal=2;
	  			  }

	  			  if(errorfatal == 0)
	  			  {
	  				  // CONECTANDO Sim7600
	  				  Delay(1000); //DEBUG
	  				  printf("[Main.c] Conectando Sim7070 \n");
	  				Sim7070_status=Sim7070_Connect();
						Sim7070_Status_Handler (Sim7070_status);
	  				 if (Sim7070_status != CONNECTION_SUCCESS )
	  				  {
	  					 	 //No se pone error_fatal = 1 porque a veces da falsos positivos de errores
	  				  }

	  			  } //errorfatal==0
	  	      } //Sim7600_status == RESET_SUCCESS
	  		  else
	  		  {
	  			  if(errorfatal == 0) errorfatal = 3;
	  	      }
	  	  	  }
	  	   }//errorfatal==0

	  printf("\n------------------------- Parte 4: TOMA DE IMAGEN -------------------------\n");

	  printf("[Main.c] Asignando memoria en la RAM externa para la foto\n\n");
	  assign_photo_memory_space();

	  printf("[Main.c] Detectando camara..\n");
	  camera_detect();
	  if (!camera_error) //Solo tomar foto si no ha habido error en la inicialización de la cámara
	  	  {
	  		  printf("\n[Main.c] Captura de imagen...\n");
	  		  Delay(10000);

	  		  do
	  		  {
	  			  take_picture(stringImg);
	  			  sizeImg = get_picture_size(stringImg);

	  			  if(sizeImg <= 0){
	  				  printf("Imagen mal tomada\n");
	  				  sdram_flush(); //Borrar DRAM antes de apagarla
	  			  }

	  		  } while(sizeImg <= 0);



	  		  printf("\n[Main.c] Tamanyo de la imagen %d\n\n", sizeImg);
	  		  SD_print(depurado_txt, "[Main.c] Tamanyo de la imagen: %d\n", sizeImg);

	  		  PCF8523_Get_Time(&hora, 0, &hi2c1);
	  	      //e_sprintf(filename,"%02d%02d%02d%02d.jpg", fecha.Day, fecha.Month, hora.Hours, hora.Minutes);

	  		  e_sprintf(filename, "%d.jpg", sizeImg);
	  		  printf("[Main.c] Nombre de la imagen: %s\n", filename);
	  		  SD_print(depurado_txt, "[Main.c] Nombre de la imagen: %s\n", filename);
	  		  SD_save_JPEG(filename, stringImg, sizeImg);


	  		  if(sizeImg == MAX_IMG_SIZE)
	  			  strcat(log_present, "WARNING: PHOTO_NOT_TAKEN\\n");

	  		}//!camera_error

		Delay(100);



		printf("\n------------------------- Parte 5: SUBIDA AL SERVIDOR -------------------------\n");

		if(errorfatal == 0)
		{
			if(Sim7600_Module_Info()==SIMCOM7600)
			{
			  Sim7600_status = Sim7600_HoraInterna(datatime);
			  Sim7600_Status_Handler (Sim7600_status);
			  if(Sim7600_status == INTERNAL_TIME_SUCCESS)
			  {
				  printf("[Main.c]: Fecha y hora Sim7600: %s\n", datatime);
				  SD_print(depurado_txt, "[Main.c]: Fecha y hora Sim7600: %s\n", datatime);
			  }

			  printf("[Main.c] Comprobando operadora conectada: \n");
			  SD_print(depurado_txt, "[Main.c] Comprobando operadora conectada: \n");

			  Sim7600_status = Sim7600_operadora(operad2);
			  Sim7600_Status_Handler (Sim7600_status);

			  if(Sim7600_status != OPERATOR_CONNECTED_SUCCESS)
			  {
				  if (errorfatal == 0) errorfatal = 4;
			  }

			  printf("%s\n", operad2);
			  SD_print(depurado_txt, "%s\n", operad2);

			  printf("[Main.c] Obteniendo valores CSQ\n");
			  Sim7600_status = Sim7600_UpdateCSQ(CSQ_Values); // Update global var rssi and ber
			  Sim7600_Status_Handler (Sim7600_status);
			  if(Sim7600_status == CSQ_UPDATE_SUCCESS)
			  {
				  printf("[Main.c] Valores obtenidos: RSSI -- %d, BER -- %d\n", CSQ_Values[0], CSQ_Values[1]);
				  SD_print(depurado_txt,"[Main.c] Valores obtenidos: RSSI -- %d, BER -- %d\n", CSQ_Values[0], CSQ_Values[1]);
			  }
			}
			else if(Sim7600_Module_Info()==SIMCOM7070)
			{
				  Sim7070_status = Sim7600_HoraInterna(datatime);
				  Sim7070_Status_Handler (Sim7070_status);
				  if(Sim7070_status == INTERNAL_TIME_SUCCESS)
				  {
					  printf("[Main.c]: Fecha y hora Sim7070: %s\n", datatime);
					  SD_print(depurado_txt, "[Main.c]: Fecha y hora Sim7070: %s\n", datatime);
				  }

				  printf("[Main.c] Comprobando operadora conectada: \n");
				  SD_print(depurado_txt, "[Main.c] Comprobando operadora conectada: \n");

				  Sim7070_status = Sim7070_operadora(operad2);
				  Sim7070_Status_Handler (Sim7070_status);

				  if(Sim7070_status != OPERATOR_CONNECTED_SUCCESS)
				  {
					  if (errorfatal == 0) errorfatal = 4;
				  }

				  printf("%s\n", operad2);
				  SD_print(depurado_txt, "%s\n", operad2);

				  printf("[Main.c] Obteniendo valores CSQ\n");
				  Sim7070_status = Sim7070_UpdateCSQ(CSQ_Values); // Update global var rssi and ber
				  Sim7070_Status_Handler (Sim7070_status);
				  if(Sim7070_status == CSQ_UPDATE_SUCCESS)
				  {
					  printf("[Main.c] Valores obtenidos: RSSI -- %d, BER -- %d\n", CSQ_Values[0], CSQ_Values[1]);
					  SD_print(depurado_txt,"[Main.c] Valores obtenidos: RSSI -- %d, BER -- %d\n", CSQ_Values[0], CSQ_Values[1]);
				  }


			}
		}//errorfatal==0

		if(camera_error == 0 && errorfatal == 0)
		{
			printf("\n--------- Parte 5.1: SUBIDA DE IMAGEN ---------\n");

			if(Sim7600_Module_Info()==SIMCOM7600)
			{
			printf("[Main.c] Iniciando protocolo HTTP\n");
			Sim7600_status = Sim7600_Init_HTTP(FOTO,"http://riverview.es/clicnfish/sensors/5c766b7600a7895d784a6efa/add");
			//Sim7600_status = Sim7600_Init_HTTP(FOTO,"http://www.riverview.es/clicnfish/sensors/5c766b7600a7895d784a6efa/add");

			Sim7600_Status_Handler (Sim7600_status);

			printf("[Main.c] Realizando un POST request\n");
			Sim7600_status = Sim7600_PostImagen(filename, stringImg, sizeImg);
			Sim7600_Status_Handler (Sim7600_status);

			if(Sim7600_status != IMAGE_POST_SUCCESS)
				  errornofatal = 3;

			printf("[Main.c] Esperando a que se actualice el servidor...\n");
			//Delay(IMG_SERVER_WAIT_TIME); //Esperar a que se suba la imagen (Si no se espera habrá un error con la terminación del servicio HTTP)


			Delay(10000);
			//printf("[Main.c] El servidor debe haberse actualizado\n\n");

			printf("[Main.c] Terminando protocolo HTTP\n");
			Sim7600_status = Sim7600_Term_HTTP();
			Sim7600_Status_Handler (Sim7600_status);

			if(Sim7600_status != TERM_HTTP_SUCCESS) //Si no se ha cerrado bien la sesión HTTP
			{
				printf("[Main.c] WARNING: No se establecio conexion con el servidor de imagenes.\n");
				SD_print(depurado_txt, "[Main.c] WARNING: No se establecio conexion con el servidor de imagenes.\n");
			}

			}
			else if (Sim7600_Module_Info()==SIMCOM7070)
				printf("[Main.c] Sim7070 conectado. No habra envio de imagen.\n");

		}

		if(errorfatal == 0)
		{
			printf("\n--------- Parte 5.2: SUBIDA DE DATOS ---------\n");

			if(Sim7600_Module_Info()==SIMCOM7600)
			{
			//Delay(15000);
			printf("[Main.c] Iniciando protocolo HTTP\n");
			//Sim7600_status = Sim7600_Init_HTTP(DATOS,"http://ptsv2.com/t/ddysl-1617033560/post");
			Sim7600_status = Sim7600_Init_HTTP(DATOS,"https://www.riverview.es/clicnfish/sensors/5c766b7600a7895d784a6efa/addmeasurement");
			Sim7600_Status_Handler (Sim7600_status);

			printf("[Main.c] Creando objeto JSON con los datos a enviar\n");

			preparar_JSON();
			printf("%s",JSON_Datos);
			printf("[Main.c] Realizando un POST request\n");
			Sim7600_status = Sim7600_Data_Post(JSON_Datos, myStrlen(JSON_Datos));
			Sim7600_Status_Handler (Sim7600_status);

			if (Sim7600_status != DATA_POST_SUCCESS)
			{
				errornofatal = 1;
			}

			printf("[Main.c] Esperando respuesta del servidor\n");

			Delay(30000);

			//Delay(5000);

			Sim_status_aux = Sim7600_Resp_Servidor(Server_Values);
			Sim7600_Status_Handler (Sim_status_aux);
			printf("[Main.c] Terminando servicio HTTP\n");
			Sim7600_status = Sim7600_Term_HTTP();
			Sim7600_Status_Handler (Sim7600_status);
			}
			else if(Sim7600_Module_Info()==SIMCOM7070)
			{
				printf("[Main.c] Creando objeto JSON con los datos a enviar\n");

				preparar_JSON();
				printf("[Main.c] Iniciando protocolo HTTP\n");
				printf("[Main.c] Realizando un POST request\n");
				Sim7070_status=Sim7070_Init_HTTP("http://www.riverview.es/clicnfish/sensors/5c766b7600a7895d784a6efa/addmeasurement", JSON_Datos, myStrlen(JSON_Datos));
				Sim7070_Status_Handler (Sim7070_status);

				if (Sim7070_status != DATA_POST_SUCCESS)
				{
					errornofatal = 1;
				}

				printf("[Main.c] Esperando respuesta del servidor\n");

				Delay(25000);

				//Delay(5000);

				Sim_status_aux = Sim7070_Resp_Servidor(Server_Values);
				Sim7070_Status_Handler (Sim_status_aux);
				printf("[Main.c] Terminando servicio HTTP\n");
				Sim7070_status = Sim7070_Term_HTTP();
				Sim7070_Status_Handler (Sim7070_status);

			}

			if (Sim_status_aux == SERVER_RESPONSE_SUCCESS || Sim_status_aux == SERVER_RESPONSE_SUCCESS)
			{

				printf("[Main.c] Valores extraidos del servidor:\n");

				printf("         Hora servidor:         %02d:%02d:%02d\n", Server_Values[5], Server_Values[6], Server_Values[7]);
				printf("         Hora de despertar:     %d\n", Server_Values[0]);
				printf("         Hora de dormir:        %d\n", Server_Values[1]);
				printf("         Intervalo toma datos:  %d\n", Server_Values[2]);
				printf("         Espera cobertura:      %d\n", Server_Values[3]);
				printf("         Modo:                  %d\n", Server_Values[4]);
				printf("         Tiempo ampliacion:     %d\n", Server_Values[8]);
				printf("         Nivel log:             %d\n", Server_Values[9]);
				printf("         Nivel SMS:             %d\n", Server_Values[10]);
				printf("         Numero telefono:       %d\n", Server_Values[11]);
				printf("         Mensaje SMS:           %d\n", Server_Values[12]);


				Config_Data.hora_despertar 		     = Server_Values[0]; //horas
				Config_Data.hora_dormir			     = Server_Values[1];
				Config_Data.min_intervalo_dia		 = Server_Values[2]; //mins
				Config_Data.min_espera_cobertura	 = Server_Values[3];//Cobertura
				Config_Data.modo_trabajo			 = Server_Values[4];
				Config_Data.ampliacion_intervalo_dia = Server_Values[8];
				Config_Data.nivel_log				 = Server_Values[9];
				Config_Data.nivel_sms				 = Server_Values[10];

				//EEPROM_Write_NBytes(numero_tlf, sizeof(numero_tlf), NUM_TLF_ADD,  &hi2c1);
				//Delay(1000);
				EEPROM_Write_NBytes((uint8_t*)&Config_Data, sizeof(struct s_config_Data), CONFIG_DATA_ADD,  &hi2c1);
				Delay(1000);

				printf("[Main.c] Actualizar EEPROM a  %d,%d,%d,%d,%d,%d,%d,%d,%s\n",Config_Data.hora_despertar,
				   																    Config_Data.hora_dormir,
																					Config_Data.min_intervalo_dia,
																					Config_Data.min_espera_cobertura,
																					Config_Data.modo_trabajo,
																					Config_Data.ampliacion_intervalo_dia,
																					Config_Data.nivel_log,
																					Config_Data.nivel_sms,
																					numero_tlf);

				SD_print(depurado_txt, "[Main.c] Actualizar EEPROM a  %d,%d,%d,%d,%d,%d,%d,%d,%s\n",Config_Data.hora_despertar,
				   																  	  	  	     Config_Data.hora_dormir,
																							     Config_Data.min_intervalo_dia,
																							     Config_Data.min_espera_cobertura,
																							     Config_Data.modo_trabajo,
																							     Config_Data.ampliacion_intervalo_dia,
																							     Config_Data.nivel_log,
																							     Config_Data.nivel_sms,
																							     numero_tlf);


				Server_Values[5] = Server_Values[5] + 1; //Se añade 1 a la hora del servidor (La hora del servidor actualmente está retrasada)


				printf("[Main.c] Sincronizando RTC a servidor (el servidor esta retrasado 1 hora): %02d:%02d:%02d\n", Server_Values[5], Server_Values[6], Server_Values[7]);
				SD_print(depurado_txt, "[Main.c] Sincronizado RTC a servidor: %02d:%02d:%02d\n", Server_Values[5], Server_Values[6], Server_Values[7]);

				hora.Hours   = Server_Values[5];
				hora.Minutes = Server_Values[6];
				hora.Seconds = Server_Values[7];

				PCF8523_Set_Time(hora, 0, &hi2c1);

			} //if Sim7600_status == SERVER_RESPONSE_SUCCESS

			else
			{
				printf("[Main.c] WARNING: No se ha actualizado la EEPROM\n");
				SD_print(depurado_txt, "[Main.c] WARNING: No se ha actualizado la EEPROM\n");
			}



	  } //if(error_fatal == 0) (en parte 5.2 SUBIDA DE DATOS)

	  //Escritura de datos enviados en SD
	  SD_print(datos_txt, "----------------------------------\n");
	  SD_print(datos_txt, "HORA LOCAL DE INICIO: %02d:%02d - %02d/%02d/%d\n", hora_inicio, min_inicio, fecha.Day, fecha.Month, fecha.Year);
	  SD_print(datos_txt, "DATOS ENVIADOS\n");
	  SD_print(datos_txt, "pH 12 bit:            %s\n", phmedia12bit_str);
	  SD_print(datos_txt, "O2 12 bit:            %s\n", o2media12bit_str);
	  SD_print(datos_txt, "Conductividad 12 bit: %s\n\n", condmedia12bit_str);
	  SD_print(datos_txt, "----------------------------------\n\n\n");

	  SD_save_log("Datos.txt", datos_txt);


	  printf("\n------------------------- Parte 6: FINALIZACION -------------------------\n");

	  if(errorfatal != 1)
	  {
		  printf("[Main.c] Apagando módulo \n");
		  SD_print(depurado_txt, "[Main.c] Apagando módulo \n");
		  Sim7600_status = Sim7600_SleepModeIn(&huart2);
		  Sim7600_Status_Handler(Sim7600_status);

	   }

	 RTC_HOUR();

	 printf("[Main.c] Hora de fin: %02d:%02d:%02d\n", hora.Hours, hora.Minutes, hora.Seconds);
	 SD_print(depurado_txt, "[Main.c] Hora de fin: %02d:%02d\n", hora.Hours, hora.Minutes);

	 leds_off(LEDS_GREEN);
	 leds_off(LEDS_BLUE);
	 leds_off(LEDS_RED);

	 if(errorfatal == 0)
	 {
		 BSP_CAMERA_Suspend();
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET);

	 }//ConFoto

	 sdram_flush(); //Borrar DRAM antes de apagarla
	 Delay(10);

	/* ///DEBUG (Para simular hora de dormir)
	 hora.Hours   = 22;
	 hora.Minutes = 59;
	 hora.Seconds = 50;

	 PCF8523_Set_Time(hora, 0, &hi2c1);*/

	 RTC_HOUR();
	 calcular_siesta();

	 SD_save_log("Depurado.txt", depurado_txt);

	 HAL_SuspendTick();		//Disable tick interruption.
	 HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn); //Desactiva camara
	 HAL_NVIC_DisableIRQ(SDRAM_DMAx_IRQn); //Desactiva SDRAM
			//power down
			//HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			/* Disable all used wakeup sources: Pin1(PA.0) */
			//HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

			/* Clear all related wakeup flags */
	 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

			//APAGADO ALIMENTACIONES
	 printf("[Main.c] Desactivacion alimentacion principal de los sensores\n");

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //OFF ALIM PH
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //OFF ALIM O2
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //OFF ALIM COND
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //OFF ALIM SENSORES



	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	      //Delay(500);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);

	  printf("\n----------------------------- FIN MAIN -----------------------------\n\n\n");

	  HAL_PWR_EnterSTANDBYMode(); // A dormir


  } //WHILE
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**ADC1 GPIO Configuration
        PA2     ------> ADC1_IN2  TURB
        PA3     ------> ADC1_IN3  COND_DC (Transistor)
        PA7     ------> ADC1_IN7  COND_AC (Oscilador)
        PB0     ------> ADC1_IN8  PIRO
        //PC1     ------> ADC1_IN11 VELETA
        PC2     ------> ADC1_IN12 PH
        PC3     ------> ADC1_IN13 O2
        */
    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


/* DCMI init function */
static void MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	char erri2c[] = "\r\n\nError I2C";
	HAL_UART_Transmit(&huart3, (uint8_t *) erri2c, myStrlen(erri2c), 100);
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}


/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}



/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
 // RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm A
    */
/*  sAlarm.AlarmTime.Hours = 0x1;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x15;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
*/
    /**Enable the WakeUp
    */
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 800, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }

}



/* SPI6 init function */
static void MX_SPI6_Init(void)
{

  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{
	//La frecuencia con la que se alimenta al WWDG es PCLK1 / (4096*Prescaler) (Ver Manual HAL)
	//En este caso, PCLK1 = 90Hz y Prescaler = 8 => Frec_wwdg ~ 2.7KHz
	//Para un Window WatchDog (WWDG), el WWDG salta si no ha habido ninguna instrucción nueva hasta que se agota el contador Counter
	//Además, salta si se refresca el WWDG antes de que se agote el contador WWDG.
	//Por lo tanto, el tiempo mínimo y máximo en el que se puede refrescar el WWDG es (manual HAL):
	// t_min = (Counter - Window) / Frec_wwdg == 1ms
	// t_max = (Counter - 0x40) / Frec_wwdg == 20ms

	//NOTA Window y Counter tienen que ser un valor entre 0x40 (64) y 0x7F (127)

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 116;
  hwwdg.Init.Counter = 119;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
 /* hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;// a mi me lo genera como DMA2_Stream1 amtes hdma_memtomem_dma2_stream0
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)//aqui tb stream1
  {
    Error_Handler();
  }*/


  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
//  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
/*  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;*/
  /* SdramTiming */
/*  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler();
  }
*/
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC2
                           PC3 PC4 PC5 PC6
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pins : PB14 PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 PD2
                           PD4 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_2
                          |GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG3 PG6 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*MARCO*/
  /*Configure GPIO pins : PF0 PF2 PF3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_3;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);

}




/* USER CODE BEGIN 4 */
/*
 * Dummy function to avoid compiler error
 */
//void _init() {

//}


void BSP_CAMERA_ErrorCallback(void){
      leds_toggle(LEDS_RED);
      printf("Linea 2611: Frame Error\n");

}

void BSP_CAMERA_FrameEventCallback(void)
{
      leds_toggle(LEDS_BLUE);
      printf("Linea 2618: Frame event\n");
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	 if(hadc->Instance==ADC1)
	  {
		// dbg_write_str("adc1_end\n");
	  }
	 if(hadc->Instance==ADC3)
	  {
		// dbg_write_str("adc3_end\n");
	  }
}

 void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	 if(hadc->Instance==ADC1)
	{
		 printf("Linea 2360: ADC1 Error\n");
	}
	 if(hadc->Instance==ADC3)
	  {
		 printf("Linea 2364: ADC3 Error\n");
	  }
	 if (hadc->ErrorCode ==HAL_ADC_ERROR_INTERNAL  ) printf("ADC IP internal error: if problem of clocking, enable/disable, erroneous state");
	 if (hadc->ErrorCode ==HAL_ADC_ERROR_OVR  ) printf("Overrun error\n");
	 if (hadc->ErrorCode ==HAL_ADC_ERROR_DMA  ) printf("DMA transfer error ");

}


void HAL_SYSTICK_Callback(void){
	timing_handler();

}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);
	  leds_toggle(LEDS_RED);
	  leds_toggle(LEDS_GREEN);
	  leds_toggle(LEDS_BLUE);
	  Delay(300);

}
/*
 * @brief Nos da la hora del RTC en el momento de llamar a esta funcion
 */
void RTC_HOUR(void){
	  PCF8523_Get_Time(&hora, 0, &hi2c1);
	  //printf("%02d:%02d\n",hora.Hours,hora.Minutes);
	  return;
}

void assign_photo_memory_space()
{
	uint32_t i = 0;

	for(; i < 1000000; i++) //Vaciar al menos 1MB de la SDRAM para guardar la foto
	{
		*(__IO uint32_t*) (CAMERA_FRAME_BUFFER + i) = 0xffffffff;
	}

	stringImg = (uint8_t*)CAMERA_FRAME_BUFFER;

}


void lectura_sensor_ADS(char* tension_media, char* tension_12_bit, uint8_t sensor){

	unsigned char ADSwrite[6];
	int16_t reading;
	uint16_t sensor_reg; //Registro del sensor en el ADS

	uint8_t medidas;

	float voltage[4]       = {0}; // ARRAY PARA LOS 4 CANALES INICIALIZADO A CERO
	float tension_acc      = 0; //Valor acumulado
	float tension_media_f  = 0; //Valor de la tensión en float
	float factor           = 0;

	switch(sensor)
	{
	case PH:
		sensor_reg = ADS1115_ADDRESS;
		medidas = PH_MEASUREMENTS;
		factor = VOLTAGE_CONV_PH;
		break;

	case O2:
		sensor_reg = ADS1115_ADDRESS2;
		medidas = O2_MEASUREMENTS;
		factor = VOLTAGE_CONV_O2;
		break;

	case COND:
		sensor_reg = ADS1115_ADDRESS3;
		medidas = COND_MEASUREMENTS;
		factor = VOLTAGE_CONV_COND;
		break;

	default:
		return;
	}

	  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_7); //INDICADOR LED IZQUIERDO
	  Delay(STABILIZATION_TIME); //ESTABILIZACIÓN (5 segundos)

	  for(int i = 0; i < medidas; i++)    //SE TOMAN 10 MEDIDAS DEL SENSOR
	  {
		  Delay(TIME_BETWEEN_READS); //(150 ms)
		  //CONFIGURACIÓN
		  for(int j = 0; j < 4; j++)
		  {
			  ADSwrite[0] = CONFIG_REGISTER;
			  Delay (10);
			  switch(j)
			  {
			  	  case(0):
						  ADSwrite[1] = 0xC1; //11000001
				  	  	  break;
				  case(1):
						  ADSwrite[1] = 0xD1; //11010001
				  	      break;
				  case(2):
						  ADSwrite[1] = 0xE1;
						  break;
				  case(3):
					      ADSwrite[1] = 0xF1;
						  break;
			   }

			   ADSwrite[2] = 0x83; //10000011 LSB
			   if (HAL_I2C_Master_Transmit(&hi2c1, sensor_reg << 1, ADSwrite, 3, 1000)!= HAL_OK)
			   {
				   printf("[Main.c] ERROR: Comunicacion I2C con sensor fallida. Configuracion fallida.\n");
			   }

			   ADSwrite[0] = 0x00; //Entrando en modo lectura
			   if (HAL_I2C_Master_Transmit(&hi2c1, sensor_reg << 1 , ADSwrite, 1 ,1000)!= HAL_OK)
			   {
				   printf("[Main.c] ERROR: Comunicacion I2C con sensor fallida. Modo lectura fallido.\n");
			   }

			   Delay(50);

			   //LECTURA DEL ADC
			   if (HAL_I2C_Master_Receive(&hi2c1, sensor_reg << 1, ADSwrite, 2, 1000)!= HAL_OK)
			   {
				   printf("[Main.c] ERROR: Comunicacion I2C con sensor fallida. Lectura fallida.\n");
			   }

			   reading = (ADSwrite[0] << 8 | ADSwrite[1] );
			   if(reading < 0)
			   {
				   reading = 0;
			   }
			   voltage[j] = reading * factor;
		} //for(int j ...)

		tension_acc = tension_acc + voltage[0];
	 } //for(int i ...)

	 tension_media_f	  = tension_acc / medidas;
	 e_sprintf(tension_12_bit, "%d", (int) (tension_media_f * (4095 / 3.3)));

	 myFtoa(tension_media_f, tension_media);

	 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);//OFF ALIM PH
	 HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_7);//INDICADOR LED DERECHO
	 Delay (TIME_BETWEEN_SENSORS);

}

void preparar_JSON(){

	//FIXME Por favor, mejorar esto
	char tempchar[10];
	myFtoa (temp_val/100.0f, tempchar);
	char ampchar[10];
	myFtoa (DS2764_dev.mfCurrent, ampchar);

	switch(Config_Data.modo_trabajo)
	{
	case MODO_DEBUG:
	case MODO_1: //Enviar datos de los sensores
		e_sprintf(JSON_Datos ,
						"{\"values\":[{\"magnitude\":\"temp\",\"value\":%s},"
						"{\"magnitude\":\"amp\",\"value\":%s},"
						"{\"magnitude\":\"bat\",\"value\":%d},"
						"{\"magnitude\":\"turb\",\"value\":%d},"
						"{\"magnitude\":\"condt\",\"value\":%s},"
						"{\"magnitude\":\"piro\",\"value\":%d},"
						"{\"magnitude\":\"ph\",\"value\":%s},"
						"{\"magnitude\":\"o2\",\"value\":%s},"
						"{\"magnitude\":\"rssi\",\"value\":%d},"
						"{\"magnitude\":\"ber\",\"value\":%d},"
						"{\"magnitude\":\"hora\",\"value\":%d}],"
						"\"log_present\":\"%s\"}",
						tempchar, ampchar, DS2764_dev.miAccCurrent,_turb,condmedia12bit_str,_piro,phmedia12bit_str,o2media12bit_str,CSQ_Values[0],CSQ_Values[1],hora, log_present
					   );

		break;

	default: //En modos desconocidos y modo 2 solo se envia este pedazo de JSON
		strcat(log_present, "WARNING: Modo desconocido. Usando DEBUG");
	case MODO_2:
		e_sprintf(JSON_Datos ,
					    "{\"values\":[{\"magnitude\":\"bat\",\"value\":%d},"
			  			"{\"magnitude\":\"rssi\",\"value\":%d},"
			  			"{\"magnitude\":\"ber\",\"value\":%d},"
			  			"{\"magnitude\":\"hora\",\"value\":%d}],"
			  			"\"log_present\":\"%s\"}",
			  		    DS2764_dev.miAccCurrent, CSQ_Values[0], CSQ_Values[1], hora, log_present);
		printf("%s",JSON_Datos);
		printf("%s",JSON_Datos);
		break;
	}


}

void calcular_siesta(){

	int siesta = 60 * (Config_Data.min_intervalo_dia + Config_Data.ampliacion_intervalo_dia);

	switch (Config_Data.modo_trabajo) //Calcular si hay que dormir o no y cuánto
	{
	case MODO_DEBUG: //En el modo DEBUG, la única siesta posible es la del intervalo de envío de datos
		break;

	case MODO_1:
	case MODO_2:
		if(hora.Hours < Config_Data.hora_dormir - 1); //Si aún no es hora de dormir, la siesta es la indicada por intervalo_dia

		if(hora.Hours == Config_Data.hora_dormir - 1) //Si la hora actual es la hora anterior a cuando debe dormirse (ej: son las 20:59 y hay que dormir a las 21:00)
		{
			//Si lo que queda para que acabe la hora es menor que el intervalo para enviar datos
			if(60 - hora.Minutes <  Config_Data.min_intervalo_dia + Config_Data.ampliacion_intervalo_dia)
			{
				printf("[Main.c] Hora de dormir\n");
				SD_print(depurado_txt, "[Main.c] Hora de dormir\n");
				siesta = 3600 * (24 - (hora.Hours + 1) + Config_Data.hora_despertar) + (60 - (hora.Minutes + 1)) * 60 + (60 - hora.Seconds); //Dormir esta cantidad de segundos
			}

			if(60 - hora.Minutes ==  Config_Data.min_intervalo_dia + Config_Data.ampliacion_intervalo_dia) //Si lo que queda es igual al intervalo
			{
				if(hora.Seconds != 0) // Pero ya ha pasado cierta cantidad de segundos
				{
					printf("[Main.c] Hora de dormir\n"); //Dormir
					SD_print(depurado_txt, "[Main.c] Hora de dormir\n");
					siesta = 3600 * (24 - (hora.Hours + 1) + Config_Data.hora_despertar) + (60 - (hora.Minutes + 1)) * 60 + (60 - hora.Seconds); //Dormir esta cantidad de segundos
				}

			}
		}

		if(hora.Hours >= Config_Data.hora_dormir) //Si es la hora de dormir o más tarde
		{
			printf("[Main.c] Hora de dormir\n");
			SD_print(depurado_txt, "[Main.c] Hora de dormir\n");
			siesta = 3600 * (24 - (hora.Hours + 1) + Config_Data.hora_despertar) + (60 - (hora.Minutes + 1)) * 60 + (60 - hora.Seconds); //Dormir esta cantidad de segundos
		}

		break;

	default:
		printf("[Main.c] WARNING: Modo desconocido!");

	}

	printf("[Main.c] Siesta de %d min\n", (siesta/60));
	SD_print(depurado_txt, "[Main.c] Siesta de %d min\n", siesta/60);

	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, siesta+5*60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	{
		Error_Handler();
	}


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  /*while(1)
  {
  }*/
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
