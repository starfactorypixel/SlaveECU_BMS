/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#define DEBUG

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Params.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Button1 		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define Button2 		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define Button2 		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define STBY_H()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		// High
#define STBY_L()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	// Low

#define MARKER_FIRST_START  100           // Маркер что flash не пустая

// BMS_PACKET_SIZE - размер структуры params_t (140 байт на текущий момент)
// 60 - просто запас... на самом деле не особо нужен, наверное
// изначально размер буфера был 200 байт, оттуда запас 60 и вылез
#define UART3_BUFF_SIZE (BMS_PACKET_SIZE+60)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData[8] = {0,};
	uint8_t RxData[8] = {0,};
	uint32_t TxMailbox = 0;
	//uint8_t trans_str[30];  // used nowhere
	
	//_params_v params_obj; // used nowhere
	
	
	//------------------------  Ds18b20
	uint8_t Dev_ID[8][8]={0};
	uint8_t Dev_Cnt;
	char str1[60];
	//------------------------  ADC
	uint16_t ADC_value[10];
	uint8_t ADC_cnt_state = 0;
	uint16_t ADC_cnt = 0;
	uint16_t ADC_senors[10];
	//------------------------ Timer	
	
	//------------------------ UART	
	uint8_t receiveBuff_huart3[UART3_BUFF_SIZE];
	uint8_t receiveBuffStat_huart3[UART3_BUFF_SIZE];
	//uint16_t ReciveUartSize = 0;  // used nowhere
	uint8_t FlagReciveUART3 = 0;
	//------------------------ FLASH
	uint16_t write_data16[30];
	uint16_t read_data16[30];
	
	
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void HAL_CAN_Send();
void HAL_CAN_Send_Obj(_params_v *params_obj);
void write_flash();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------- Прерывание от USART3 по флагу Idle
// TODO: АЛЯРМА! Функция предполагает, что в буфере будет пакет целиком, причем первый байт заголовка попадёт
// в первый байт буфера. Есть ли в этом полная уверенность? Может ли быть такое, что в буфере будет остаток
// предыдущего пакета и начало следующего?
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance != USART3) return;
  if (Size < BMS_PACKET_SIZE) return;

  // Check BMS packet header
  // TODO: уже плохо соображаю... это же будет работтаь?
  uint32_t* BMS_header = (uint32_t*)receiveBuff_huart3; 
  if (BMS_header[0] != BMS_PACKET_HEADER) return;

  //ReciveUartSize = Size; // пока не понял, зачем оно нам тут

  // set flag that BMS packet received
  FlagReciveUART3 = 1;

  // copy data to intermediate buffer
  // TODO: where we are using intermediate buffer? Does we need it?
  //memcpy( receiveBuffStat_huart3, receiveBuff_huart3, Size );
  
  // fill the BMS structure with data
  memcpy( &params, receiveBuff_huart3, BMS_PACKET_SIZE );

  // clear buffer
  // TODO: do we really need clear the buffer?
  //memset( receiveBuff_huart3, 0, UART3_BUFF_SIZE );

  HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t*) receiveBuff_huart3, UART3_BUFF_SIZE);
}

//-------------------------------- Прерывание от таймера TIM1
void	IRQHandlerTIM1( void){

}
	
/* 
	Колбек для приёма данных CAN (для буфера RX_FIFO_1)
	При приёме любого кадра тут же забираем его из почтового ящика с помощью функции HAL_CAN_GetRxMessage(...)
	ВАЖНО!!!!!!  для приема нужно в MX_CAN_Init() прописать настройки фильтра
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {  
			uint16_t sw = RxHeader.StdId;
			
			switch (sw)
      {
      	case HighVoltage_ID:
					if(RxData[0] == 0x11){		// Если пришел запрос, то нужно отвечать
						TxData[0] = 0x51;				// Тип сообшения 0x51 - ответ на запрос, актуальное "нормальное" значение.
						HAL_CAN_Send_Obj(&HighVoltage);
					}
					else{
						if(RxData[0] == 0x12){
							// HighVoltage.period_ms = ((uint16_t)RxData[1] << 8) | RxData[2];
              HighVoltage.state = RxData[1];
              write_flash();
						}
					}
      		break;
      	case HighCurrent_ID:
					if(RxData[0] == 0x11){		
						TxData[0] = 0x51;				
						HAL_CAN_Send_Obj(&HighCurrent);
					}
					else{
						if(RxData[0] == 0x12){
							HighCurrent.state = RxData[1];
              write_flash();
						}
					}
          break;
      	case MaxTemperature_ID:
					if(RxData[0] == 0x11){		
						TxData[0] = 0x51;				
						HAL_CAN_Send_Obj(&MaxTemperature);
					}
					else{
						if(RxData[0] == 0x12){
							MaxTemperature.state = RxData[1];
              write_flash();
						}
					}
          break;
      	case Temperature1_ID:
					if(RxData[0] == 0x11){		
						TxData[0] = 0x51;	
            Temperature1.length = 8;	
						HAL_CAN_Send_Obj(&Temperature1);
					}
          break;
      	case Temperature2_ID:
					if(RxData[0] == 0x11){		
						TxData[0] = 0x51;				
						HAL_CAN_Send_Obj(&Temperature2);
					}
          break;
      	case Temperature3_ID:
					if(RxData[0] == 0x11){		
						TxData[0] = 0x51;				

						HAL_CAN_Send_Obj(&Temperature3);
					}
      		break;
      	case 0x07B0:    // для отладки, поморгать светодиодами
					if(RxData[6] == 0x31){
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					}
					if(RxData[6] == 0x32){
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					}
					if(RxData[6] == 0x33){
						Erase_flash(21);
					}
      		break;
      	default:
      		break;
      }

#ifdef DEBUG
			sprintf(str1,"CAN %c", RxHeader.StdId);
			HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 100);	
#endif
			

    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t er = HAL_CAN_GetError(hcan);
	sprintf(str1,"ER CAN %lu %08lX", (unsigned long)er , (unsigned long)er);
	HAL_UART_Transmit(&huart1, (uint8_t*)str1, strlen(str1), 100);
}

void HAL_CAN_Send()
{
	
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
#ifdef DEBUG
			sprintf(str1,"CAN %c", RxHeader.StdId);
			HAL_UART_Transmit(&huart1, (uint8_t*)"ER SEND\n", 8, 100);
#endif
		}
}

void HAL_CAN_Send_Obj(_params_v *params_obj)
{
	TxHeader.StdId = params_obj->ID;
	TxHeader.DLC = params_obj->length;

  memset(TxData, 0, 8);
  TxData[0] = params_obj->type;

	for(uint8_t i=1; i<8; i++){
		TxData[i] = params_obj->data[i];
	}
  
	HAL_CAN_Send();
}

uint8_t ADCtoTEMPER(uint16_t adc_val){
  uint8_t result;
	result = ((adc_val&0x07FF)>>6);
  // result = adc_val/256;
  return result;
}

void readADC(void){
	
	for(uint8_t i=0;i<10;i++)
	{
		ADC_senors[i] = ADC_value[i];
		sprintf(str1,"ADC %d =", i);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		sprintf(str1," %d \r\n",ADC_senors[i]);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	}
	for(uint8_t i=1;i<8;i++){
    Temperature1.data[i] = ADCtoTEMPER(ADC_senors[i-1]);
    if(Temperature1.data[i]>MaxTemperature.data[1])
      MaxTemperature.data[1] = Temperature1.data[i];
	}
	for(uint8_t i=1;i<4;i++){
    Temperature2.data[i] = ADCtoTEMPER(ADC_senors[i+6]);
    if(Temperature2.data[i]>MaxTemperature.data[1])
      MaxTemperature.data[1] = Temperature2.data[i];
	}
}

void write_flash(){
	write_data16[0] = MARKER_FIRST_START;
	write_data16[1] = BlockInfo.state;
	write_data16[2] = BlockHealth.state;
	write_data16[3] = BlockCfg.state;
	write_data16[4] = BlockError.state;
	write_data16[5] = HighVoltage.state;
	write_data16[6] = HighCurrent.state;
  write_data16[7] = MaxTemperature.state;
	write_data16[8] = Temperature1.state;
	write_data16[9] = Temperature2.state;
	write_data16[10] = Temperature3.state;
	write_data16[11] = LowVoltage1_3.state;
	write_data16[12] = LowVoltage4_6.state;
	write_data16[13] = LowVoltage7_9.state;
	write_data16[14] = LowVoltage10_12.state;
	write_data16[15] = LowVoltage13_15.state;
	write_data16[16] = LowVoltage16_18.state;
	write_data16[17] = LowVoltage19_21.state;
	write_data16[18] = LowVoltage22_24.state;
	write_data16[19] = LowVoltageMinMaxDelta.state;
  write_data16[20] = MaxTemperature.data[2];

	Erase_flash(21);
	Write_flash_16b(21);
}

void readEeprom (uint8_t num){

	Read_flash_16b(num);

	// Обработка ситуации чистой EEPROM памяти
	// Если в flash нет данных, то заполнить и устанавливить маркер, указывающий, EEPROM инициализировано
	if(read_data16[0] != MARKER_FIRST_START){
	BlockInfo.state = 0x00;	
	BlockHealth.state = 0x00;
	BlockCfg.state = 0x00;
  BlockError.state = 0x00;
  HighVoltage.state = 0x01;     //  отравка по таймеру
	HighCurrent.state = 0x01;
	MaxTemperature.state = 0x01;
	Temperature1.state = 0x01;
	Temperature2.state = 0x01;
	Temperature3.state = 0x01;
  LowVoltage1_3.state = 0x00;
	LowVoltage4_6.state = 0x00;
	LowVoltage7_9.state = 0x00;
	LowVoltage10_12.state = 0x00;
	LowVoltage13_15.state = 0x00;
	LowVoltage16_18.state = 0x00;
  LowVoltage19_21.state = 0x00;
	LowVoltage22_24.state = 0x00;
	LowVoltageMinMaxDelta.state = 0x00;
  MaxTemperature.data[2] = ThresholdTemperature1;   // пороговое значение предельной температуры

		write_flash();
	}
	
	Read_flash_16b(num);
	BlockInfo.state = read_data16[1];
	BlockHealth.state = read_data16[2];
	BlockCfg.state = read_data16[3];
  BlockError.state = read_data16[4];
  HighVoltage.state = read_data16[5];
	HighCurrent.state = read_data16[6];
	MaxTemperature.state = read_data16[7];
	Temperature1.state = read_data16[8];
	Temperature2.state = read_data16[9];
	Temperature3.state = read_data16[10];
  LowVoltage1_3.state = read_data16[11];
	LowVoltage4_6.state = read_data16[12];
	LowVoltage7_9.state = read_data16[13];
	LowVoltage10_12.state = read_data16[14];
	LowVoltage13_15.state = read_data16[15];
	LowVoltage16_18.state = read_data16[16];
  LowVoltage19_21.state = read_data16[17];
	LowVoltage22_24.state = read_data16[18];
	LowVoltageMinMaxDelta.state = read_data16[19];
  MaxTemperature.data[2] = read_data16[20];
}

uint16_t get_BMS_min_voltage(params_t* BMS_params){
  if (BMS_params == nullptr) return UINT16_MAX;

  uint16_t result = UINT16_MAX;
  //result = (BMS_params->voltage_1 < result) ? BMS_params->voltage_1 : result;
  result = min(BMS_params->voltage_1, result);
  result = min(BMS_params->voltage_2, result);
  result = min(BMS_params->voltage_3, result);
  result = min(BMS_params->voltage_4, result);
  result = min(BMS_params->voltage_5, result);
  result = min(BMS_params->voltage_6, result);
  result = min(BMS_params->voltage_7, result);
  result = min(BMS_params->voltage_8, result);
  result = min(BMS_params->voltage_9, result);
  result = min(BMS_params->voltage_10, result);
  result = min(BMS_params->voltage_11, result);
  result = min(BMS_params->voltage_12, result);
  result = min(BMS_params->voltage_13, result);
  result = min(BMS_params->voltage_14, result);
  result = min(BMS_params->voltage_15, result);
  result = min(BMS_params->voltage_16, result);
  result = min(BMS_params->voltage_17, result);
  result = min(BMS_params->voltage_18, result);
  result = min(BMS_params->voltage_19, result);
  result = min(BMS_params->voltage_20, result);
  result = min(BMS_params->voltage_21, result);
  result = min(BMS_params->voltage_22, result);
  result = min(BMS_params->voltage_23, result);
  result = min(BMS_params->voltage_24, result);
  return result;
}

uint16_t get_BMS_max_voltage(params_t* BMS_params){
  if (BMS_params == nullptr) return 0;

  uint16_t result = 0;
  //result = (BMS_params->voltage_1 < result) ? BMS_params->voltage_1 : result;
  result = max(BMS_params->voltage_1, result);
  result = max(BMS_params->voltage_2, result);
  result = max(BMS_params->voltage_3, result);
  result = max(BMS_params->voltage_4, result);
  result = max(BMS_params->voltage_5, result);
  result = max(BMS_params->voltage_6, result);
  result = max(BMS_params->voltage_7, result);
  result = max(BMS_params->voltage_8, result);
  result = max(BMS_params->voltage_9, result);
  result = max(BMS_params->voltage_10, result);
  result = max(BMS_params->voltage_11, result);
  result = max(BMS_params->voltage_12, result);
  result = max(BMS_params->voltage_13, result);
  result = max(BMS_params->voltage_14, result);
  result = max(BMS_params->voltage_15, result);
  result = max(BMS_params->voltage_16, result);
  result = max(BMS_params->voltage_17, result);
  result = max(BMS_params->voltage_18, result);
  result = max(BMS_params->voltage_19, result);
  result = max(BMS_params->voltage_20, result);
  result = max(BMS_params->voltage_21, result);
  result = max(BMS_params->voltage_22, result);
  result = max(BMS_params->voltage_23, result);
  result = max(BMS_params->voltage_24, result);
  return result;
}

int16_t get_BMS_max_temperature(params_t* BMS_params){
  if (BMS_params == nullptr) return INT16_MIN;

  int16_t result = INT16_MIN;
  result = max(BMS_params->temp_mos, result);
  result = max(BMS_params->temp_bal, result);
  result = max(BMS_params->temp_1, result);
  result = max(BMS_params->temp_2, result);
  result = max(BMS_params->temp_3, result);
  result = max(BMS_params->temp_4, result);
  // TODO: other temperature should be checked too
  return result;
}

void parse_CAN_registers_from_BMS_data(void){
  uint8_t data[7];
  uint32_t uint32_val = 0;
  
  // HighVoltage in mV, params.voltage in mV*10
  uint32_val = params.voltage*10;
  CAN_register_fill_data( &HighVoltage, sizeof(uint32_t), (uint8_t *) &uint32_val );
  
  uint32_val = (uint32_t)params.current;
  CAN_register_fill_data( &HighCurrent, sizeof(uint32_t), (uint8_t *) &uint32_val );

  // ------------ Cell voltages
  CAN_register_fill_uint16x3( &LowVoltage1_3, params.voltage_1, params.voltage_2, params.voltage_3 );
  CAN_register_fill_uint16x3( &LowVoltage4_6, params.voltage_4, params.voltage_5, params.voltage_6 );
  CAN_register_fill_uint16x3( &LowVoltage7_9, params.voltage_7, params.voltage_8, params.voltage_9 );
  CAN_register_fill_uint16x3( &LowVoltage10_12, params.voltage_10, params.voltage_11, params.voltage_12 );
  CAN_register_fill_uint16x3( &LowVoltage13_15, params.voltage_13, params.voltage_14, params.voltage_15 );
  CAN_register_fill_uint16x3( &LowVoltage16_18, params.voltage_16, params.voltage_17, params.voltage_18 );
  CAN_register_fill_uint16x3( &LowVoltage19_21, params.voltage_19, params.voltage_20, params.voltage_21 );
  CAN_register_fill_uint16x3( &LowVoltage22_24, params.voltage_22, params.voltage_23, params.voltage_24 );

  // ------------- LowVoltageMinMaxDelta
  uint16_t min_voltage = get_BMS_min_voltage( &params );
  uint16_t max_voltage = get_BMS_max_voltage( &params );
  CAN_register_fill_uint16x3( &LowVoltageMinMaxDelta, min_voltage, max_voltage, max_voltage-min_voltage );

  // ------------- Temperature1: MOS, Balancer, Temp1, Temp2, Temp3, Temp4, Temp5
  memset( data, 0, 7 );
  data[0] = (params.temp_mos & 0xFF);
  data[1] = (params.temp_bal & 0xFF);
  data[2] = (params.temp_1 & 0xFF);
  data[3] = (params.temp_1 & 0xFF);
  data[4] = (params.temp_1 & 0xFF);
  data[5] = (params.temp_1 & 0xFF);
  data[6] = 0;  // TODO: here should be additional temperature
  CAN_register_fill_data( &Temperature1, sizeof(uint8_t)*7, data );

  // ------------- MaxTemperature
  uint8_t max_temperature = (get_BMS_max_temperature( &params ) & 0xFF);
  CAN_register_fill_data( &MaxTemperature, sizeof(uint8_t), &max_temperature );
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t status;
	uint8_t dt[8];
	uint16_t raw_temper;
	float temper;
	char c;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim1);
	
	// установить начальные параметры по умолчанию (для отладки)
	HighVoltage.state = 0x01;				// Статус действия = отравка по таймеру
	HighVoltage.period_ms = 1000;		// Период отправки сообшений в ms.
	HighVoltage.length = 5;					// Длина данных + 1 байт type
	HighVoltage.data[0] = 0x61;			// Тип сообшения 0x61 - событие по таймеру, актуальное "нормальное" значение. Взято из таблицы.
	// 0x000124F8 = 75000 mV
	HighVoltage.data[1] = 0x00;
	HighVoltage.data[2] = 0x01;
	HighVoltage.data[3] = 0x24;
	HighVoltage.data[4] = 0xF8;
	
	HighCurrent.state = 0x01;				
	HighCurrent.period_ms = 1000;		
	HighCurrent.length = 5;					
	HighCurrent.data[0] = 0x61;			
	// 0x00001388 =  5000 mA
	HighCurrent.data[1] = 0x00;
	HighCurrent.data[2] = 0x00;
	HighCurrent.data[3] = 0x13;
	HighCurrent.data[4] = 0x88;
	
	MaxTemperature.state = 0x01;				
	MaxTemperature.period_ms = 5000;	
	MaxTemperature.length = 3;			
	MaxTemperature.data[0] = 0x61;	
	// 0x19 =  25
	MaxTemperature.data[1] = 0x19;
  MaxTemperature.data[2] = ThresholdTemperature1;   // пороговое значение предельной температуры

	Temperature1.length = 8;			
	Temperature2.length = 8;		
  // прочитать сохраненные параметры из Flash
	readEeprom(21);
	
	/* 
		Заполняем структуру отвечающую за отправку кадров
		StdId — это идентификатор стандартного кадра.
		ExtId — это идентификатор расширенного кадра. Мы будем отправлять стандартный поэтому сюда пишем 0.
		RTR = 
        CAN_RTR_DATA    — отправляем кадр с данными (Data Frame)
        CAN_RTR_REMOTE  — отправляем Remote Frame.
		IDE =
        CAN_ID_STD — отправляем стандартный кадр.
        CAN_ID_EXT — расширенный кадр. В этом случае в StdId нужно будет указать 0,
                     а в ExtId записать расширенный идентификатор.
		DLC = 8 — количество полезных байт передаваемых в кадре (от 1 до 8).
		TransmitGlobalTime — относится к Time Triggered Communication Mode, мы это не используем поэтому пишем 0.
	*/
	TxHeader.StdId = 0x07B0;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	/* активируем события которые будут вызывать прерывания  */
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	HAL_CAN_Start(&hcan);

	/* активируем прерывания USART3*/
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t*) receiveBuff_huart3, UART3_BUFF_SIZE);
	STBY_L();				// MCP2562 STBY mode = normal
	
// опросить датчики ds18b20
	port_init();
	status = ds18b20_init(NO_SKIP_ROM);
	sprintf(str1,"Init Status: %d\r\n",status);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	sprintf(str1,"Dev count: %d\r\n", Dev_Cnt);
	HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	for(uint8_t i=1;i<=Dev_Cnt;i++)
	{
		sprintf(str1,"Device %d\r\n", i);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		sprintf(str1,"ROM RAW: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
			Dev_ID[i-1][0], Dev_ID[i-1][1], Dev_ID[i-1][2], Dev_ID[i-1][3],
			Dev_ID[i-1][4], Dev_ID[i-1][5], Dev_ID[i-1][6], Dev_ID[i-1][7]);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		sprintf(str1,"Family CODE: 0x%02X\r\n", Dev_ID[i-1][0]);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		sprintf(str1,"ROM CODE: 0x%02X%02X%02X%02X%02X%02X\r\n", Dev_ID[i-1][6], Dev_ID[i-1][5],
			Dev_ID[i-1][4], Dev_ID[i-1][3], Dev_ID[i-1][2], Dev_ID[i-1][1]);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
		sprintf(str1,"CRC: 0x%02X\r\n", Dev_ID[i-1][7]);
		HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	}

  // запустить в цикле опрос 10 каналов ADC через DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_value, 10);			

	// записать значения таймера
  CAN_register_update_timer( &HighVoltage );
  CAN_register_update_timer( &HighCurrent );
  CAN_register_update_timer( &MaxTemperature );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    if(FlagReciveUART3 == 1){
      parse_CAN_registers_from_BMS_data();
      FlagReciveUART3 = 0;
    }

		if((HAL_GetTick() - HighVoltage.current_timer) > HighVoltage.period_ms && HighVoltage.state == 0x01){
			HighVoltage.current_timer = HAL_GetTick();
			TxData[0] = 0x61;				// Тип сообшения 0x61 - событие по таймеру, актуальное "нормальное" значение.
			HAL_CAN_Send_Obj(&HighVoltage);
		}
		if((HAL_GetTick() - HighCurrent.current_timer) > HighCurrent.period_ms && HighCurrent.state == 0x01) {
			HighCurrent.current_timer = HAL_GetTick();
			TxData[0] = 0x61;				
			HAL_CAN_Send_Obj(&HighCurrent);
		}
		if((HAL_GetTick() - MaxTemperature.current_timer) > MaxTemperature.period_ms && MaxTemperature.state == 0x01){
			MaxTemperature.current_timer = HAL_GetTick();
			MaxTemperature.data[1]=0;
			readADC(); // прочитать все каналы ADC

			// прочитать все датчики DS18B20
			for(uint8_t i=1;i<=Dev_Cnt;i++)
			{
				ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
			}
			for(uint8_t i=1;i<=Dev_Cnt;i++)
			{
				ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
				sprintf(str1,"STRATHPAD %d: %02X %02X %02X %02X %02X %02X %02X %02X; ",
					i, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7]);
				HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
				raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
				if(ds18b20_GetSign(raw_temper)) c='-';
				else c='+';
				temper = ds18b20_Convert(raw_temper);
				sprintf(str1,"Raw t: 0x%04X; t: %c%.2f\r\n", raw_temper, c, temper);
				HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
				
				Temperature3.data[i] = ((raw_temper&0x07FF)>>4);
			}
			Temperature3.length = Dev_Cnt+1;
			
      // Вычислить максимальную температуру, и критическую c датчиков ds18b20
      for(uint8_t i=1;i<Dev_Cnt+1;i++){
        if(Temperature3.data[i]>MaxTemperature.data[1])
        MaxTemperature.data[1] = Temperature3.data[i];
      }
       if(MaxTemperature.data[1]>MaxTemperature.data[2]){   // если температура больше чем пороговое значение
        TxData[0] = 0x63;			// Тип сообшения 0x63 - событие по таймеру, актуальное "критическое" значение
      }
      else{
        TxData[0] = 0x61;			// Тип сообшения 0x61 - событие по таймеру, актуальное "нормальное" значение.
      }

			HAL_CAN_Send_Obj(&MaxTemperature);
		}

		


		if(Button1 == 0){
			TxHeader.StdId = 0x07B0;
			TxHeader.DLC = 8;
			TxData[0] = 0x44;
			TxData[1] = 0x53;
			TxData[2] = 0x46;
			TxData[3] = 0x30;
			TxData[4] = 0x30;
			TxData[5] = 0x30;
			TxData[6] = 0x31;
			TxData[7] = 0x00;
			HAL_CAN_Send();
		}
		if(Button2 == 0){
			TxHeader.StdId = 0x07B0;
			TxHeader.DLC = 8;
			TxData[0] = 0x44;
			TxData[1] = 0x53;
			TxData[2] = 0x46;
			TxData[3] = 0x30;
			TxData[4] = 0x30;
			TxData[5] = 0x30;
			TxData[6] = 0x32;
			TxData[7] = 0x00;
			HAL_CAN_Send();
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB3 PB4
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
	{
		IRQHandlerTIM1();
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
