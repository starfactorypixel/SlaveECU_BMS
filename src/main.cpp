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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Button1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define Button2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define STBY_H() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // High
#define STBY_L() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Low

#define MARKER_FIRST_START 100 // Маркер что flash не пустая

// BMS_PACKET_SIZE - размер структуры params_t (140 байт на текущий момент)
// 60 - просто запас... на самом деле не особо нужен, наверное
// изначально размер буфера был 200 байт, оттуда запас 60 и вылез
#define UART3_BUFF_SIZE (BMS_BOARD_PACKET_SIZE + 60)

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

uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE] = {0};
bms_can_data_t bms_can_data;
CANManager can_manager(&HAL_GetTick);

//------------------------  Ds18b20
uint8_t Dev_ID[8][8] = {0};
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
// uint8_t receiveBuffStat_huart3[UART3_BUFF_SIZE];
// uint16_t ReciveUartSize = 0;  // used nowhere
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
void HAL_CAN_Send(CANFrame *can_frame = nullptr);
// void HAL_CAN_Send_Obj(_params_v *params_obj);
void write_flash();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------- Прерывание от USART3 по флагу Idle
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART3)
    return;

  // Check BMS packet header
  uint32_t *BMS_header = (uint32_t *)receiveBuff_huart3;
  if (*BMS_header == BMS_PACKET_HEADER && Size >= BMS_BOARD_PACKET_SIZE)
  {
    // ReciveUartSize = Size; // пока не понял, зачем оно нам тут

    // set flag that BMS packet received
    FlagReciveUART3 = 1;

    // fill the BMS structure with data
    memcpy(&bms_packet_data, receiveBuff_huart3, BMS_BOARD_PACKET_SIZE);
  }

  HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)receiveBuff_huart3, UART3_BUFF_SIZE);
}

//-------------------------------- Прерывание от таймера TIM1
void IRQHandlerTIM1(void)
{
}

/*
  Колбек для приёма данных CAN (для буфера RX_FIFO_1)
  При приёме любого кадра тут же забираем его из почтового ящика с помощью функции HAL_CAN_GetRxMessage(...)
  ВАЖНО!!!!!!  для приема нужно в MX_CAN_Init() прописать настройки фильтра
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8] = {0};
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
  {
    can_manager.take_new_rx_frame(RxHeader, RxData);

#ifdef DEBUG
    sprintf(str1, "CAN 0x%04lX", RxHeader.StdId);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 100);
#endif
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t er = HAL_CAN_GetError(hcan);
  sprintf(str1, "ER CAN %lu %08lX", (unsigned long)er, (unsigned long)er);
  HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 100);
}

void HAL_CAN_Send(CANFrame *can_frame)
{
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
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = {0};
  uint32_t TxMailbox = 0;
  TxHeader.StdId = UINT32_MAX;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    ;

  if (can_manager.has_tx_frames_for_transmission())
  {
    can_manager.give_tx_frame(TxHeader, TxData);
  }

  if (can_frame != nullptr)
  {
    TxHeader.StdId = can_frame->get_id();
    TxHeader.DLC = can_frame->get_data_length();
    can_frame->copy_frame_data_to(TxData, 8);
  }

  if (TxHeader.StdId != UINT32_MAX)
  {
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
#ifdef DEBUG
      sprintf(str1, "CAN 0x%04lX", TxHeader.StdId);
      HAL_UART_Transmit(&huart1, (uint8_t *)"ER SEND\n", 8, 100);
#endif
    }
  }
}

uint8_t ADCtoTEMPER(uint16_t adc_val)
{
  uint8_t result;
  result = ((adc_val & 0x07FF) >> 6);
  // result = adc_val/256;
  return result;
}

void readADC(void)
{
  for (uint8_t i = 0; i < 10; i++)
  {
    ADC_senors[i] = ADC_value[i];
    sprintf(str1, "ADC %d =", i);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, " %d \r\n", ADC_senors[i]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  }
  for (uint8_t i = 0; i < 10; i++)
  {
    bms_can_data.other_temperature.temp_5_19[i] = ADCtoTEMPER(ADC_senors[i]);
  }

  convert_bms_data_from_uart_to_can_structure(bms_packet_data, bms_can_data);
}

void read_ds18b20()
{
  // TODO: We should carefully check the ds18b20 temperature conversion
  // I really didn't figure out are this temperature conversions correct or not
  // Probably this conversion loses sign of temperature...
  // In ideal world we should rewrite all ds18b20 related code into OOP paradigm

  uint8_t dt[8];
  uint16_t raw_temper;
  float temper;
  char c;

  // TODO: Dev_Cnt filled in the ds18b20,cpp as extern variable
  // Should rewrite this
  for (uint8_t i = 1; i <= Dev_Cnt; i++)
  {
    ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
  }
  for (uint8_t i = 1; i <= Dev_Cnt; i++)
  {
    ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
    sprintf(str1, "STRATHPAD %d: %02X %02X %02X %02X %02X %02X %02X %02X; ",
            i, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    raw_temper = ((uint16_t)dt[1] << 8) | dt[0];
    if (ds18b20_GetSign(raw_temper))
      c = '-';
    else
      c = '+';
    temper = ds18b20_Convert(raw_temper);

    sprintf(str1, "Raw t: 0x%04X; t: %c%.2f\r\n", raw_temper, c, temper);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);

    // temp_5_19[i] for i=0..9 is reserved for 10 ADC values
    if (i+9 < 15)
    {
      bms_can_data.other_temperature.temp_5_19[i+9] = ((raw_temper & 0x07FF) >> 4);
    }
  }

  convert_bms_data_from_uart_to_can_structure(bms_packet_data, bms_can_data);
}

void write_flash()
{
  // TODO: Выяснить, что это и зачем. Пока в CAN Manager не реализовано
  /*
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
  */
}

void readEeprom(uint8_t num)
{
  // TODO: Выяснить, что это и зачем. Пока в CAN Manager не реализовано
  /*
  Read_flash_16b(num);

  // Обработка ситуации чистой EEPROM памяти
  // Если в flash нет данных, то заполнить и устанавливить маркер, указывающий, EEPROM инициализировано
  if (read_data16[0] != MARKER_FIRST_START)
  {
    BlockInfo.state = 0x00;
    BlockHealth.state = 0x00;
    BlockCfg.state = 0x00;
    BlockError.state = 0x00;
    HighVoltage.state = 0x01; //  отравка по таймеру
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
    MaxTemperature.data[2] = ThresholdTemperature1; // пороговое значение предельной температуры

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
  */
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
  
  /*
  uint8_t dt[8];
  uint16_t raw_temper;
  float temper;
  char c;
  */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  memset(&bms_can_data, 0, sizeof(bms_can_data_t));
  init_can_manager_for_bms(can_manager, bms_can_data);
  
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

  // TODO: вроде как в таблице никаких пороговых значений температуры нет...
  //MaxTemperature.data[2] = ThresholdTemperature1; // пороговое значение предельной температуры

  // прочитать сохраненные параметры из Flash
  readEeprom(21);

  /* активируем события которые будут вызывать прерывания  */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
  HAL_CAN_Start(&hcan);

  /* активируем прерывания USART3*/
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)receiveBuff_huart3, UART3_BUFF_SIZE);
  STBY_L(); // MCP2562 STBY mode = normal

  // опросить датчики ds18b20
  port_init();
  status = ds18b20_init(NO_SKIP_ROM);
  sprintf(str1, "Init Status: %d\r\n", status);
  HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  sprintf(str1, "Dev count: %d\r\n", Dev_Cnt);
  HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  for (uint8_t i = 1; i <= Dev_Cnt; i++)
  {
    sprintf(str1, "Device %d\r\n", i);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "ROM RAW: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            Dev_ID[i - 1][0], Dev_ID[i - 1][1], Dev_ID[i - 1][2], Dev_ID[i - 1][3],
            Dev_ID[i - 1][4], Dev_ID[i - 1][5], Dev_ID[i - 1][6], Dev_ID[i - 1][7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "Family CODE: 0x%02X\r\n", Dev_ID[i - 1][0]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "ROM CODE: 0x%02X%02X%02X%02X%02X%02X\r\n", Dev_ID[i - 1][6], Dev_ID[i - 1][5],
            Dev_ID[i - 1][4], Dev_ID[i - 1][3], Dev_ID[i - 1][2], Dev_ID[i - 1][1]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
    sprintf(str1, "CRC: 0x%02X\r\n", Dev_ID[i - 1][7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)str1, strlen(str1), 0x1000);
  }

  // запустить в цикле опрос 10 каналов ADC через DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_value, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_tick1 = HAL_GetTick();
  uint32_t last_tick2 = HAL_GetTick();
  CANFrame can_frame;
  uint8_t can_frame_data[8];
  while (1)
  {
    if (FlagReciveUART3 == 1)
    {
      convert_bms_data_from_uart_to_can_structure(bms_packet_data, bms_can_data);
      FlagReciveUART3 = 0;
    }

    // CAN Manager checks data every 300 ms
    if (HAL_GetTick() - last_tick1 > 300)
    {
      can_manager.process();
      if (can_manager.has_tx_frames_for_transmission())
        HAL_CAN_Send();

      last_tick1 = HAL_GetTick();
    }

    // Perform ADC reading with 1 sec period
    if (HAL_GetTick() - last_tick2 > 1000)
    {
      readADC(); // read 10 ADC channels
      read_ds18b20(); // read all DS18B20 temperature sensors
      convert_bms_data_from_uart_to_can_structure(bms_packet_data, bms_can_data);
      last_tick2 = HAL_GetTick();
    }

    if (Button1 == 0)
    {
      can_frame_data[0] = 0x44;
      can_frame_data[1] = 0x53;
      can_frame_data[2] = 0x46;
      can_frame_data[3] = 0x30;
      can_frame_data[4] = 0x30;
      can_frame_data[5] = 0x30;
      can_frame_data[6] = 0x31;
      can_frame_data[7] = 0x00;
      can_frame.set_frame(0x07B0, can_frame_data, 8);
    }
    if (Button2 == 0)
    {
      can_frame_data[0] = 0x44;
      can_frame_data[1] = 0x53;
      can_frame_data[2] = 0x46;
      can_frame_data[3] = 0x30;
      can_frame_data[4] = 0x30;
      can_frame_data[5] = 0x30;
      can_frame_data[6] = 0x32;
      can_frame_data[7] = 0x00;
      can_frame.set_frame(0x07B0, can_frame_data, 8);
    }
    if (can_frame.is_initialized())
    {
      HAL_CAN_Send(&can_frame);
      can_frame.clear_frame();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  CAN_FilterTypeDef sFilterConfig;
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
  // sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
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
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
static void MX_USART3_UART8b_Init(void)
{
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
}
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
  if (htim->Instance == TIM1) // check if the interrupt comes from TIM1
  {
    IRQHandlerTIM1();
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
