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

#include "main.h"
#include "ds18b20.h"
#include "BMS_low_level_abstraction.h"
#include <ConstantLibrary.h>
#include <LoggerLibrary.h>
#include <About.h>
#include <Leds.h>
#include <CANLogic.h>
#include <BMSLogic.h>

#define MARKER_FIRST_START 100 // Маркер что flash не пустая

#define BMS_BATTERY_NUMBER_OF_CELLS 32 // Number of cells in BMS packet
#define BMS_PACKET_HEADER 0xAA55AAFF   // from SlaveECU github

// BMS_PACKET_SIZE - размер структуры params_t (140 байт на текущий момент)
// 60 - просто запас... на самом деле не особо нужен, наверное
// изначально размер буфера был 200 байт, оттуда запас 60 и вылез
#define UART3_BUFF_SIZE (BMS_BOARD_PACKET_SIZE + 60)

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef hDebugUart;
UART_HandleTypeDef hBmsUart;

//------------------------  Ds18b20
#define MAX_DS18B20_COUNT 8 // TODO: почему 8?! Вроде по описанию в гуглотаблице максимум 6 должно быть...
uint8_t Dev_ID[MAX_DS18B20_COUNT][8] = {0};
uint8_t Dev_Cnt;
//------------------------  ADC
#define ADC_CHANNEL_COUNT 10
uint16_t ADC_value[ADC_CHANNEL_COUNT];
uint8_t ADC_cnt_state = 0;
uint16_t ADC_cnt = 0;
uint16_t ADC_senors[ADC_CHANNEL_COUNT];

//------------------------ UART
uint8_t receiveBuff_hBmsUart[UART3_BUFF_SIZE];
uint8_t FlagReciveUART3 = 0;
uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE] = {0};

// collected sensor's temperatures
//    t[0]..t[ADC_CHANNEL_COUNT-1] - external temperature sensors (ADC)
//    t[ADC_CHANNEL_COUNT]..t[max] - external temperature sensors (ds18b20)
int8_t temperatures[ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

//-------------------------------- Прерывание от USART3 по флагу Idle
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3)
        return;

    // Check BMS packet header
    //uint32_t *BMS_header = (uint32_t *)receiveBuff_hBmsUart;
    //if (*BMS_header == BMS_PACKET_HEADER && Size >= BMS_BOARD_PACKET_SIZE)
    {
        // fill the BMS structure with data
        memcpy(&bms_packet_data, receiveBuff_hBmsUart, BMS_BOARD_PACKET_SIZE);

        // set flag that BMS packet received
        FlagReciveUART3 = 1;
    }

    HAL_UARTEx_ReceiveToIdle_IT(&hBmsUart, (uint8_t *)receiveBuff_hBmsUart, UART3_BUFF_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        DEBUG_LOG_TOPIC("uart3", "ERR: %d\r\n", huart->ErrorCode);

        HAL_UART_AbortReceive_IT(&hBmsUart);
        HAL_UARTEx_ReceiveToIdle_IT(&hBmsUart, (uint8_t *)receiveBuff_hBmsUart, UART3_BUFF_SIZE);
    }
}

//-------------------------------- Прерывание от таймера TIM1
void IRQHandlerTIM1(void)
{
}

/// @brief Callback function of CAN receiver.
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
        // DEBUG_LOG("RX: CAN 0x%04lX", RxHeader.StdId);
    }
}

/// @brief Callback function for CAN error handler
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    DEBUG_LOG("CAN ERROR: %lu %08lX", hcan->ErrorCode, hcan->ErrorCode);
}

/// @brief Sends data via CAN bus
/// @param id CANObject ID
/// @param data Pointer to the CAN frame data buffer (8 bytes max)
/// @param length Length of the CAN frame data buffer
void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;

    TxHeader.StdId = id;                   // Standard frame ID (sets to 0 if extended one used)
    TxHeader.ExtId = 0;                    // Extended frame ID (sets to 0 if standard one used)
    TxHeader.RTR = CAN_RTR_DATA;           // CAN_RTR_DATA: CAN frame with data will be sent
                                           // CAN_RTR_REMOTE: remote CAN frame will be sent
    TxHeader.IDE = CAN_ID_STD;             // CAN_ID_STD: CAN frame with standard ID
                                           // CAN_ID_EXT: CAN frame with extended ID
    TxHeader.DLC = length;                 // Data length of the CAN frame
    TxHeader.TransmitGlobalTime = DISABLE; // Time Triggered Communication Mode

    memcpy(TxData, data, length);

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        Leds::obj.SetOn(Leds::LED_YELLOW);
    }
    Leds::obj.SetOff(Leds::LED_YELLOW);

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        DEBUG_LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}

inline int8_t ADCtoTEMPER(uint16_t adc_val)
{
    return ((adc_val & 0x07FF) >> 6);
}

/// @brief Reads temperature from ADC sensors
void readADC()
{
    // int8_t temperatures[ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT];
    //    t[0]..t[ADC_CHANNEL_COUNT-1] - external temperature sensors (ADC)
    //    t[ADC_CHANNEL_COUNT]..t[max] - external temperature sensors (ds18b20)

    memcpy(ADC_senors, ADC_value, ADC_CHANNEL_COUNT);

    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        temperatures[i] = ADCtoTEMPER(ADC_senors[i - 6]);
    }
}

/// @brief
void read_ds18b20()
{
    // TODO: We should carefully check the ds18b20 temperature conversion
    // I really didn't figure out are this temperature conversions correct or not
    // Probably this conversion loses sign of temperature...
    // In ideal world we should rewrite all ds18b20 related code into OOP paradigm

    uint8_t dt[8];
    uint16_t raw_temper;
    float temper;

    // TODO: Dev_Cnt filled in the ds18b20,cpp as extern variable
    // Should rewrite this
    for (uint8_t i = 1; i <= Dev_Cnt; i++)
    {
        ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
    }
    for (uint8_t i = 1; i <= Dev_Cnt; i++)
    {
        ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
        DEBUG_LOG_TOPIC("DS18b", "STRATHPAD %d: %02X %02X %02X %02X %02X %02X %02X %02X\n",
               i, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7]);

        raw_temper = ((uint16_t)dt[1] << 8) | dt[0];
        temper = ds18b20_Convert(raw_temper);

        DEBUG_LOG_TOPIC("DS18b", "Raw t: 0x%04X; t: %s%.2f\n", raw_temper, (ds18b20_GetSign(raw_temper)) ? "-" : "+", temper);

        // int8_t temperatures[ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT];
        //    t[0]..t[ADC_CHANNEL_COUNT-1] - external temperature sensors (ADC)
        //    t[ADC_CHANNEL_COUNT]..t[max] - external temperature sensors (ds18b20)
        if (i - 1 < MAX_DS18B20_COUNT)
        {
            temperatures[ADC_CHANNEL_COUNT + i - 1] = (int8_t)(temper);
        }
    }
}

/// @brief Peripherals initialization: GPIO, DMA, CAN, SPI, USART, ADC, Timers
void InitPeripherals()
{
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    HAL_TIM_Base_Start_IT(&htim1);
};

/// @brief Initialization of DS18B20 digital termometers
void InitDS18B20()
{
    port_init();
    DEBUG_LOG_TOPIC("DS18b", "Init Status: %d\n", ds18b20_init(NO_SKIP_ROM));
    DEBUG_LOG_TOPIC("DS18b", "Dev count: %d\n", Dev_Cnt);
    for (uint8_t i = 1; i <= Dev_Cnt; i++)
    {
        DEBUG_LOG_TOPIC("DS18b", "Device %d\n", i);
        DEBUG_LOG_TOPIC("DS18b", "ROM RAW: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            Dev_ID[i - 1][0], Dev_ID[i - 1][1], Dev_ID[i - 1][2], Dev_ID[i - 1][3],
            Dev_ID[i - 1][4], Dev_ID[i - 1][5], Dev_ID[i - 1][6], Dev_ID[i - 1][7]);
        DEBUG_LOG_TOPIC("DS18b", "Family CODE: 0x%02X\n", Dev_ID[i - 1][0]);
        DEBUG_LOG_TOPIC("DS18b", "ROM CODE: 0x%02X%02X%02X%02X%02X%02X\n", Dev_ID[i - 1][6], Dev_ID[i - 1][5],
            Dev_ID[i - 1][4], Dev_ID[i - 1][3], Dev_ID[i - 1][2], Dev_ID[i - 1][1]);
        DEBUG_LOG_TOPIC("DS18b", "CRC: 0x%02X\n", Dev_ID[i - 1][7]);
    }
}

/// @brief Updates temperature data in temperatures[ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT]
void UpdateTemperatureData()
{
    // int8_t temperatures[ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT];
    //    t[0]..t[ADC_CHANNEL_COUNT-1] - external temperature sensors (ADC)
    //    t[ADC_CHANNEL_COUNT]..t[max] - external temperature sensors (ds18b20)
    readADC();
    read_ds18b20();
}

/// @brief  The application entry point.
/// @retval int
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    InitPeripherals();

    // Сразу после инициализации периферии, иначе программа упадёт, если попробовать включить диод.
    // Green LED:  lights up, when programm falls in the Error_Handler()
    // Blue LED:   is unused yet
    // Red LED:    is unused yet
    // Yellow LED: is on while free CAN mailboxes are not available.
    //             When at least one mailbox is free, LED will go off.
    About::Setup();
    Leds::Setup();

    /* активируем события которые будут вызывать прерывания  */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

    HAL_CAN_Start(&hcan);
    CANLib::Setup();

    /* активируем прерывания USART3*/
    HAL_UARTEx_ReceiveToIdle_IT(&hBmsUart, (uint8_t *)receiveBuff_hBmsUart, UART3_BUFF_SIZE);
    
    // CAN MCP2562
    // if we need normal CAN operation then STBY pin should be LOW
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // LOW
    // if we need switch MCP2562 into standby mode then STBY pin should be HIGH
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // HIGH

    InitDS18B20();

    // запустить в цикле опрос 10 каналов ADC через DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_value, ADC_CHANNEL_COUNT);

    uint32_t last_tick1 = HAL_GetTick();
    uint32_t current_time = HAL_GetTick();
    while (1)
    {
        if (FlagReciveUART3 == 1)
        {
			DEBUG_LOG_ARRAY_HEX("BMS2", bms_packet_data, sizeof(bms_packet_data));
            CANLib::UpdateCANObjects_BMS(bms_packet_data);
            FlagReciveUART3 = 0;
        }

        // Perform ADC & ds18b20 reading with 1 sec period
        if (current_time - last_tick1 > 1000)
        {
            UpdateTemperatureData();
            CANLib::UpdateCANObjects_ExternalTemperature(temperatures, ADC_CHANNEL_COUNT + MAX_DS18B20_COUNT);
            last_tick1 = current_time;
        }

        // don't need to update current_time because it is always updated by Loop() functions
        // current_time = HAL_GetTick();
        About::Loop(current_time);
        Leds::Loop(current_time);
        CANLib::Loop(current_time);
        BMSLogic::Loop(current_time);
    }
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
    ADC_ChannelConfTypeDef sConfig = {0};

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
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
    // https://istarik.ru/blog/stm32/159.html

    CAN_FilterTypeDef sFilterConfig;

    // CAN interface initialization
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL; // CAN_MODE_NORMAL
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;   // DISABLE
    hcan.Init.AutoBusOff = ENABLE;           // DISABLE
    hcan.Init.AutoWakeUp = ENABLE;           // DISABLE
    hcan.Init.AutoRetransmission = DISABLE;  // DISABLE
    hcan.Init.ReceiveFifoLocked = ENABLE;    // DISABLE
    hcan.Init.TransmitFifoPriority = ENABLE; // DISABLE
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN filtering initialization
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    hDebugUart.Instance = USART1;
    hDebugUart.Init.BaudRate = 500000;
    hDebugUart.Init.WordLength = UART_WORDLENGTH_8B;
    hDebugUart.Init.StopBits = UART_STOPBITS_1;
    hDebugUart.Init.Parity = UART_PARITY_NONE;
    hDebugUart.Init.Mode = UART_MODE_TX_RX;
    hDebugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hDebugUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hDebugUart) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    hBmsUart.Instance = USART3;
    hBmsUart.Init.BaudRate = 19200;
    hBmsUart.Init.WordLength = UART_WORDLENGTH_8B;
    hBmsUart.Init.StopBits = UART_STOPBITS_1;
    hBmsUart.Init.Parity = UART_PARITY_NONE;
    hBmsUart.Init.Mode = UART_MODE_TX_RX;
    hBmsUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hBmsUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hBmsUart) != HAL_OK)
    {
        Error_Handler();
    }
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
/*
static void MX_USART3_UART8b_Init(void)
{
    hBmsUart.Instance = USART3;
    hBmsUart.Init.BaudRate = 115200;
    hBmsUart.Init.WordLength = UART_WORDLENGTH_8B;
    hBmsUart.Init.StopBits = UART_STOPBITS_1;
    hBmsUart.Init.Parity = UART_PARITY_NONE;
    hBmsUart.Init.Mode = UART_MODE_TX_RX;
    hBmsUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hBmsUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hBmsUart) != HAL_OK)
    {
        Error_Handler();
    }
}
*/
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
    if (htim->Instance == TIM1) // check if the interrupt comes from TIM1
    {
        IRQHandlerTIM1();
    }
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    Leds::obj.SetOff(Leds::LED_GREEN);
    while (1)
    {
    }
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
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
