#include "BMS_low_level_abstraction.h"

/*******************************************************************************************\
 *
 * BMS raw packet data CRC calculation.
 *
\*******************************************************************************************/
uint16_t bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    uint16_t result = 0x0000;

    for (uint8_t i = sizeof(BMS_PACKET_HEADER); i < BMS_BOARD_PACKET_SIZE - 2; ++i)
    {
        result += *(bms_packet_data + i);
    }

    return result;
}

/*******************************************************************************************\
 *
 * BMS raw packet data CRC extraction
 *
\*******************************************************************************************/
uint16_t get_bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    return *(uint16_t *)&bms_packet_data[BMS_BOARD_PACKET_SIZE - 2];
}

/*******************************************************************************************\
 *
 * BMS raw packet data CRC validation.
 *
\*******************************************************************************************/
bool bms_raw_data_validation(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    if (bms_packet_data == nullptr)
        return false;

    return get_bms_raw_data_crc(bms_packet_data) == bms_raw_data_crc(bms_packet_data);
}

/*******************************************************************************************\
 *
 * Function updates max temperature value according to the temperatures of sensors.
 *
\*******************************************************************************************/
void update_max_temperature(bms_can_data_t &bms_can_data)
{
    bms_can_data.max_temperature = INT8_MIN;
    if (bms_can_data.max_temperature < bms_can_data.bms_temperature.balancer)
        bms_can_data.max_temperature = bms_can_data.bms_temperature.balancer;
    if (bms_can_data.max_temperature < bms_can_data.bms_temperature.mosfet)
        bms_can_data.max_temperature = bms_can_data.bms_temperature.mosfet;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (bms_can_data.max_temperature < bms_can_data.bms_temperature.bms_sensor_arr[i])
            bms_can_data.max_temperature = bms_can_data.bms_temperature.bms_sensor_arr[i];
    }
    // assume that other temperatures already placed in the bms_can_data
    for (uint8_t i = 0; i < 15; i++)
    {
        if (bms_can_data.max_temperature < bms_can_data.other_temperature.temp_5_19[i])
            bms_can_data.max_temperature = bms_can_data.other_temperature.temp_5_19[i];
    }
}

/*******************************************************************************************\
 *
 * Function converts BMS packet (UART byte array) to the BMS can data structure.
 * Only used parts of byte array are converted from big endian to little endian.
 *
\*******************************************************************************************/
void convert_bms_data_from_uart_to_can_structure(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE], bms_can_data_t &bms_can_data)
{
    uint32_t *BMS_header = (uint32_t *)bms_packet_data;
    if (*BMS_header != BMS_PACKET_HEADER)
    {
        LOG("ERROR: BMS header error! Expectations: 0x%08X, reality: 0x%08lX", BMS_PACKET_HEADER, *BMS_header);
        return;
    }

    if (!bms_raw_data_validation(bms_packet_data))
    {
        LOG("ERROR: BMS CRC error! Expectations: 0x%04X, reality: 0x%04X", bms_raw_data_crc(bms_packet_data), get_bms_raw_data_crc(bms_packet_data));
        return;
    }

    packet_structure_t *bms_packet_struct = (packet_structure_t *)bms_packet_data;

    // TODO: This blocks are not described yet. Should be implemented later.
    // bms_can_data.block_cfg
    // bms_can_data.block_error
    // bms_can_data.block_info
    swap_endian(bms_packet_struct->current);
    bms_can_data.block_health.current = bms_packet_struct->current;

    swap_endian(bms_packet_struct->vmin_voltage);
    bms_can_data.bms_low_voltage_min_max_delta.min = bms_packet_struct->vmin_voltage;
    swap_endian(bms_packet_struct->vmax_voltage);
    bms_can_data.bms_low_voltage_min_max_delta.max = bms_packet_struct->vmax_voltage;
    bms_can_data.bms_low_voltage_min_max_delta.delta = bms_can_data.bms_low_voltage_min_max_delta.max - bms_can_data.bms_low_voltage_min_max_delta.min;

    swap_endian(bms_packet_struct->temperature_balancer);
    bms_can_data.bms_temperature.balancer = bms_packet_struct->temperature_balancer;
    swap_endian(bms_packet_struct->temperature_mosfet);
    bms_can_data.bms_temperature.mosfet = bms_packet_struct->temperature_mosfet;
    for (uint8_t i = 0; i < 4; i++)
    {
        swap_endian(bms_packet_struct->temperature_mosfet);
        bms_can_data.bms_temperature.bms_sensor_arr[i] = bms_packet_struct->temperature_sensors[i];
    }

    bms_can_data.high_current = bms_packet_struct->current;
    bms_can_data.high_voltage = bms_packet_struct->voltage * 10; // BMS reports voltage in x10 mV

    for (uint8_t i = 0; i < BMS_BATTERY_NUMBER_OF_CELLS; i++)
    {
        bms_can_data.low_voltage.cells_voltage[i] = bms_packet_struct->cells_voltage[i];
    }

    update_max_temperature(bms_can_data);
}

/*******************************************************************************************\
 *
 * Function performs full initialization of CANManager object for BMS device.
 *
\*******************************************************************************************/
bool init_can_manager_for_bms(CANManager &cm, bms_can_data_t &bms_can_data)
{
    CANObject *co = nullptr;

    // 0x0040	BlockInfo
    // request | timer:15000	byte	1 + 7	{ type[0] data[1..7] }
    // Основная информация о блоке. См. "Системные параметры".
    init_block_info(cm, BMS_CANO_ID_BLOCK_INFO, bms_can_data.block_info);

    // 0x0041	BlockHealth
    // request | event	UINT32	1 + 7	{ type[0] data[1..7] }
    // Информация о здоровье блока. См. "Системные параметры".
    init_block_health(cm, BMS_CANO_ID_BLOCK_HEALTH, bms_can_data.block_health);

    // 0x0042	BlockCfg
    // request	byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
    // Чтение и запись настроек блока. См. "Системные параметры".
    init_block_cfg(cm, BMS_CANO_ID_BLOCK_CFG, bms_can_data.block_cfg);

    // 0x0043	BlockError
    // request | event	byte	1 + X	{ type[0] data[1..7] }
    // Ошибки блока. См. "Системные параметры".
    init_block_error(cm, BMS_CANO_ID_BLOCK_ERROR, bms_can_data.block_error);

    // 0x0044	HighVoltage
    // request | timer:1000	uint32_t	1 + 4	{ type[0] data[1..4] }
    // Общее напряжение АКБ, мВ.
    co = cm.add_can_object(BMS_CANO_ID_HIGH_VOLTAGE, "HighVoltage");
    co->add_data_field(DF_UINT32, &bms_can_data.high_voltage);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 1000);
    
    // 0x0045	HighCurrent
    // request | timer:1000	int32_t	1 + 4	{ type[0] data[1..4] }
    // Общий ток разряда / заряда АКБ, мА.
    co = cm.add_can_object(BMS_CANO_ID_HIGH_CURRENT, "HighCurrent");
    co->add_data_field(DF_INT32, &bms_can_data.high_current);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 1000);

    // 0x0046	MaxTemperature
    // request | timer:5000 | event	int8_t	1 + 1	{ type[0] data[1] }
    // Максимально зафиксированная температура.
    co = cm.add_can_object(BMS_CANO_ID_MAX_TEMPERATURE, "MaxTemperature");
    co->add_data_field(DF_INT8, &bms_can_data.max_temperature);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 5000);

    // 0x0047	LowVoltageMinMaxDelta
    // request | event	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: Минимальное, Максимальное, Дельта.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_MIN_MAX_DELTA, "LowVoltageMinMaxDelta");
    co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.min);
    co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.max);
    co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.delta);
    co->add_function(CAN_FUNC_REQUEST_IN);
    co->add_function(CAN_FUNC_EVENT_ERROR);

    // 0x0048	Temperature1
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: MOS, Balancer, Temp1, Temp2, Temp3, Temp4, Temp5.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_1, "Temperature1");
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.mosfet);
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.balancer);
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor1);
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor2);
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor3);
    co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor4);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp5);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0049	Temperature2
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: Temp6, Temp7, Temp8, Temp9, Temp10, Temp11, Temp12.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_2, "Temperature2");
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp6);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp7);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp8);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp9);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp10);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp11);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp12);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004A	Temperature3
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: Temp13, Temp14, Temp15, Temp16, Temp17, Temp18, Temp19.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_3, "Temperature3");
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp13);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp14);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp15);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp16);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp17);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp18);
    co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp19);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004B	LowVoltage1-3
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 1, 2, 3.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_1_3, "LowVoltage1-3");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_1);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_2);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_3);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004C	LowVoltage4-6
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 4, 5, 6.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_4_6, "LowVoltage4-6");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_4);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_5);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_6);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004D	LowVoltage7-9
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 7, 8, 9.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_7_9, "LowVoltage7-9");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_7);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_8);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_9);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004E	LowVoltage10-12
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 10, 11, 12.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_10_12, "LowVoltage10-12");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_10);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_11);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_12);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004F	LowVoltage13-15
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 13, 14, 15.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_13_15, "LowVoltage13-15");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_13);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_14);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_15);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0050	LowVoltage16-18
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 16, 17, 18.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_16_18, "LowVoltage16-18");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_16);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_17);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_18);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0051	LowVoltage19-21
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 19, 20, 21.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_19_21, "LowVoltage19-21");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_19);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_20);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_21);
    co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0052	LowVoltage22-24
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 22, 23, 24.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_22_24, "LowVoltage22-24");
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_22);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_23);
    co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_24);
    co->add_function(CAN_FUNC_REQUEST_IN);

    return true;
}