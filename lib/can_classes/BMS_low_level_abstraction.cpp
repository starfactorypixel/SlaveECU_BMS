#include "BMS_low_level_abstraction.h"

void convert_bms_data_from_uart_to_can_structure(uint8_t *bms_packet_data, bms_can_data_t &bms_can_data)
{
    packet_structure_t *bms_packet_struct = (packet_structure_t *)bms_packet_data;

    // bms_can_data.block_cfg
    // bms_can_data.block_error
    bms_can_data.block_health.current = bms_packet_struct->current;
    // bms_can_data.block_info

    bms_can_data.bms_low_voltage_min_max_delta.min = bms_packet_struct->vmin_voltage;
    bms_can_data.bms_low_voltage_min_max_delta.max = bms_packet_struct->vmax_voltage;
    bms_can_data.bms_low_voltage_min_max_delta.delta = bms_can_data.bms_low_voltage_min_max_delta.max - bms_can_data.bms_low_voltage_min_max_delta.min;

    bms_can_data.bms_temperature.balancer = bms_packet_struct->temperature_balancer;
    bms_can_data.bms_temperature.mosfet = bms_packet_struct->temperature_mosfet;
    bms_can_data.bms_temperature.bms_sensor1 = bms_packet_struct->temperature_sensors[0];
    bms_can_data.bms_temperature.bms_sensor2 = bms_packet_struct->temperature_sensors[1];
    bms_can_data.bms_temperature.bms_sensor3 = bms_packet_struct->temperature_sensors[2];
    bms_can_data.bms_temperature.bms_sensor4 = bms_packet_struct->temperature_sensors[3];

    bms_can_data.high_current = bms_packet_struct->current;
    bms_can_data.high_voltage = bms_packet_struct->voltage * 10; // BMS reports voltage in x10 mV

    for (uint8_t i = 0; i < BMS_BATTERY_NUMBER_OF_CELLS; i++)
    {
        bms_can_data.low_voltage.cells_voltage[i] = bms_packet_struct->cells_voltage[i];
    }

    // ************************************** MAX TEMPERATURE SETTER **************************************
    bms_can_data.max_temperature = INT8_MIN;
    if (bms_can_data.max_temperature < bms_can_data.bms_temperature.balancer)
        bms_can_data.max_temperature = bms_can_data.bms_temperature.balancer;
    if (bms_can_data.max_temperature < bms_can_data.bms_temperature.mosfet)
        bms_can_data.max_temperature = bms_can_data.bms_temperature.mosfet;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (bms_can_data.max_temperature < bms_packet_struct->temperature_sensors[i])
            bms_can_data.max_temperature = bms_packet_struct->temperature_sensors[i];
    }
    // assume that other temperature already placed in the bms_can_data
    for (uint8_t i = 0; i < 15; i++)
    {
        if (bms_can_data.max_temperature < bms_can_data.other_temperature.temp_5_19[i])
            bms_can_data.max_temperature = bms_can_data.other_temperature.temp_5_19[i];
    }
}

bool init_can_manager_for_bms(CANManager &cm, bms_can_data_t &bms_can_data)
{
    CANObject *co = nullptr;
    DataField *df = nullptr;
    CANFunctionTimerNormal *func_timer_norm = nullptr;
    CANFunctionRequest *func_request = nullptr;
    CANFunctionSimpleEvent *func_event = nullptr;
    data_mapper_t min_val = {0};
    data_mapper_t max_val = {0};

    // 0x0040	BlockInfo
    // request | timer:15000	byte	1 + 7	{ type[0] data[1..7] }
    // Основная информация о блоке. См. "Системные параметры".
    co = cm.add_can_object(BMS_CANO_ID_BLOCK_INFO, "BlockInfo");
    df = co->add_data_field(DF_UINT8, &bms_can_data.block_info.board_data_byte);
    df = co->add_data_field(DF_UINT8, &bms_can_data.block_info.software_data_byte);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);
    func_timer_norm = (CANFunctionTimerNormal *)co->add_function(CAN_FUNC_TIMER_NORMAL);
    func_timer_norm->set_period(15000);

    // 0x0041	BlockHealth
    // request | event	UINT32	1 + 7	{ type[0] data[1..7] }
    // Информация о здоровье блока. См. "Системные параметры".
    co = cm.add_can_object(BMS_CANO_ID_BLOCK_HEALTH, "BlockHealth");
    df = co->add_data_field(DF_UINT16, &bms_can_data.block_health.voltage);
    df = co->add_data_field(DF_INT16, &bms_can_data.block_health.current);
    min_val.i16 = -500;
    max_val.i16 = 500;
    df->set_alarm_checker(min_val, max_val);
    df = co->add_data_field(DF_INT8, &bms_can_data.block_health.temperature);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);
    func_event = (CANFunctionSimpleEvent *)co->add_function(CAN_FUNC_EVENT_ERROR);
    func_event->set_period(3000);

    // 0x0042	BlockCfg
    // request	byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
    // Чтение и запись настроек блока. См. "Системные параметры".
    co = cm.add_can_object(BMS_CANO_ID_BLOCK_CFG, "BlockCfg");
    // df = co->add_data_field(DF_UINT8, ??, ??);    // byte 1
    // df = co->add_data_field(DF_UINT8, ??, 6);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0043	BlockError
    // request | event	byte	1 + X	{ type[0] data[1..7] }
    // Ошибки блока. См. "Системные параметры".
    co = cm.add_can_object(BMS_CANO_ID_BLOCK_ERROR, "BlockError");
    df = co->add_data_field(DF_UINT8, &bms_can_data.block_error.code);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0044	HighVoltage
    // request | timer:1000	uint32_t	1 + 4	{ type[0] data[1..4] }
    // Общее напряжение АКБ, мВ.
    co = cm.add_can_object(BMS_CANO_ID_HIGH_VOLTAGE, "HighVoltage");
    df = co->add_data_field(DF_UINT32, &bms_can_data.high_voltage);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);
    func_timer_norm = (CANFunctionTimerNormal *)co->add_function(CAN_FUNC_TIMER_NORMAL);
    func_timer_norm->set_period(1000);

    // 0x0045	HighCurrent
    // request | timer:1000	int32_t	1 + 4	{ type[0] data[1..4] }
    // Общий ток разряда / заряда АКБ, мА.
    co = cm.add_can_object(BMS_CANO_ID_HIGH_CURRENT, "HighCurrent");
    df = co->add_data_field(DF_INT32, &bms_can_data.high_current);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);
    func_timer_norm = (CANFunctionTimerNormal *)co->add_function(CAN_FUNC_TIMER_NORMAL);
    func_timer_norm->set_period(1000);

    // 0x0046	MaxTemperature
    // request | timer:5000 | event	int8_t	1 + 1	{ type[0] data[1] }
    // Максимально зафиксированная температура.
    co = cm.add_can_object(BMS_CANO_ID_MAX_TEMPERATURE, "MaxTemperature");
    df = co->add_data_field(DF_INT8, &bms_can_data.max_temperature);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);
    func_timer_norm = (CANFunctionTimerNormal *)co->add_function(CAN_FUNC_TIMER_NORMAL);
    func_timer_norm->set_period(5000);

    // 0x0047	LowVoltageMinMaxDelta
    // request | event	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: Минимальное, Максимальное, Дельта.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_MIN_MAX_DELTA, "LowVoltageMinMaxDelta");
    df = co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.min);
    df = co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.max);
    df = co->add_data_field(DF_UINT16, &bms_can_data.bms_low_voltage_min_max_delta.delta);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0048	Temperature1
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: MOS, Balancer, Temp1, Temp2, Temp3, Temp4, Temp5.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_1, "Temperature1");
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.mosfet);
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.balancer);
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor1);
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor2);
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor3);
    df = co->add_data_field(DF_INT8, &bms_can_data.bms_temperature.bms_sensor4);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp5);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0049	Temperature2
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: Temp6, Temp7, Temp8, Temp9, Temp10, Temp11, Temp12.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_2, "Temperature2");
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp6);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp7);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp8);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp9);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp10);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp11);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp12);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004A	Temperature3
    // request	int8_t	1 + 7	{ type[0] t1[1] t2[2] t3[3] t4[4] t5[5] t6[6] t7[7] }
    // Температура: Temp13, Temp14, Temp15, Temp16, Temp17, Temp18, Temp19.
    co = cm.add_can_object(BMS_CANO_ID_TEMPERATURE_3, "Temperature3");
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp13);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp14);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp15);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp16);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp17);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp18);
    df = co->add_data_field(DF_INT8, &bms_can_data.other_temperature.temp19);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004B	LowVoltage1-3
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 1, 2, 3.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_1_3, "LowVoltage1-3");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_1);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_2);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_3);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004C	LowVoltage4-6
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 4, 5, 6.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_4_6, "LowVoltage4-6");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_4);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_5);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_6);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004D	LowVoltage7-9
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 7, 8, 9.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_7_9, "LowVoltage7-9");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_7);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_8);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_9);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004E	LowVoltage10-12
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 10, 11, 12.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_10_12, "LowVoltage10-12");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_10);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_11);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_12);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x004F	LowVoltage13-15
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 13, 14, 15.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_13_15, "LowVoltage13-15");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_13);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_14);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_15);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0050	LowVoltage16-18
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 16, 17, 18.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_16_18, "LowVoltage16-18");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_16);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_17);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_18);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0051	LowVoltage19-21
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 19, 20, 21.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_19_21, "LowVoltage19-21");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_19);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_20);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_21);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    // 0x0052	LowVoltage22-24
    // request	uint16_t	1 + 6	{ type[0] v1[1..2] v2[3..4] v3[5..6] }
    // Напряжение на банках: 22, 23, 24.
    co = cm.add_can_object(BMS_CANO_ID_LOW_VOLTAGE_22_24, "LowVoltage22-24");
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_22);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_23);
    df = co->add_data_field(DF_UINT16, &bms_can_data.low_voltage.cell_24);
    func_request = (CANFunctionRequest *)co->add_function(CAN_FUNC_REQUEST_IN);

    return true;
}