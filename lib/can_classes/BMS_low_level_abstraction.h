#ifndef BMS_DATA_H
#define BMS_DATA_H

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <cstring>

#include "CANLibrary.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BMS_BOARD_PACKET_SIZE 140      // The size of uart packet of the BMS. Used for buffer declaration
#define BMS_BATTERY_NUMBER_OF_CELLS 32 // Number of cells in BMS packet
#define BMS_PACKET_HEADER 0xAA55AAFF   // from SlaveECU github

// ***************************************************************************************************
// BMS Packet structure
#pragma pack(push, 1)
    typedef struct
    {
        uint32_t header; // Заголовок пакета.

        uint16_t voltage;                                    // Напряжение на всей АКБ, x10 мВ.
        uint16_t cells_voltage[BMS_BATTERY_NUMBER_OF_CELLS]; // Напряжение на каждой из 32-х ячеек АКБ, мВ.
        uint16_t reserved_70;                                //
        int16_t current;                                     // Ток разряда или заряда, мА.

        uint8_t percent;         // Оставшаяся ёмкость АКБ, проценты.
        uint32_t phy_capacity;   // Фактическая ёмкость АКБ, мА/ч.
        uint32_t rem_capacity;   // Оставшаяся ёмкость АКБ, мА/ч.
        uint32_t cycle_capacity; // WAT?

        uint32_t uptime; // Кол-во времени с питанием, секунд.

        // Attention! In case of order changing of the temperature fields we need:
        //      - edit _bms_packet_temperature_sensor enum,
        //      - change the get_temperature_by_id() method.
        // If temperature fields are not arranged in a row, the get_temperature_by_id() method will be broken.
        int16_t temperature_mosfet;     // Температура датчика MOSFET, Градусы.
        int16_t temperature_balancer;   // Температура датчика Balance, Градусы.
        int16_t temperature_sensors[4]; // Температура датчиков 1, Градусы.

        uint8_t charge_fet;   // Флаг состояние ключа зарядки.
        uint8_t dcharge_fet;  // Флаг состояние ключа разрядки.
        uint8_t balanced;     // Флаг состояние балансировки.
        uint16_t tire_length; // WAT?
        uint16_t pulses_num;  // WAT?
        uint8_t relay;        // WAT?
        uint32_t power;       // Текущая мощность, Вт.

        uint8_t vmax_cell;     // Номер ячейки батареи с максимальным напряжением.
        uint16_t vmax_voltage; // Напряжение ячейки батареи с максимальным напряжением, мВ.
        uint8_t vmin_cell;     // Номер ячейки батареи с минимальным напряжением.
        uint16_t vmin_voltage; // Напряжение ячейки батареи с минимальным напряжением, мВ.
        uint16_t vmid_voltage; // Среднее напряжение батареи, мВ.

        uint8_t unknown_1;         // WAT?
        uint16_t dcharge_fet_lost; // Падение на транзисторе нагрузки, В.
        uint16_t dcharge_fet_v;    // Напряжение на затворе транзистора нагрузки, В.
        uint16_t charge_fet_v;     // Напряжение на затворе транзистора зарядки, В.
        uint16_t unknown_2;        // WAT?
        uint32_t balance_bits;     // Флаги сбалансированных ячеек.
        uint16_t logs;             // WAT?

        uint16_t crc; // Контрольная сумма.
    } packet_structure_t;
#pragma pack(pop)

    static_assert(sizeof(packet_structure_t) == BMS_BOARD_PACKET_SIZE, "BMS_BOARD_PACKET_SIZE const differs with actual size of params_t structure.");

    // ***************************************************************************************************
    // BMS CAN related types and constants
    enum bms_can_object_id_t : uint16_t
    {
        BMS_CANO_ID_BLOCK_INFO = 0x0040,
        BMS_CANO_ID_BLOCK_HEALTH = 0x0041,
        BMS_CANO_ID_BLOCK_CFG = 0x0042,
        BMS_CANO_ID_BLOCK_ERROR = 0x0043,
        BMS_CANO_ID_HIGH_VOLTAGE = 0x0044,
        BMS_CANO_ID_HIGH_CURRENT = 0x0045,
        BMS_CANO_ID_MAX_TEMPERATURE = 0x0046,
        BMS_CANO_ID_LOW_VOLTAGE_MIN_MAX_DELTA = 0x0047,
        BMS_CANO_ID_TEMPERATURE_1 = 0x0048,
        BMS_CANO_ID_TEMPERATURE_2 = 0x0049,
        BMS_CANO_ID_TEMPERATURE_3 = 0x004A,
        BMS_CANO_ID_LOW_VOLTAGE_1_3 = 0x004B,
        BMS_CANO_ID_LOW_VOLTAGE_4_6 = 0x004C,
        BMS_CANO_ID_LOW_VOLTAGE_7_9 = 0x004D,
        BMS_CANO_ID_LOW_VOLTAGE_10_12 = 0x004E,
        BMS_CANO_ID_LOW_VOLTAGE_13_15 = 0x004F,
        BMS_CANO_ID_LOW_VOLTAGE_16_18 = 0x0050,
        BMS_CANO_ID_LOW_VOLTAGE_19_21 = 0x0051,
        BMS_CANO_ID_LOW_VOLTAGE_22_24 = 0x0052,
    };

// 0x0040	BlockInfo
typedef block_info_t bms_block_info_t; 

// 0x0041	BlockHealth
typedef block_health_t bms_block_health_t; 

// 0x0042	BlockCfg
typedef block_cfg_t bms_block_cfg_t; 

// 0x0043	BlockError
typedef block_error_t bms_block_error_t; 

// 0x0047	LowVoltageMinMaxDelta
#pragma pack(push, 1)
    struct bms_low_voltage_min_max_delta_t
    {
        // byte 1 & 2
        uint16_t min;

        // byte 3 & 4
        uint16_t max;

        // byte 5 & 6
        uint16_t delta;

        // byte 7
        // uint8_t unused1;
    };
#pragma pack(pop)

// 0x0048	Temperature1
#pragma pack(push, 1)
    struct bms_temperature_t
    {
        int8_t mosfet;
        int8_t balancer;
        int8_t bms_sensor1;
        int8_t bms_sensor2;
        int8_t bms_sensor3;
        int8_t bms_sensor4;
    };
#pragma pack(pop)

// 0x0048	Temperature1
// 0x0049	Temperature2
// 0x004A	Temperature3
#pragma pack(push, 1)
    struct other_temperature_t
    {
        union
        {
            int8_t temp_5_19[15];
            struct
            {
                // 0x0048	Temperature1
                int8_t temp5;
                // 0x0049	Temperature2
                int8_t temp6;
                int8_t temp7;
                int8_t temp8;
                int8_t temp9;
                int8_t temp10;
                int8_t temp11;
                int8_t temp12;
                // 0x004A	Temperature3
                int8_t temp13;
                int8_t temp14;
                int8_t temp15;
                int8_t temp16;
                int8_t temp17;
                int8_t temp18;
                int8_t temp19;
            };
        };
    };
#pragma pack(pop)

// 0x004B	LowVoltage1-3
// 0x004C	LowVoltage4-6
// 0x004D	LowVoltage7-9
// 0x004E	LowVoltage10-12
// 0x004F	LowVoltage13-15
// 0x0050	LowVoltage16-18
// 0x0051	LowVoltage19-21
// 0x0052	LowVoltage22-24
#pragma pack(push, 1)
    struct low_voltage_t
    {
        union
        {
            uint16_t cells_voltage[BMS_BATTERY_NUMBER_OF_CELLS];
            struct
            {
                uint16_t cell_1;
                uint16_t cell_2;
                uint16_t cell_3;
                uint16_t cell_4;
                uint16_t cell_5;
                uint16_t cell_6;
                uint16_t cell_7;
                uint16_t cell_8;
                uint16_t cell_9;
                uint16_t cell_10;
                uint16_t cell_11;
                uint16_t cell_12;
                uint16_t cell_13;
                uint16_t cell_14;
                uint16_t cell_15;
                uint16_t cell_16;
                uint16_t cell_17;
                uint16_t cell_18;
                uint16_t cell_19;
                uint16_t cell_20;
                uint16_t cell_21;
                uint16_t cell_22;
                uint16_t cell_23;
                uint16_t cell_24;
                // below are  cells unused by Pixel
                uint16_t cell_25;
                uint16_t cell_26;
                uint16_t cell_27;
                uint16_t cell_28;
                uint16_t cell_29;
                uint16_t cell_30;
                uint16_t cell_31;
                uint16_t cell_32;
            };
        };
    };

#pragma pack(pop)

#pragma pack(push, 1)
    struct bms_can_data_t
    {
        // 0x0040	BlockInfo
        bms_block_info_t block_info;

        // 0x0041	BlockHealth
        bms_block_health_t block_health;

        // 0x0042	BlockCfg
        bms_block_cfg_t block_cfg;

        // 0x0043	BlockError
        bms_block_error_t block_error;

        // 0x0044	HighVoltage
        uint32_t high_voltage;

        // 0x0045	HighCurrent
        int32_t high_current;

        // 0x0046	MaxTemperature
        int8_t max_temperature;

        // 0x0047	LowVoltageMinMaxDelta
        bms_low_voltage_min_max_delta_t bms_low_voltage_min_max_delta;

        // 0x0048	Temperature1
        bms_temperature_t bms_temperature;

        // 0x0048	Temperature1
        // 0x0049	Temperature2
        // 0x004A	Temperature3
        other_temperature_t other_temperature;

        // 0x004B	LowVoltage1-3
        // 0x004C	LowVoltage4-6
        // 0x004D	LowVoltage7-9
        // 0x004E	LowVoltage10-12
        // 0x004F	LowVoltage13-15
        // 0x0050	LowVoltage16-18
        // 0x0051	LowVoltage19-21
        // 0x0052	LowVoltage22-24
        low_voltage_t low_voltage;
    };
#pragma pack(pop)

    // ***************************************************************************************************
    // BMS related functions
    uint16_t bms_crc(uint8_t *bms_packet_data);
    void fill_bms_test_data(uint8_t *bms_packet_data);
    void convert_bms_data_from_uart_to_can_structure(uint8_t *bms_packet_data, bms_can_data_t &bms_can_data);
    bool init_can_manager_for_bms(CANManager &cm, bms_can_data_t &bms_can_data);

#ifdef __cplusplus
}
#endif

#endif // BMS_DATA_H