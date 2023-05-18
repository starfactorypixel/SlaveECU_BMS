#ifndef BMS_DATA_H
#define BMS_DATA_H

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <cstring>

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
    struct packet_structure_t
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
    };
#pragma pack(pop)
    static_assert(sizeof(packet_structure_t) == BMS_BOARD_PACKET_SIZE, "BMS_BOARD_PACKET_SIZE const differs with actual size of structure.");

#define BMS_TEMPERATURE_SENSORS_COUNT 6
#define EXTERNAL_TEMPERATURE_SENSORS_COUNT 15
#define TOTAL_TEMPERATURE_SENSORS_COUNT BMS_TEMPERATURE_SENSORS_COUNT + EXTERNAL_TEMPERATURE_SENSORS_COUNT

    // ***************************************************************************************************
    // BMS related functions
    uint16_t bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE]);
    uint16_t get_bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE]);
    bool bms_raw_data_validation(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE]);
    int8_t calculate_max_temperature(int8_t temperatures[TOTAL_TEMPERATURE_SENSORS_COUNT]);
    void process_bms_packet(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE]);

#ifdef __cplusplus
}
#endif

#endif // BMS_DATA_H