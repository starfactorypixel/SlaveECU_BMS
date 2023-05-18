#include "BMS_low_level_abstraction.h"

/// @brief Calculates BMS raw packet data CRC
/// @param bms_packet_data Array with raw BMS data
/// @return Calculated CRC of the BMS packet
uint16_t bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    uint16_t result = 0x0000;

    for (uint8_t i = sizeof(BMS_PACKET_HEADER); i < BMS_BOARD_PACKET_SIZE - 2; ++i)
    {
        result += *(bms_packet_data + i);
    }

    return result;
}

/// @brief Return CRC value of BMS raw packet
/// @param bms_packet_data Array with raw BMS data
/// @return CRC from the BMS packet
uint16_t get_bms_raw_data_crc(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    return *(uint16_t *)&bms_packet_data[BMS_BOARD_PACKET_SIZE - 2];
}

/// @brief BMS raw packet data CRC validation
/// @param bms_packet_data Array with raw BMS data
/// @return 'true' if the calculated CRC matches the one stored in the BMS packet
bool bms_raw_data_validation(uint8_t bms_packet_data[BMS_BOARD_PACKET_SIZE])
{
    if (bms_packet_data == nullptr)
        return false;

    return get_bms_raw_data_crc(bms_packet_data) == bms_raw_data_crc(bms_packet_data);
}

/// @brief Calculates the maximum temperature based on the values from the internal and external BMS temperature sensors.
/// @param temperatures Array of temperatures
/// @return Maximum temperature
int8_t calculate_max_temperature(int8_t temperatures[TOTAL_TEMPERATURE_SENSORS_COUNT])
{
    int8_t result = INT8_MIN;

    for (uint8_t i = 0; i < TOTAL_TEMPERATURE_SENSORS_COUNT; i++)
    {
        if (result < temperatures[i])
            result = temperatures[i];
    }

    return result;
}
