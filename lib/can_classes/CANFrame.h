#ifndef CANFRAME_H
#define CANFRAME_H
// #pragma once

#include <stdint.h>
#include <string.h>

#include "logger.h"
#include "CAN_common.h"

/******************************************************************************************************************************
 * CANFrame: data frame of CAN bus
 ******************************************************************************************************************************/
class CANFrame
{
public:
    CANFrame();
    CANFrame(can_id_t id, uint8_t *data, uint8_t data_length);
    ~CANFrame();

    bool operator==(const CANFrame &frame);

    void set_frame(CANFrame &can_frame);
    void set_frame(can_id_t id, uint8_t *data, uint8_t data_length);
    void set_frame(can_id_t id, uint8_t data_length,
                   uint8_t v1 = 0, uint8_t v2 = 0, uint8_t v3 = 0, uint8_t v4 = 0,
                   uint8_t v5 = 0, uint8_t v6 = 0, uint8_t v7 = 0, uint8_t v8 = 0);

    void clear_frame();

    bool is_initialized();

    const can_id_t get_id();
    void set_id(can_id_t id);

    const CAN_function_id_t get_function_id();
    void set_function_id(CAN_function_id_t id);

    const uint8_t get_data_length();
    static const uint8_t get_max_data_length();
    uint8_t *get_data_pointer();
    bool copy_frame_data_to(uint8_t *destination, uint8_t max_dest_length);

    const bool has_data();

    void print(const char *prefix);

protected:
    void _frame_func_to_string(char *dest_string);

    can_id_t _id = 0;
    uint8_t _data[CAN_MAX_PAYLOAD] = {0x00};
    uint8_t _data_length = 0;

    bool _is_initialized = false;
};

#endif // CANFRAME_H