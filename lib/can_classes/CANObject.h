#ifndef CANOBJECT_H
#define CANOBJECT_H
//#pragma once

#include <stdint.h>

#include "CAN_common.h"
#include "DataField.h"
#include "CANFrame.h"

/******************************************************************************************************************************
 *
 * CANObject class: CAN data object with ID and several DataFields
 *
 ******************************************************************************************************************************/
class CANObject
{
public:
    CANObject();
    CANObject(uint16_t id, get_ms_tick_function_t tick_func = nullptr);
    ~CANObject();

    can_id_t get_id();
    void set_id(can_id_t id);

    void set_tick_func(get_ms_tick_function_t tick_func);
    bool has_tick_func();
    uint32_t get_tick();

    uint8_t get_data_fields_count();
    bool has_data_fields();

    DataField *add_data_field();
    DataField *add_data_field(data_field_t type, void *data, uint32_t array_item_count = 1);
    bool delete_data_field(uint8_t index);
    DataField *get_data_field(uint8_t index);

    uint8_t calculate_all_data_size();

    can_object_state_t get_state();
    can_object_state_t update_state();
    DataField *get_first_erroneous_data_field();

    bool update();
    void print(const char *prefix);

    bool has_data_to_send();
    bool fill_can_frame(CANFrame &can_frame, func_type_t func);

protected:
    void _set_state(can_object_state_t state);

    bool _delete_data_local();
    bool _resize_data_local(uint8_t new_size);
    bool _fit_data_local_to_data_fields();
    void *_get_data_local();

    bool _copy_data_field_to_local(uint8_t data_field_index, uint8_t dest_byte_offset);

    bool _check_timer();

private:
    can_id_t _id;

    DataField **_data_fields = nullptr;
    uint8_t _data_fields_count = 0;

    uint8_t *_data_local = nullptr;
    //uint8_t _data_local_length = 0;

    bool _has_data_to_send = false;
    uint32_t _was_send_at = 0;        // time of last sending
    uint32_t _period_ms = UINT32_MAX; // period of regular sending, ms. UINT32_MAX = timer is off

    can_object_state_t _state = COS_OK;

    get_ms_tick_function_t _tick_func = nullptr;
};

#endif // CANOBJECT_H