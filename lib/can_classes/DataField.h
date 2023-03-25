#ifndef DATAFIELD_H
#define DATAFIELD_H
//#pragma once

#include <stdint.h>
#include <string.h>

#include "CAN_common.h"

/******************************************************************************************************************************
 *
 * DataField class: keep the data pointer, data type and data size of the data source
 *
 ******************************************************************************************************************************/
class DataField
{
public:
    DataField();
    DataField(data_field_t type, void *data, uint32_t array_item_count, get_ms_tick_function_t tick_func);
    ~DataField();

    void set_tick_func(get_ms_tick_function_t tick_func);
    bool has_tick_func();
    uint32_t get_tick();

    void delete_data_source();
    bool set_data_source(data_field_t type = DF_UNKNOWN, void *data = nullptr, uint32_t array_item_count = 1);
    bool has_data_source();

    // checks if there is a difference in data between the source and local copy
    bool is_data_changed();
    // updates local copy of the data from the source
    bool update_local_copy();

    uint32_t get_last_update_time();

    data_field_t get_source_type();
    uint32_t get_item_count();
    uint8_t get_item_size();
    void *get_src_pointer();

    void *get_data_byte_array_pointer();
    uint32_t get_data_byte_array_length();
    bool copy_data_to(void *destination, uint8_t destination_max_size);

    data_field_state_t get_state();
    data_field_state_t update_state();

private:
    void _zerroing_all_unsafe();
    //void _set_state(data_field_state_t state);

    void *_last_data_copy = nullptr;

    data_field_t _source_type = DF_UNKNOWN;
    void *_src_data_pointer = nullptr;
    uint32_t _array_item_count = 0; // number of items in the data source array; for single variable it is 1
    uint8_t _array_item_size = 0;   // sizeof one item of array; _type related

    uint32_t _last_update_time; // time of the last update

    data_field_state_t _state; // the state of data field

    get_ms_tick_function_t _tick_func = nullptr;
};

#endif // DATAFIELD_H