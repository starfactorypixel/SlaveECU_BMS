#ifndef DATAFIELD_H
#define DATAFIELD_H
// #pragma once

#include <stdint.h>
#include <string.h>

#include "logger.h"
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
    DataField(data_field_t type, void *data_source, uint32_t array_item_count);
    ~DataField();

    void delete_data_source();
    bool set_data_source(data_field_t type = DF_UNKNOWN, void *data_source = nullptr, uint32_t array_item_count = 1);
    bool has_data_source();

    // checks if there is a difference in data between the source and local copy
    bool is_data_changed();
    // updates local copy of the data from the source
    bool update_local_copy();

    // alarm checker
    void set_alarm_checker(data_mapper_t min, data_mapper_t max);
    void reset_alarm_checker();
    // checks if there any issues found by checker;
    // it checks local copy of data, not the source
    void perform_alarm_check();
    bool has_alarm_state();

    data_field_t get_source_type();
    const char *get_source_type_name();

    uint32_t get_item_count();
    uint8_t get_item_size();
    void *get_src_pointer();

    void *get_data_byte_array_pointer();
    uint32_t get_data_byte_array_length();
    bool copy_data_to(void *destination, uint8_t destination_max_size);

    data_field_state_t get_state();
    void set_state(data_field_state_t state);
    data_field_state_t update_state();
    bool has_errors();
    const char *get_state_name();

    void print(const char *prefix);

private:
    // use it with caution! it is unsafe! risk of memory leak!
    void _zerroing_all_unsafe();

    void *_last_data_copy = nullptr;

    data_field_t _source_type = DF_UNKNOWN;
    void *_src_data_pointer = nullptr;
    uint32_t _array_item_count = 0; // number of items in the data source array; for single variable it is 1
    uint8_t _array_item_size = 0;   // sizeof one item of array; _type related

    data_field_state_t _state; // the state of data field

    // checker properties
    data_mapper_t _checker_min;
    data_mapper_t _checker_max;

    // 'unknown' for logging
    static const char *_value_unknown;
    // the data field source type names for logging
    static const char *_source_type_int8;
    static const char *_source_type_uint8;
    static const char *_source_type_int16;
    static const char *_source_type_uint16;
    static const char *_source_type_int32;
    static const char *_source_type_uint32;

    // the data field state names for logging
    static const char *_state_data_field_ok;
    static const char *_state_data_field_alarm;
    static const char *_state_data_field_error;
};

#endif // DATAFIELD_H