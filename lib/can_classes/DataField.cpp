#include "DataField.h"

/******************************************************************************************************************************
 *
 * DataField class: keep the data pointer, data type and data size of the data source
 *
 ******************************************************************************************************************************/
// 'unknown' for logging
const char *DataField::_value_unknown = "unknown";
// the data field source type names for logging
const char *DataField::_source_type_int8 = "int8";
const char *DataField::_source_type_uint8 = "uint8";
const char *DataField::_source_type_int16 = "int16";
const char *DataField::_source_type_uint16 = "uint16";
const char *DataField::_source_type_int32 = "int32";
const char *DataField::_source_type_uint32 = "uint32";

// the data field state names for logging
const char *DataField::_state_data_field_ok = "ok";
const char *DataField::_state_data_field_alarm = "alarm";
const char *DataField::_state_data_field_error = "error";

DataField::DataField()
{
    _zerroing_all_unsafe();
}

DataField::DataField(data_field_t source_type, void *data_source, uint32_t array_item_count) : DataField()
{
    set_data_source(source_type, data_source, array_item_count);
}

DataField::~DataField()
{
    delete_data_source();
}

// use it with caution! it is unsafe! risk of memory leak!
void DataField::_zerroing_all_unsafe()
{
    _last_data_copy = nullptr;

    _src_data_pointer = nullptr;
    _source_type = DF_UNKNOWN;
    _array_item_count = 0;
    _array_item_size = 0;

    _checker_min.u32 = 0;
    _checker_max.u32 = 0;

    set_state(DFS_ERROR);
}

void DataField::delete_data_source()
{
    if (_last_data_copy != nullptr)
    {
        switch (_source_type)
        {
        case DF_INT8:
            delete[] (int8_t *)_last_data_copy;
            break;

        case DF_UINT8:
            delete[] (uint8_t *)_last_data_copy;
            break;

        case DF_INT16:
            delete[] (int16_t *)_last_data_copy;
            break;

        case DF_UINT16:
            delete[] (uint16_t *)_last_data_copy;
            break;

        case DF_INT32:
            delete[] (int32_t *)_last_data_copy;
            break;

        case DF_UINT32:
            delete[] (uint32_t *)_last_data_copy;
            break;

        case DF_UNKNOWN:
        default:
            break;
        }
    };
    _zerroing_all_unsafe();
}

bool DataField::set_data_source(data_field_t source_type, void *data, uint32_t array_item_count)
{
    if (source_type == DF_UNKNOWN || data == nullptr || array_item_count == 0)
    {
        set_state(DFS_ERROR);
        return false;
    }
    delete_data_source();

    switch (source_type)
    {
    case DF_INT8:
        _last_data_copy = new int8_t[array_item_count];
        _array_item_size = sizeof(int8_t);
        break;

    case DF_UINT8:
        _last_data_copy = new uint8_t[array_item_count];
        _array_item_size = sizeof(uint8_t);
        break;

    case DF_INT16:
        _last_data_copy = new int16_t[array_item_count];
        _array_item_size = sizeof(int16_t);
        break;

    case DF_UINT16:
        _last_data_copy = new uint16_t[array_item_count];
        _array_item_size = sizeof(uint16_t);
        break;

    case DF_INT32:
        _last_data_copy = new int32_t[array_item_count];
        _array_item_size = sizeof(int32_t);
        break;

    case DF_UINT32:
        _last_data_copy = new uint32_t[array_item_count];
        _array_item_size = sizeof(uint32_t);
        break;

    case DF_UNKNOWN:
    default:
        set_state(DFS_ERROR);
        // all data was set to zero below
        return false;
    }

    _source_type = source_type;
    _array_item_count = array_item_count;
    _src_data_pointer = data;

    update_local_copy();
    update_state();

    return !has_errors();
}

bool DataField::has_data_source()
{
    if (_array_item_size == 0 || _array_item_count == 0)
        return false;

    if (_src_data_pointer == nullptr || _last_data_copy == nullptr)
        return false;

    if (_source_type == DF_UNKNOWN)
        return false;

    return true;
}

bool DataField::is_data_changed()
{
    if (!has_data_source())
        return false;

    uint8_t *src = (uint8_t *)_src_data_pointer;
    uint8_t *last = (uint8_t *)_last_data_copy;

    for (uint8_t i = 0; i < get_data_byte_array_length(); i++)
    {
        if ((src[i] ^ last[i]) != 0)
            return true;
    }
    return false;
}

bool DataField::update_local_copy()
{
    if (!has_data_source())
        return false;

    memcpy(get_data_byte_array_pointer(), get_src_pointer(), get_data_byte_array_length());

    return true;
}

void DataField::set_alarm_checker(data_mapper_t min, data_mapper_t max)
{
    _checker_min = min;
    _checker_max = max;
}

void DataField::reset_alarm_checker()
{
    _checker_min.u32 = 0;
    _checker_max.u32 = 0;
}

// checks if there any issues found by checker;
// it checks local copy of data, not the source
void DataField::perform_alarm_check()
{
    if (!has_data_source() || has_errors())
        return;

    // checker is disabled
    if (_checker_min.u32 == 0 && _checker_max.u32 == 0)
        return;

    data_mapper_t curr_value = {0};
    memcpy(&curr_value, get_data_byte_array_pointer(), _array_item_size);

    switch (get_source_type())
    {
    case DF_INT8:
        if (curr_value.i8 < _checker_min.i8 || curr_value.i8 > _checker_max.i8)
            set_state(DFS_ALARM);
        break;

    case DF_UINT8:
        if (curr_value.u8 < _checker_min.u8 || curr_value.u8 > _checker_max.u8)
            set_state(DFS_ALARM);
        break;

    case DF_INT16:
        if (curr_value.i16 < _checker_min.i16 || curr_value.i16 > _checker_max.i16)
            set_state(DFS_ALARM);
        break;

    case DF_UINT16:
        if (curr_value.u16 < _checker_min.u16 || curr_value.u16 > _checker_max.u16)
            set_state(DFS_ALARM);
        break;

    case DF_INT32:
        if (curr_value.i32 < _checker_min.i32 || curr_value.i32 > _checker_max.i32)
            set_state(DFS_ALARM);
        break;

    case DF_UINT32:
        if (curr_value.u32 < _checker_min.u32 || curr_value.u32 > _checker_max.u32)
            set_state(DFS_ALARM);
        break;

    default:
        break;
    }
}

bool DataField::has_alarm_state()
{
    return get_state() == DFS_ALARM;
}

data_field_t DataField::get_source_type()
{
    return _source_type;
}

const char *DataField::get_source_type_name()
{
    switch (get_source_type())
    {
    case DF_INT8:
        return _source_type_int8;

    case DF_UINT8:
        return _source_type_uint8;

    case DF_INT16:
        return _source_type_int16;

    case DF_UINT16:
        return _source_type_uint16;

    case DF_INT32:
        return _source_type_int32;

    case DF_UINT32:
        return _source_type_uint32;

    case DF_UNKNOWN:
    default:
        return _value_unknown;
    }
}

uint32_t DataField::get_item_count()
{
    return _array_item_count;
}

uint8_t DataField::get_item_size()
{
    return _array_item_size;
}

void *DataField::get_src_pointer()
{
    return _src_data_pointer;
}

void *DataField::get_data_byte_array_pointer()
{
    return _last_data_copy;
}

uint32_t DataField::get_data_byte_array_length()
{
    return get_item_count() * get_item_size();
}

bool DataField::copy_data_to(void *destination, uint8_t destination_max_size)
{
    if (!has_data_source())
        return false;

    if (destination == nullptr || destination_max_size < get_data_byte_array_length())
        return false;

    memcpy(destination, get_data_byte_array_pointer(), get_data_byte_array_length());
    return true;
}

data_field_state_t DataField::get_state()
{
    return _state;
}

void DataField::set_state(data_field_state_t state)
{
    _state = state;
}

data_field_state_t DataField::update_state()
{
    set_state(DFS_OK);

    perform_alarm_check();

    if (!has_data_source())
    {
        set_state(DFS_ERROR);
    }

    return get_state();
}

bool DataField::has_errors()
{
    return get_state() == DFS_ERROR;
}

const char *DataField::get_state_name()
{
    switch (get_state())
    {
    case DFS_OK:
        return _state_data_field_ok;

    case DFS_ALARM:
        return _state_data_field_alarm;

    case DFS_ERROR:
        return _state_data_field_error;

    default:
        return _value_unknown;
    }
}

void DataField::print(const char *prefix)
{
    LOG("%sDataField: state = %s, item size = %d (%s), item count = %d", prefix, get_state_name(),
        get_item_size(), get_source_type_name(), get_item_count());
}