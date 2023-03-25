#include "DataField.h"

/******************************************************************************************************************************
 *
 * DataField class: keep the data pointer, data type and data size of the data source
 *
 ******************************************************************************************************************************/
DataField::DataField()
{
    _zerroing_all_unsafe();
}

DataField::DataField(data_field_t source_type, void *data, uint32_t array_item_count, get_ms_tick_function_t tick_func)
{
    DataField();
    set_data_source(source_type, data, array_item_count);
    set_tick_func(tick_func);
}

DataField::~DataField()
{
    delete_data_source();
}

void DataField::set_tick_func(get_ms_tick_function_t tick_func)
{
    _tick_func = tick_func;
}

bool DataField::has_tick_func()
{
    return _tick_func != nullptr;
}

uint32_t DataField::get_tick()
{
    if (has_tick_func())
        return _tick_func();
    
    return 0;
}

void DataField::_zerroing_all_unsafe()
{
    _last_data_copy = nullptr;

    _src_data_pointer = nullptr;
    _source_type = DF_UNKNOWN;
    _array_item_count = 0;
    _array_item_size = 0;

    _last_update_time = 0;
    _state = DFS_ERROR;
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
        update_state();
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
        update_state();
        // all data was set to zero below
        return false;
    }

    _source_type = source_type;
    _array_item_count = array_item_count;
    _src_data_pointer = data;

    update_local_copy();
    
    return update_state() == DFS_OK;
}

bool DataField::has_data_source()
{
    if (_array_item_size == 0 || _array_item_count == 0)
    {
        //_set_state(DFS_ERROR);
        return false;
    }

    if (_src_data_pointer == nullptr || _last_data_copy == nullptr)
    {
        //_set_state(DFS_ERROR);
        return false;
    }

    if (_source_type == DF_UNKNOWN)
    {
        //_set_state(DFS_ERROR);
        return false;
    }
    
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

    _last_update_time = get_tick();

    return true;
}

uint32_t DataField::get_last_update_time()
{
    return _last_update_time;
}

data_field_t DataField::get_source_type()
{
    return _source_type;
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

/*
void DataField::_set_state(data_field_state_t state)
{
    _state = state;
}
*/

data_field_state_t DataField::update_state()
{
    bool state_checker = true;
    state_checker = state_checker && has_data_source();
    state_checker = state_checker && has_tick_func();

    _state = (state_checker) ? DFS_OK : DFS_ERROR;

    return get_state();
}
