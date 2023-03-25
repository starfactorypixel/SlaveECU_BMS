#include "CANObject.h"

/******************************************************************************************************************************
 * 
 * CANObject class: CAN data object with ID and several DataFields
 * 
 ******************************************************************************************************************************/
CANObject::CANObject()
{
    _id = 0;
    
    _data_fields = nullptr;
    _data_fields_count = 0;

    _data_local = nullptr;
    //_data_local_length = 0;
    
    _has_data_to_send = false;
    _was_send_at = 0;
    _period_ms = UINT32_MAX;

    _state = COS_OK;
    _tick_func = nullptr;
}

CANObject::CANObject(uint16_t id, get_ms_tick_function_t tick_func) : CANObject()
{
    set_id(id);
    set_tick_func(tick_func);
}

CANObject::~CANObject()
{
    if (has_data_fields())
    {
        for (uint8_t i = 0; i < get_data_fields_count(); i++)
        {
            delete _data_fields[i];
        }
        delete[] _data_fields;
    }
    if (_data_local != nullptr)
        delete [] _data_local;
}

can_id_t CANObject::get_id()
{
    return _id;
}

void CANObject::set_id(can_id_t id)
{
    _id = id;
}

void CANObject::set_tick_func(get_ms_tick_function_t tick_func)
{
    _tick_func = tick_func;
}

bool CANObject::has_tick_func()
{
    return _tick_func != nullptr;
}

uint32_t CANObject::get_tick()
{
    if (has_tick_func())
        return _tick_func();
    
    return 0;
}

bool CANObject::_delete_data_local()
{
    if (_data_local == nullptr)
        return false;
    
    delete [] _data_local;
    _data_local = nullptr;

    return true;
}

bool CANObject::_resize_data_local(uint8_t new_size)
{
    _delete_data_local();
    _data_local = new uint8_t[new_size];
    memset(_data_local, 0, new_size);

    return true;
}

bool CANObject::_fit_data_local_to_data_fields()
{
    return _resize_data_local(calculate_all_data_size());
}

void *CANObject::_get_data_local()
{
    return _data_local;
}

uint8_t CANObject::get_data_fields_count()
{
    return _data_fields_count;
}

bool CANObject::has_data_fields()
{
    if ((_data_fields == nullptr || get_data_fields_count() == 0))
    {
        _set_state(COS_DATA_FIELD_ERROR);
        return false;
    }
    return true;
}

DataField *CANObject::add_data_field()
{
    DataField **new_data_fields = new DataField *[get_data_fields_count() + 1];
    memcpy(new_data_fields, _data_fields, sizeof(DataField *) * get_data_fields_count());
    new_data_fields[get_data_fields_count()] = new DataField;
    new_data_fields[get_data_fields_count()]->set_tick_func(_tick_func);
    new_data_fields[get_data_fields_count()]->update_state();

    if (has_data_fields())
    {
        delete[] _data_fields;
    }
    _data_fields = new_data_fields;
    _data_fields_count++;

    update_state();

    return _data_fields[get_data_fields_count() - 1];
}

DataField *CANObject::add_data_field(data_field_t type, void *data, uint32_t array_item_count)
{
    DataField *new_data_field = add_data_field();
    new_data_field->set_data_source(type, data, array_item_count);
    new_data_field->update_state();
    
    update_state();
    
    return new_data_field;
}

bool CANObject::delete_data_field(uint8_t index)
{
    if (index >= get_data_fields_count())
        return false;

    DataField **new_data_fields = new DataField *[get_data_fields_count() - 1];

    memcpy(new_data_fields, _data_fields, sizeof(DataField *) * index);
    memcpy(new_data_fields + index, _data_fields + index + 1, sizeof(DataField *) * (get_data_fields_count() - index - 1));
    delete _data_fields[index];
    delete[] _data_fields;
    _data_fields = new_data_fields;
    _data_fields_count--;

    update_state();

    return true;
}

DataField *CANObject::get_data_field(uint8_t index)
{
    if (index >= get_data_fields_count())
        return nullptr;

    return _data_fields[index];
}

uint8_t CANObject::calculate_all_data_size()
{
    if (!has_data_fields()) return 0;

    uint8_t result = 0;
    for (uint8_t i = 0; i < get_data_fields_count(); i++)
    {
        result += get_data_field(i)->get_data_byte_array_length();
    }

    // 1 additional byte we should use for FUNC_TYPE
    return result + 1;
}

can_object_state_t CANObject::get_state()
{
    return _state;
}

void CANObject::_set_state(can_object_state_t state)
{
    _state = state;
}

can_object_state_t CANObject::update_state()
{
    if (!has_data_fields())
        return get_state();

    for (uint8_t i = 0; i < get_data_fields_count(); i++)
    {
        get_data_field(i)->update_state();
    }

    _set_state(COS_OK);
    DataField *erroneous_data_field = get_first_erroneous_data_field();
    if (erroneous_data_field != nullptr)
        _set_state(COS_DATA_FIELD_ERROR);
    
    return get_state();
}

bool CANObject::update()
{
    if (update_state() != COS_OK)
        return false;
    
    _fit_data_local_to_data_fields();

    bool has_new_data_flag = false;
    // use second one byte as the start point for DataFields
    // because we should reserve the first byte for CAN frame FUNC_TYPE
    uint8_t destination_offset = 1;
    for (uint8_t i = 0; i < get_data_fields_count(); i++)
    {
        get_data_field(i)->update_state();
        
        has_new_data_flag = has_new_data_flag || get_data_field(i)->is_data_changed();
        if (get_data_field(i)->is_data_changed())
        {
            get_data_field(i)->update_local_copy();
            if (!_copy_data_field_to_local(i, destination_offset))
                return false;
        }

        destination_offset += get_data_field(i)->get_data_byte_array_length();
    }
    _has_data_to_send = _has_data_to_send || has_new_data_flag; // if already has data to send then don't replace it with false

    return true;
}

void CANObject::print(const char *prefix)
{
    LOG("%sCAN object 0x%04X: state = %d, num of fields = %d", prefix, get_id(), get_state(), get_data_fields_count());
    for (uint8_t i = 0; i < get_data_fields_count(); i++)
    {
        if (i == 0) LOG("%sData fields:", prefix);
        LOG("%s#%d: state = %s, item size = %d, item count = %d", prefix, i,
            (get_data_field(i)->get_state() == DFS_OK) ? "DFS_OK" : "DFS_ERROR",
            get_data_field(i)->get_item_size(),
            get_data_field(i)->get_item_count());
    }
}

bool CANObject::_copy_data_field_to_local(uint8_t data_field_index, uint8_t dest_byte_offset)
{
    uint8_t remaining_data = 0;
    uint8_t *data_pointer = ((uint8_t *)_get_data_local());
    remaining_data = calculate_all_data_size() - dest_byte_offset;
    if (remaining_data == 0)
    {
        _set_state(COS_LOCAL_DATA_BUFFER_SIZE_ERROR);
        return false;
    }
    data_pointer += dest_byte_offset;
    get_data_field(data_field_index)->copy_data_to(data_pointer, remaining_data);

    return true;
}

DataField *CANObject::get_first_erroneous_data_field()
{
    if (!has_data_fields()) return nullptr;

    for (uint8_t i = 0; i < get_data_fields_count(); i++)
    {
        if (get_data_field(i)->update_state() != DFS_OK)
            return get_data_field(i);
    }
    
    return nullptr;
}

bool CANObject::has_data_to_send()
{
    return _has_data_to_send || _check_timer();
}

bool CANObject::_check_timer()
{
    if (!has_tick_func() || _period_ms == UINT32_MAX)
        return false;
    
    if (get_tick() - _was_send_at >= _period_ms)
        return true;

    return false;
}

bool CANObject::fill_can_frame(CANFrame &can_frame, func_type_t func)
{
    // TODO: error reporting shold be here to?
    if (get_state() != COS_OK)
    //if (get_state() != COS_OK || !has_data_to_send())
        return false;
    
    *(uint8_t *)_get_data_local() = func;

    if (can_frame.get_max_data_length() < calculate_all_data_size())
        return false;

    can_frame.clear_frame();
    can_frame.set_frame(get_id(), (uint8_t *)_get_data_local(), calculate_all_data_size()); 

    _was_send_at = get_tick();
    _has_data_to_send = false;


    return true;
}