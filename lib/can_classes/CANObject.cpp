#include "CANObject.h"

/******************************************************************************************************************************
 *
 * CANObject class: CAN data object with ID and several DataFields
 *
 ******************************************************************************************************************************/
CANObject::CANObject()
{
    _id = 0;
    _parent = nullptr;

    _data_fields_list.clear();
    _functions_list.clear();

    _data_local = nullptr;

    _state = COS_UNKNOWN_ERROR;
}

CANObject::CANObject(uint16_t id, CANManager &parent) : CANObject()
{
    set_id(id);
    set_parent(parent);
    _set_state(COS_OK);
}

CANObject::~CANObject()
{
    _data_fields_list.clear();
    for (CANFunctionBase *i : _functions_list)
    {
        if (i != nullptr)
            delete i;
    }
    _functions_list.clear();
    _delete_data_local();
}

bool CANObject::operator==(const CANObject &can_object)
{
    return (get_id() == can_object._id);
}

can_id_t CANObject::get_id()
{
    return _id;
}

void CANObject::set_id(can_id_t id)
{
    _id = id;
}

CANManager *CANObject::get_parent()
{
    return _parent;
}

void CANObject::set_parent(CANManager &parent)
{
    _parent = &parent;
}

uint32_t CANObject::get_tick()
{
    if (_parent == nullptr)
        return 0;

    return get_parent()->get_tick();
}

bool CANObject::_delete_data_local()
{
    if (_data_local == nullptr)
        return false;

    delete[] _data_local;
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
    return _data_fields_list.size();
}

bool CANObject::has_data_fields()
{
    if ((get_data_fields_count() == 0))
    {
        _set_state(COS_DATA_FIELD_ERROR);
        return false;
    }
    return true;
}

DataField *CANObject::add_data_field()
{
    DataField df;
    df.update_state();
    _data_fields_list.push_back(df);

    update_state();

    return &_data_fields_list.back();
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

    std::list<DataField>::iterator it = _data_fields_list.begin();
    std::advance(it, index);
    _data_fields_list.erase(it);

    update_state();

    return true;
}

DataField *CANObject::get_data_field(uint8_t index)
{
    if (index >= get_data_fields_count())
        return nullptr;

    std::list<DataField>::iterator it = _data_fields_list.begin();
    std::advance(it, index);

    return &(*it);
}

bool CANObject::has_data_fields_alarm()
{
    for (DataField &i : _data_fields_list)
    {
        i.update_state();
        if (i.has_alarm_state())
            return true;
    }
    return false;
}

uint8_t CANObject::calculate_all_data_size()
{
    if (!has_data_fields())
        return 0;

    uint8_t result = 0;
    for (DataField &i : _data_fields_list)
    {
        result += i.get_data_byte_array_length();
    }

    // 1 additional byte for FUNC_TYPE
    return result + 1;
}

can_object_state_t CANObject::get_state()
{
    return _state;
}

bool CANObject::is_state_ok()
{
    return get_state() == COS_OK;
}

void CANObject::_set_state(can_object_state_t state)
{
    _state = state;
}

// only updates states of data fields and transfer CANObject in error state if there is at least one erroneous DataField
can_object_state_t CANObject::update_state()
{
    if (!has_data_fields())
        return get_state();

    for (DataField &i : _data_fields_list)
    {
        i.update_state();
    }

    _set_state(COS_OK);
    DataField *erroneous_data_field = get_first_erroneous_data_field();
    if (erroneous_data_field != nullptr)
        _set_state(COS_DATA_FIELD_ERROR);

    return get_state();
}

// updates local data storage, performs active and automatic/blended functions calls
bool CANObject::update()
{
    update_state();
    if (!is_state_ok())
        return false;

    // work with data fields
    if (!update_local_data())
        return false;

    // work with functions
    for (CANFunctionBase *i : _functions_list)
    {
        // call only active and automatic/blended functions
        if (i->is_active() && i->is_automatic_function())
            i->process();
    }

    return true;
}

// updates local data storage only
bool CANObject::update_local_data()
{
    _fit_data_local_to_data_fields();

    // use second one byte as the start point for DataFields
    // because we should reserve the first byte for CAN frame FUNC_TYPE
    uint8_t destination_offset = 1;
    for (DataField &i : _data_fields_list)
    {
        i.update_state();
        i.update_local_copy();
        if (!_copy_data_field_to_local(i, destination_offset))
            return false;
        destination_offset += i.get_data_byte_array_length();
    }
    return true;
}

void CANObject::print(const char *prefix)
{
    char str_buff[70] = {0};
    switch (get_state())
    {
    case COS_OK:
        strcpy(str_buff, "COS_OK");
        break;

    case COS_DATA_FIELD_ERROR:
        strcpy(str_buff, "COS_DATA_FIELD_ERROR");
        break;

    case COS_LOCAL_DATA_BUFFER_SIZE_ERROR:
        strcpy(str_buff, "COS_LOCAL_DATA_BUFFER_SIZE_ERROR");
        break;

    case COS_UNKNOWN_ERROR:
        strcpy(str_buff, "COS_UNKNOWN_ERROR");
        break;

    default:
        break;
    }

    LOG("%sCAN object 0x%04X: state = %s, num of fields = %d", prefix, get_id(), str_buff, get_data_fields_count());

    LOG("%sData fields: %s", prefix, _data_fields_list.empty() ? "no data fields" : "");
    uint8_t count = 0;
    for (DataField &i : _data_fields_list)
    {
        LOG("%s    #%d: state = %s, item size = %d, item count = %d", prefix, ++count,
            (i.get_state() == DFS_OK) ? "DFS_OK" : "DFS_ERROR",
            i.get_item_size(),
            i.get_item_count());
    }

    LOG("%sFunctions: %s", prefix, _functions_list.empty() ? "no functions" : "");
    count = 0;
    for (CANFunctionBase *i : _functions_list)
    {
        LOG("%s    #%d: id = 0x%02X, state = %s", prefix, ++count,
            i->get_id(),
            (i->get_state() == CAN_FS_STOPPED) ? "stopped" : (i->get_state() == CAN_FS_ACTIVE) ? "active"
                                                                                               : "unknown");
    }
}

bool CANObject::_copy_data_field_to_local(DataField &data_field, uint8_t dest_byte_offset)
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
    data_field.copy_data_to(data_pointer, remaining_data);

    return true;
}

DataField *CANObject::get_first_erroneous_data_field()
{
    if (!has_data_fields())
        return nullptr;

    for (DataField &i : _data_fields_list)
    {
        i.update_state();
        if (i.has_errors())
            return &i;
    }

    return nullptr;
}

// if function already exists and it has responding or blended type then existing one will be returned
// for automatic and indirect functions there are no such limitations
CANFunctionBase *CANObject::add_function(CAN_function_id_t id)
{
    bool is_responding_function = CANFunctionBase::is_responding_by_func_id(id);

    if (is_responding_function && has_function(id))
    {
        // return existing one only for responding or blended functions
        return get_function(id);
    }

    CANFunctionBase *cf = nullptr;
    switch (id)
    {
    case CAN_FUNC_SET_BOOL_IN:
        cf = new CANFunctionSet(this);
        // Next OK handler
        cf->set_next_ok_function(add_function(CAN_FUNC_SET_BOOL_OUT_OK));
        // Next Error handler
        cf->set_next_err_function(add_function(CAN_FUNC_SET_BOOL_OUT_ERR));
        break;

    case CAN_FUNC_SET_VALUE_IN:
        cf = new CANFunctionSet(this);
        // Next OK handler
        cf->set_next_ok_function(add_function(CAN_FUNC_SET_VALUE_OUT_OK));
        // Next Error handler
        cf->set_next_err_function(add_function(CAN_FUNC_SET_VALUE_OUT_ERR));
        break;

    case CAN_FUNC_REQUEST_IN:
        cf = new CANFunctionRequest(this);
        // Next OK handler
        cf->set_next_ok_function(add_function(CAN_FUNC_REQUEST_OUT_OK));
        // Next Error handler
        cf->set_next_err_function(add_function(CAN_FUNC_REQUEST_OUT_ERR));
        break;

    case CAN_FUNC_SET_BOOL_OUT_OK:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_SET_BOOL_OUT_OK);
        break;

    case CAN_FUNC_SET_BOOL_OUT_ERR:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_SET_BOOL_OUT_ERR);
        break;

    case CAN_FUNC_SET_VALUE_OUT_OK:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_SET_VALUE_OUT_OK);
        break;

    case CAN_FUNC_SET_VALUE_OUT_ERR:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_SET_VALUE_OUT_ERR);
        break;

    case CAN_FUNC_REQUEST_OUT_OK:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_REQUEST_OUT_OK);
        break;

    case CAN_FUNC_REQUEST_OUT_ERR:
        cf = new CANFunctionSimpleSender(this);
        cf->set_id(CAN_FUNC_REQUEST_OUT_ERR);
        break;

    case CAN_FUNC_TIMER_NORMAL:
        cf = new CANFunctionTimerNormal(this, UINT32_MAX);
        break;

    case CAN_FUNC_TIMER_ATTENTION:
    case CAN_FUNC_TIMER_CRITICAL:
        break;

    case CAN_FUNC_EVENT_ERROR:
        cf = new CANFunctionSimpleEvent(this, UINT32_MAX);
        break;

    case CAN_FUNC_NONE:
    default:
        return nullptr;
    }

    if (cf == nullptr)
        return nullptr;

    cf->set_parent(this);
    _functions_list.push_back(cf);

    return _functions_list.back();
}

CANFunctionBase *CANObject::get_function(CAN_function_id_t func_id)
{
    for (CANFunctionBase *i : _functions_list)
    {
        if (i->get_id() == func_id)
            return i;
    }

    return nullptr;
}

CANFunctionBase *CANObject::get_function(CANFunctionBase &function)
{
    for (CANFunctionBase *i : _functions_list)
    {
        if (*i == function)
            return i;
    }

    return nullptr;
}

bool CANObject::has_function(CAN_function_id_t func_id)
{
    return get_function(func_id) != nullptr;
}

bool CANObject::has_function(CANFunctionBase &function)
{
    return get_function(function) != nullptr;
}

uint8_t CANObject::get_functions_count()
{
    return _functions_list.size();
}

// only fills specified CANFrame with data from local storage
// no error state checks, data updates or other additional stuff
bool CANObject::fill_can_frame_with_data(CANFrame &can_frame, CAN_function_id_t func)
{
    if (_get_data_local() == nullptr)
        return false;

    *(uint8_t *)_get_data_local() = func;

    if (can_frame.get_max_data_length() < calculate_all_data_size())
        return false;

    can_frame.clear_frame();
    can_frame.set_frame(get_id(), (uint8_t *)_get_data_local(), calculate_all_data_size());

    return true;
}

bool CANObject::process_incoming_frame(CANFrame &can_frame)
{
    if (can_frame.get_id() != get_id())
        return false;

    for (CANFunctionBase *i : _functions_list)
    {
        if (i->get_id() == can_frame.get_function_id() && i->is_responding_function())
            i->process(&can_frame);
    }
    return true;
}