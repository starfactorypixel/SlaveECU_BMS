#include "CANFunction.h"

/******************************************************************************************************************************
 *
 * CANFunction base class: base class for all CAN functions
 *
 ******************************************************************************************************************************/
const char *CANFunctionBase::_state_function_stopped = "stopped";
const char *CANFunctionBase::_state_function_active = "active";
const char *CANFunctionBase::_value_unknown = "unknown";

const char *CANFunctionBase::_type_function_responding = "responding";
const char *CANFunctionBase::_type_function_automatic = "automatic";
const char *CANFunctionBase::_type_function_blended = "blended";
const char *CANFunctionBase::_type_function_indirect = "indirect";

CANFunctionBase::CANFunctionBase(CAN_function_id_t id,
                                 CANObject *parent,
                                 CAN_function_handler_t external_handler,
                                 CANFunctionBase *next_ok_function,
                                 CANFunctionBase *next_err_function)
    : _id(id), _state(CAN_FS_STOPPED), _parent(parent), _type(CAN_FT_INDIRECT), _external_handler(external_handler),
      _next_ok_function(next_ok_function), _next_err_function(next_err_function), _name(nullptr)
{
}

CANFunctionBase::~CANFunctionBase()
{
    delete_name();
}

bool CANFunctionBase::operator==(const CANFunctionBase &other)
{
    return this->_equals(other);
}

bool CANFunctionBase::operator!=(const CANFunctionBase &other)
{
    return !(*this == other);
}

bool CANFunctionBase::_equals(CANFunctionBase const &other) const
{
    if (typeid(*this) != typeid(other))
        return false;

    return (this->_id == other._id);
}

void CANFunctionBase::set_id(CAN_function_id_t id, const char *name)
{
    _id = id;
    set_name(name);
}

CAN_function_id_t CANFunctionBase::get_id()
{
    return _id;
}

void CANFunctionBase::set_parent(CANObject *parent)
{
    _parent = parent;
}

CANObject *CANFunctionBase::get_parent()
{
    return _parent;
}

void CANFunctionBase::set_external_handler(CAN_function_handler_t external_handler)
{
    _external_handler = external_handler;
}

CAN_function_handler_t CANFunctionBase::get_external_handler()
{
    return _external_handler;
}

bool CANFunctionBase::has_external_handler()
{
    return get_external_handler() != nullptr;
}

void CANFunctionBase::set_next_ok_function(CANFunctionBase *next_ok_function)
{
    _next_ok_function = next_ok_function;
}

CANFunctionBase *CANFunctionBase::get_next_ok_function()
{
    return _next_ok_function;
}

bool CANFunctionBase::has_next_ok_function()
{
    return get_next_ok_function() != nullptr;
}

void CANFunctionBase::set_next_err_function(CANFunctionBase *next_err_function)
{
    _next_err_function = next_err_function;
}

CANFunctionBase *CANFunctionBase::get_next_err_function()
{
    return _next_err_function;
}

bool CANFunctionBase::has_next_err_function()
{
    return get_next_err_function() != nullptr;
}

void CANFunctionBase::disable()
{
    _set_state(CAN_FS_STOPPED);
}

void CANFunctionBase::enable()
{
    _set_state(CAN_FS_ACTIVE);
}

bool CANFunctionBase::is_active()
{
    return get_state() == CAN_FS_ACTIVE;
}

CAN_function_state_t CANFunctionBase::get_state()
{
    return _state;
}

const char *CANFunctionBase::get_state_name()
{
    switch (get_state())
    {
    case CAN_FS_STOPPED:
        return _state_function_stopped;

    case CAN_FS_ACTIVE:
        return _state_function_active;

    default:
        return _value_unknown;
    }
}

const char *CANFunctionBase::get_name()
{
    return _name;
}

void CANFunctionBase::set_name(const char *name)
{
    if (name == nullptr)
        return;

    _name = new char[strlen(name) + 1];
    strcpy(_name, name);
}

bool CANFunctionBase::has_name()
{
    return get_name() != nullptr;
}

void CANFunctionBase::delete_name()
{
    if (has_name())
        delete[] _name;
}

void CANFunctionBase::_set_state(CAN_function_state_t state)
{
    _state = state;
}

// function main handler
bool CANFunctionBase::process(CANFrame *can_frame)
{
    if (!is_active())
        return false;

    CAN_function_result_t result = CAN_RES_NONE;

    if (has_external_handler())
    {
        result = _before_external_handler(can_frame);
        if (result == CAN_RES_NEXT_OK)
        {
            result = (get_external_handler())(*get_parent(), *this, can_frame);
            result = _after_external_handler(result, can_frame);
        }
    }
    else
    {
        result = _default_handler(can_frame);
    }

    if (result == CAN_RES_NEXT_OK && has_next_ok_function())
    {
        get_next_ok_function()->process(can_frame);
    }
    else if (result == CAN_RES_NEXT_ERR && has_next_err_function())
    {
        get_next_err_function()->process(can_frame);
    }

    return result != CAN_RES_NONE;
}

bool CANFunctionBase::is_responding_function()
{
    return CAN_FT_RESPONDING == (get_type() & CAN_FT_RESPONDING);
}

bool CANFunctionBase::is_automatic_function()
{
    return CAN_FT_AUTOMATIC == (get_type() & CAN_FT_AUTOMATIC);
}

bool CANFunctionBase::is_indirect_function()
{
    return CAN_FT_INDIRECT == (get_type() & CAN_FT_INDIRECT);
}

void CANFunctionBase::set_type(CAN_function_type_t func_type)
{
    _type = func_type;
}

CAN_function_type_t CANFunctionBase::get_type()
{
    return _type;
}

const char *CANFunctionBase::get_type_name()
{
    switch (get_type())
    {
    case CAN_FT_RESPONDING:
        return _type_function_responding;

    case CAN_FT_AUTOMATIC:
        return _type_function_automatic;

    case CAN_FT_BLENDED:
        return _type_function_blended;

    case CAN_FT_INDIRECT:
        return _type_function_indirect;

    default:
        return _value_unknown;
    }
}

bool CANFunctionBase::is_responding_by_func_id(CAN_function_id_t id)
{
    switch (id)
    {
    case CAN_FUNC_SET_IN:
    case CAN_FUNC_REQUEST_IN:
        return true;

    case CAN_FUNC_SET_OUT_OK:
    case CAN_FUNC_SET_OUT_ERR:
    case CAN_FUNC_REQUEST_OUT_OK:
    case CAN_FUNC_REQUEST_OUT_ERR:
    case CAN_FUNC_TIMER_NORMAL:
    case CAN_FUNC_TIMER_ATTENTION:
    case CAN_FUNC_TIMER_CRITICAL:
    case CAN_FUNC_SIMPLE_SENDER:
    case CAN_FUNC_EVENT_ERROR:

    case CAN_FUNC_NONE:
    default:
        return false;
    }
}

void CANFunctionBase::print(const char *prefix)
{
    LOG("%sFunction: id = 0x%02X (%s), type = %s, state = %s", prefix, get_id(),
        has_name() ? get_name() : "noname", get_type_name(), get_state_name());
}

void CANFunctionBase::_fill_error_can_frame(CANFrame &can_frame, pixel_error_codes_t error_code, uint8_t additional_error_code)
{
    uint8_t error_data[CAN_MAX_PAYLOAD] = {0};
    error_data[0] = get_id(); // by default id will be rewrited by sender function

    uint8_t idx = 1;
    memcpy(&(error_data[idx]), &error_code, sizeof(error_code));
    idx += sizeof(error_code);
    memcpy(&(error_data[idx]), &additional_error_code, sizeof(additional_error_code));
    idx += sizeof(additional_error_code);

    can_frame.clear_frame();
    can_frame.set_frame(get_parent()->get_id(), error_data, idx);
}

/******************************************************************************************************************************
 *
 * CANFunctionTimerBase: class for timed messages
 *
 ******************************************************************************************************************************/
CANFunctionTimerBase::CANFunctionTimerBase(CAN_function_id_t id,
                                           CANObject *parent,
                                           uint32_t period_ms,
                                           CAN_function_handler_t external_handler,
                                           CANFunctionBase *next_ok_function,
                                           CANFunctionBase *next_err_function)
    : CANFunctionBase(id, parent, external_handler, next_ok_function, next_err_function),
      _period_ms(period_ms), _last_action_tick(0)
{
    set_type(CAN_FT_AUTOMATIC);
}

bool CANFunctionTimerBase::_equals(CANFunctionBase const &other) const
{
    if (typeid(*this) != typeid(other))
        return false;

    auto that = static_cast<CANFunctionTimerBase const &>(other);
    if (this->_period_ms != that._period_ms)
        return false;
    return CANFunctionBase::_equals(other);
}

void CANFunctionTimerBase::set_period(uint32_t period_ms)
{
    _period_ms = period_ms;
}

uint32_t CANFunctionTimerBase::get_period()
{
    return _period_ms;
}

CAN_function_result_t CANFunctionTimerBase::_default_handler(CANFrame *can_frame)
{
    if (get_parent()->get_tick() - _last_action_tick >= get_period())
    {
        _last_action_tick = get_parent()->get_tick();
        return _timer_handler();
    }

    return CAN_RES_NONE;
}

CAN_function_result_t CANFunctionTimerBase::_before_external_handler(CANFrame *can_frame)
{
    if (get_parent()->get_tick() - _last_action_tick >= get_period())
    {
        _last_action_tick = get_parent()->get_tick();
        return CAN_RES_NEXT_OK;
    }

    return CAN_RES_FINAL;
}

/******************************************************************************************************************************
 *
 * CANFunctionTimerNormal: class for normal timed messages
 *
 ******************************************************************************************************************************/
CANFunctionTimerNormal::CANFunctionTimerNormal(CANObject *parent, uint32_t period_ms, CAN_function_handler_t external_handler,
                                               CANFunctionBase *next_ok_function, CANFunctionBase *next_err_function)
    : CANFunctionTimerBase(CAN_FUNC_TIMER_NORMAL, parent, period_ms, external_handler, next_ok_function, next_err_function)
{
    set_name("CANFunctionTimerNormal");
    enable();
}

CAN_function_result_t CANFunctionTimerNormal::_timer_handler()
{
    CANFrame can_frame;
    CANObject &can_object = *get_parent();
    can_object.fill_can_frame_with_data(can_frame, get_id());

    can_frame.print("CANFunctionTimerNormal: ");

    CANManager &can_manager = *can_object.get_parent();
    can_manager.add_tx_queue_item(can_frame);

    return CAN_RES_FINAL;
}

/******************************************************************************************************************************
 *
 * CANFunctionRequest: class for incoming request
 *
 ******************************************************************************************************************************/
CANFunctionRequest::CANFunctionRequest(CANObject *parent, CAN_function_handler_t external_handler,
                                       CANFunctionBase *next_ok_function, CANFunctionBase *next_err_function)
    : CANFunctionBase(CAN_FUNC_REQUEST_IN, parent, external_handler, next_ok_function, next_err_function)
{
    set_type(CAN_FT_RESPONDING);
    set_name("CANFunctionRequest");
    enable();
}

CAN_function_result_t CANFunctionRequest::_default_handler(CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    can_frame->print("CANFunctionRequest [incoming frame]: ");

    if (can_frame->get_data_length() != 1)
    {
        _fill_error_can_frame(*can_frame, PIX_ERR_CAN_FRAME, CAN_FRAME_SIZE_ERROR);
        return CAN_RES_NEXT_ERR;
    }

    CANObject &co = *get_parent();
    can_object_state_t co_state = co.update_state();

    if (co.is_state_ok() && co.update_local_data())
    {
        co.fill_can_frame_with_data(*can_frame, get_id());
        return CAN_RES_NEXT_OK;
    }

    // it is possible that CANObject.update_local_data() returns false
    // in this case we will send PIX_ERR_CAN_OBJECT error with additional error code = COS_OK
    _fill_error_can_frame(*can_frame, PIX_ERR_CAN_OBJECT, co_state);
    return CAN_RES_NEXT_ERR;
}

/******************************************************************************************************************************
 *
 * CANFunctionSimpleSender: just sends the CAN frame specified in the _default_handler() parameters,
 * overrides specified in the CAN frame function ID with value, returned by get_id()
 * May be useful as the base class for all functions with outcoming messages. But it is very useful itself.
 *
 ******************************************************************************************************************************/
CANFunctionSimpleSender::CANFunctionSimpleSender(CANObject *parent, CAN_function_handler_t external_handler,
                                                 CANFunctionBase *next_ok_function, CANFunctionBase *next_err_function)
    : CANFunctionBase(CAN_FUNC_SIMPLE_SENDER, parent, external_handler, next_ok_function, next_err_function)
{
    set_type(CAN_FT_INDIRECT);
    set_name("CANFunctionSimpleSender");
    enable();
}

CAN_function_result_t CANFunctionSimpleSender::_default_handler(CANFrame *can_frame)
{
    CANFrame cf;
    CAN_function_result_t result = CAN_RES_NONE;

    if (can_frame == nullptr)
    {
        result = CAN_RES_NEXT_ERR;
        _fill_error_can_frame(cf, PIX_ERR_CAN_FRAME, CAN_FRAME_IS_NULL);
    }
    else
    {
        cf.set_frame(*can_frame);
        result = CAN_RES_NEXT_OK;
    }

    cf.set_function_id(get_id());
    cf.print("CANFunctionSimpleSender: ");

    CANObject &co = *get_parent();
    CANManager &cm = *co.get_parent();
    cm.add_tx_queue_item(cf);

    return result;
}

/******************************************************************************************************************************
 *
 * CANFunctionSimpleEvent: class for events
 *
 ******************************************************************************************************************************/
CANFunctionSimpleEvent::CANFunctionSimpleEvent(CANObject *parent, uint32_t period_ms,
                                               CAN_function_handler_t external_handler,
                                               CANFunctionBase *next_ok_function, CANFunctionBase *next_err_function)
    : CANFunctionTimerBase(CAN_FUNC_EVENT_ERROR, parent, period_ms, external_handler, next_ok_function, next_err_function)
{
    set_name("CANFunctionSimpleEvent");
    enable();
}

// decorator for CANFunctionTimerBase::_default_handler
// before timeout _last_action_tick update we should check if there any alarm
CAN_function_result_t CANFunctionSimpleEvent::_default_handler(CANFrame *can_frame)
{
    CANObject &can_object = *get_parent();

    if (!can_object.has_data_fields_alarm())
        return CAN_RES_FINAL;

    return CANFunctionTimerBase::_default_handler(can_frame);
}

// virtual function for correct and systematic comparison of derived classes
// should be complement by derived class
CAN_function_result_t CANFunctionSimpleEvent::_timer_handler()
{
    CANObject &can_object = *get_parent();
    CANFrame can_frame;
    can_object.update_local_data();
    can_object.fill_can_frame_with_data(can_frame, get_id());

    can_frame.print("CANFunctionSimpleEvent: ");

    CANManager &can_manager = *can_object.get_parent();
    can_manager.add_tx_queue_item(can_frame);

    return CAN_RES_FINAL;
}

/******************************************************************************************************************************
 *
 * CANFunctionSet: class for setter
 *
 ******************************************************************************************************************************/
CANFunctionSet::CANFunctionSet(CANObject *parent, CAN_function_handler_t external_handler,
                               CANFunctionBase *next_ok_function, CANFunctionBase *next_err_function)
    : CANFunctionBase(CAN_FUNC_SET_IN, parent, external_handler, next_ok_function, next_err_function)
{
    set_type(CAN_FT_RESPONDING);
    set_name("CANFunctionSet");
    enable();
}

// should return CAN_RES_NEXT_OK for external handler call performing
CAN_function_result_t CANFunctionSet::_before_external_handler(CANFrame *can_frame)
{
    // if we are here then there is an external handler specified, all checks was performed in CANFunctionBase::process() method
    // But we should check can_frame anyway because of possibility of external handler incorrect work
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    can_frame->print("CANFunctionSet [incoming frame]: ");

    CANObject &co = *get_parent();
    if (co.get_data_fields_count() != 1)
    {
        // only objects with exactly one data field are writable
        _fill_error_can_frame(*can_frame, PIX_ERR_FUNCTION, CAN_FUNC_ERROR_READONLY_OBJECT);
        return CAN_RES_NEXT_ERR;
    }

    DataField *df = co.get_data_field(0);

    if (can_frame->get_data_length() != df->get_data_byte_array_length() + 1)
    {
        _fill_error_can_frame(*can_frame, PIX_ERR_CAN_FRAME, CAN_FRAME_SIZE_ERROR);
        return CAN_RES_NEXT_ERR;
    }

    return CAN_RES_NEXT_OK;
}

// the return value of _after_external_handler() overwrites value, returned by the external handler
// if you don't want change it, just return the same value
CAN_function_result_t CANFunctionSet::_after_external_handler(CAN_function_result_t external_handler_result, CANFrame *can_frame)
{
    // can_frame was checked by _before_external_handler()... but external handler may delete it or do something another bad
    // in this case we shouldn't return external_handler_result
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    if (external_handler_result == CAN_RES_NEXT_OK)
    {
        CANObject &co = *get_parent();
        can_object_state_t co_state = co.update_state();

        if (co.is_state_ok() && co.update_local_data())
        {
            co.fill_can_frame_with_data(*can_frame, get_id());
            return CAN_RES_NEXT_OK;
        }
        else
        {
            // something wrong with data update
            external_handler_result = CAN_RES_NEXT_ERR;
            _fill_error_can_frame(*can_frame, PIX_ERR_CAN_OBJECT, co_state);
        }
    }
    else if (external_handler_result == CAN_RES_NEXT_ERR)
    {
        _fill_error_can_frame(*can_frame, PIX_ERR_FUNCTION, CAN_FUNC_ERROR_UNKNOWN_SETTER_ERROR);
    }

    return external_handler_result;
}

CAN_function_result_t CANFunctionSet::_default_handler(CANFrame *can_frame)
{
    // if the default handler is called then there is no external handler specified
    // this is an error situation
    _fill_error_can_frame(*can_frame, PIX_ERR_FUNCTION, CAN_FUNC_ERROR_NO_EXTERNAL_HANDLER);
    return CAN_RES_NEXT_ERR;
}
