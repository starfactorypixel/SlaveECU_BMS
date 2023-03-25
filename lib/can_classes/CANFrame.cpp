#include "CANFrame.h"

/******************************************************************************************************************************
 * 
 * CANFrame: data frame of CAN bus
 * 
 ******************************************************************************************************************************/
CANFrame::CANFrame()
{
    clear_frame();
}

CANFrame::CANFrame(can_id_t id, uint8_t *data, uint8_t data_length)
{
    CANFrame();
    set_frame(id, data, data_length);
}

CANFrame::~CANFrame()
{
}

void CANFrame::set_frame(CANFrame &can_frame)
{
    set_frame(can_frame.get_id(), can_frame.get_data_pointer(), can_frame.get_data_length());
}

void CANFrame::set_frame(can_id_t id, uint8_t *data, uint8_t data_length)
{
    if (is_initialized())
        clear_frame();

    _id = id;
    _data_length = data_length;
    memcpy(_data, data, _data_length);
    _is_initialized = true;
}

void CANFrame::set_frame(can_id_t id, uint8_t data_length, uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4, uint8_t v5, uint8_t v6, uint8_t v7, uint8_t v8)
{
    if (is_initialized())
        clear_frame();

    _id = id;
    _data_length = data_length;
    _data[0] = v1;
    _data[1] = v2;
    _data[2] = v3;
    _data[3] = v4;
    _data[4] = v5;
    _data[5] = v6;
    _data[6] = v7;
    _data[7] = v8;
    _is_initialized = true;
}

void CANFrame::clear_frame()
{
    _id = 0;
    for (uint8_t i = 0; i < CAN_MAX_PAYLOAD; i++)
    {
        _data[i] = 0x00;
    }
    _data_length = 0;
    _is_initialized = false;
}

bool CANFrame::is_initialized()
{
    return _is_initialized;
}

can_id_t CANFrame::get_id()
{
    return _id;
}

uint8_t CANFrame::get_data_length()
{
    return _data_length;
}

uint8_t CANFrame::get_max_data_length()
{
    return CAN_MAX_PAYLOAD;
}

uint8_t *CANFrame::get_data_pointer()
{
    return _data;
}

bool CANFrame::copy_frame_data_to(uint8_t *destination, uint8_t max_dest_length)
{
    if (!has_data())
        return false;
    if (max_dest_length < get_data_length())
        return false;
    if (destination == nullptr)
        return false;

    memcpy(destination, get_data_pointer(), get_data_length());
    return true;
}

bool CANFrame::has_data()
{
    return get_data_length() > 0;
}

void CANFrame::print(const char *prefix)
{
    char buff[40];

    LOGwoN("%sCAN frame: [0x%04X] %d [", prefix, get_id(), get_data_length());

    for (uint8_t i = 0; i < CAN_MAX_PAYLOAD; i++)
    {
        if (i == 0)
        {
            _frame_func_to_string(buff);
        }
        else
        {
            sprintf(buff, "%d", get_data_pointer()[i]);
        }
        LOGstring("%s%s", (i >= get_data_length()) ? "-" : buff, (i == 7) ? "" : ", ");
    }

    LOGstring("]\n");
}

void CANFrame::_frame_func_to_string(char *dest_string)
{
    uint8_t func = 0xFF;

    if (get_data_length() > 0)
        func = get_data_pointer()[0];

    switch (func)
    {
    case CAN_FT_NONE:
        strcpy(dest_string, "CAN_FT_NONE");
        break;

    case CAN_FT_SET_BOOL_IN:
        strcpy(dest_string, "CAN_FT_SET_BOOL_IN");
        break;

    case CAN_FT_SET_BOOL_OUT_OK:
        strcpy(dest_string, "CAN_FT_SET_BOOL_OUT_OK");
        break;

    case CAN_FT_SET_BOOL_OUT_ERR:
        strcpy(dest_string, "CAN_FT_SET_BOOL_OUT_ERR");
        break;

    case CAN_FT_SET_VALUE_IN:
        strcpy(dest_string, "CAN_FT_SET_VALUE_IN");
        break;

    case CAN_FT_SET_VALUE_OUT_OK:
        strcpy(dest_string, "CAN_FT_SET_VALUE_OUT_OK");
        break;

    case CAN_FT_SET_VALUE_OUT_ERR:
        strcpy(dest_string, "CAN_FT_SET_VALUE_OUT_ERR");
        break;

    case CAN_FT_REQUEST_IN:
        strcpy(dest_string, "CAN_FT_REQUEST_IN");
        break;

    case CAN_FT_REQUEST_OUT_OK:
        strcpy(dest_string, "CAN_FT_REQUEST_OUT_OK");
        break;

    case CAN_FT_REQUEST_OUT_ERR:
        strcpy(dest_string, "CAN_FT_REQUEST_OUT_ERR");
        break;

    case CAN_FT_TIMER_NORMAL:
        strcpy(dest_string, "CAN_FT_TIMER_NORMAL");
        break;

    case CAN_FT_TIMER_ATTENTION:
        strcpy(dest_string, "CAN_FT_TIMER_ATTENTION");
        break;

    case CAN_FT_TIMER_CRITICAL:
        strcpy(dest_string, "CAN_FT_TIMER_CRITICAL");
        break;

    case CAN_FT_EVENT_ERROR:
        strcpy(dest_string, "CAN_FT_EVENT_ERROR");
        break;

    default:
        strcpy(dest_string, "UNKNOWN");
        break;
    }
}

/******************************************************************************************************************************
 * 
 * PixelCANFrame: Pixel specific CAN frame
 * 
 ******************************************************************************************************************************/
PixelCANFrame::PixelCANFrame() {
    _func = get_data_pointer();
}

PixelCANFrame::PixelCANFrame(can_id_t id, uint8_t *data, uint8_t data_length) : PixelCANFrame()
{
    CANFrame(id, data, data_length);
}

PixelCANFrame::~PixelCANFrame()
{
}

func_type_t PixelCANFrame::get_func()
{
    return (func_type_t)_func[0];
}

bool PixelCANFrame::set_func(func_type_t func)
{
    if (get_data_length() > 0)
    {
        _func[0] = (uint8_t)func;
        return true;
    }

    return false;
}
