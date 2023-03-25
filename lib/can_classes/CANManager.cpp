#include "CANManager.h"

/******************************************************************************************************************************
 *
 * CANManager class: implements CAN Manager interface
 *
 ******************************************************************************************************************************/
/*
// No default constructor because we should have the tick_function!
CANManager::CANManager()
{
    _rx_can_frame.clear_frame();
    _tx_can_frame.clear_frame();

    _can_objects = nullptr;
    _can_objects_count = 0;

    _tick_func = nullptr;
}
*/

CANManager::CANManager(get_ms_tick_function_t tick_func)
{
    _rx_can_frame.clear_frame();
    _tx_can_frame.clear_frame();

    _can_objects = nullptr;
    _can_objects_count = 0;

    set_tick_func(tick_func);
}

CANManager::~CANManager()
{
    if (has_can_objects())
    {
        for (uint8_t i = 0; i < get_can_objects_count(); i++)
        {
            delete _can_objects[i];
        }
        delete[] _can_objects;
    }
}

void CANManager::set_tick_func(get_ms_tick_function_t tick_func)
{
    _tick_func = tick_func;
}

uint32_t CANManager::get_tick()
{
    if (_tick_func != nullptr)
        return _tick_func();

    return 0;
}

bool CANManager::process()
{
    bool result = true;
    bool co_update_result = true;
    CANObject *co = nullptr;
    for (uint8_t i = 0; i < get_can_objects_count(); i++)
    {
        co = get_can_object_by_index(i);
        // result = result && get_can_object_by_index(i)->update();
        co_update_result = co->update();
        result = result && co_update_result;

        if (co->has_data_to_send())
        {
            PixelCANFrame new_can_frame;
            co->fill_can_frame(new_can_frame, CAN_FT_NONE);
            _tx_can_frame.set_frame(new_can_frame); // TODO: HERE!
        }
    }

    return result;
}

void CANManager::print(const char *prefix)
{
    char str[100];
    LOG("%s***********************************************", prefix);
    LOG("%sCAN Manager data", prefix);
    sprintf(str, "%sRX CAN frame: ", prefix);
    _rx_can_frame.print(str);
    sprintf(str, "%sTX CAN frame: ", prefix);
    _tx_can_frame.print(str);
    LOG("%sCAN Objects:", prefix);
    for (uint8_t i = 0; i < get_can_objects_count(); i++)
    {
        sprintf(str, "%sCAN Object #%d: ", prefix, i);
        get_can_object_by_index(i)->print(str);
    }
    LOG("%s***********************************************", prefix);
}

bool CANManager::take_new_rx_frame(CANFrame &can_frame)
{
    _rx_can_frame.set_frame(can_frame);

    can_frame.print("[CAN Manager] new RX frame: ");
    return true;
}

bool CANManager::take_new_rx_frame(can_id_t id, uint8_t *data, uint8_t data_length)
{
    _rx_can_frame.set_frame(id, data, data_length);
    take_new_rx_frame(_rx_can_frame);
    return true;
}

bool CANManager::take_new_rx_frame(CAN_RxHeaderTypeDef &header, uint8_t aData[])
{
    // TODO: Hardware dependent! Should process standard or extended frames.
    // Probably this behaviour should be placed outside the class. Or we need to implement it here.
    _rx_can_frame.set_frame(header.StdId, aData, header.DLC);
    take_new_rx_frame(_rx_can_frame);
    return true;
}

bool CANManager::has_tx_frames_for_transmission()
{
    return _tx_can_frame.is_initialized();
}

bool CANManager::give_tx_frame(CANFrame &can_frame)
{
    LOG("give_tx_frame(&can_frame)");
    can_frame.set_frame(_tx_can_frame);
    can_frame.print("[CAN Manager] new TX frame: ");

    _tx_can_frame.clear_frame();

    return false;
}

bool CANManager::give_tx_frame(can_id_t &id, uint8_t *data, uint8_t &data_length)
{
    LOG("give_tx_frame(&id, *data, &data_length)");
    id = _tx_can_frame.get_id();
    data_length = _tx_can_frame.get_data_length();
    memcpy(data, _tx_can_frame.get_data_pointer(), data_length);

    _tx_can_frame.clear_frame();

    return false;
}

bool CANManager::give_tx_frame(CAN_TxHeaderTypeDef &header, uint8_t aData[])
{
    // TODO: Hardware dependent! Should process standard or extended frames.
    // Probably this behaviour should be placed outside the class. Or we need to implement it here.
    LOG("give_tx_frame(*pHeader, aData[])");
    header.DLC = _tx_can_frame.get_data_length();
    header.StdId = _tx_can_frame.get_id();
    _tx_can_frame.copy_frame_data_to(aData, 8);

    _tx_can_frame.clear_frame();

    return false;
}

uint8_t CANManager::get_can_objects_count()
{
    return _can_objects_count;
}

CANObject *CANManager::add_can_object()
{
    CANObject **new_can_objects = new CANObject *[get_can_objects_count() + 1];
    memcpy(new_can_objects, _can_objects, sizeof(CANObject *) * get_can_objects_count());
    new_can_objects[get_can_objects_count()] = new CANObject;
    new_can_objects[get_can_objects_count()]->set_tick_func(_tick_func);
    new_can_objects[get_can_objects_count()]->update_state();

    if (has_can_objects())
    {
        delete[] _can_objects;
    }
    _can_objects = new_can_objects;
    _can_objects_count++;

    return _can_objects[get_can_objects_count() - 1];
}

CANObject *CANManager::add_can_object(can_id_t id)
{
    CANObject *new_can_object = get_can_object_by_can_id(id);

    if (new_can_object == nullptr)
    {
        new_can_object = add_can_object();
        new_can_object->set_id(id);
        new_can_object->update_state();
    }
    return new_can_object;
}

CANObject *CANManager::get_can_object_by_index(uint8_t index)
{
    if (index >= get_can_objects_count())
        return nullptr;

    return _can_objects[index];
}

CANObject *CANManager::get_can_object_by_can_id(can_id_t id)
{
    for (uint8_t i = 0; i < get_can_objects_count(); i++)
    {
        if (id == get_can_object_by_index(i)->get_id())
            return get_can_object_by_index(i);
    }

    return nullptr;
}

bool CANManager::delete_can_object(can_id_t id)
{
    if (!has_can_object(id))
        return false;

    uint8_t index = 0;
    if (!_get_can_object_index(id, index))
        return false;

    CANObject **new_can_objects = new CANObject *[get_can_objects_count() - 1];

    memcpy(new_can_objects, _can_objects, sizeof(CANObject *) * index);
    memcpy(new_can_objects + index, _can_objects + index + 1, sizeof(CANObject *) * (get_can_objects_count() - index - 1));
    delete _can_objects[index];
    delete[] _can_objects;
    _can_objects = new_can_objects;
    _can_objects_count--;

    return true;
}

bool CANManager::has_can_objects()
{
    return (_can_objects != nullptr && get_can_objects_count() != 0);
}

bool CANManager::has_can_object(can_id_t id)
{
    return (nullptr != get_can_object_by_can_id(id));
}

bool CANManager::_get_can_object_index(can_id_t id, uint8_t &index)
{
    for (uint8_t i = 0; i < get_can_objects_count(); i++)
    {
        if (id == get_can_object_by_index(i)->get_id())
        {
            index = i;
            return true;
        }
    }

    return false;
}