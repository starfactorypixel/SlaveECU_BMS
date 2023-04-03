#include "CANManager.h"

/******************************************************************************************************************************
 *
 * CANManager class: implements CAN Manager interface
 *
 ******************************************************************************************************************************/
CANManager::CANManager(get_ms_tick_function_t tick_func)
{
    _can_objects_list.clear();

    set_tick_func(tick_func);
}

CANManager::~CANManager()
{
    _can_objects_list.clear();

    while (!_rx_frame_queue.empty())
    {
        _rx_frame_queue.pop();
    }
    while (!_tx_frame_queue.empty())
    {
        _tx_frame_queue.pop();
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
    for (CANObject &i : _can_objects_list)
    {
        co_update_result = i.update();
        result = result && co_update_result;
    }

    CANFrame cf;
    CANObject *co = nullptr;
    while (has_rx_frames_in_queue())
    {
        pop_rx_frame_from_queue(cf);
        co = get_can_object_by_can_id(cf.get_id());
        if (co == nullptr)
            return false;
        co->process_incoming_frame(cf);
    }

    return result;
}

void CANManager::print(const char *prefix)
{
    char str[100];
    LOG("%s***********************************************", prefix);
    LOG("%sCAN Manager data", prefix);

    // print CANObject data
    LOG("%s  CAN Objects:", prefix);
    for (CANObject &i : _can_objects_list)
    {
        i.print("  ");
    }

    // print RX queue items
    LOG("%s  RX queue:", prefix);
    uint16_t queue_item_counter = 0;
    std::queue<CANFrame> temp = _rx_frame_queue;
    while (!temp.empty())
    {
        queue_item_counter++;
        sprintf(str, "%s    #%d: ", prefix, queue_item_counter);
        temp.front().print(str);
        temp.pop();
    }
    if (queue_item_counter == 0)
    {
        LOG("%s    RX queue is empty.", prefix);
    }

    // print TX queue items
    LOG("%s  TX queue:", prefix);
    queue_item_counter = 0;
    temp = _tx_frame_queue;
    while (!temp.empty())
    {
        queue_item_counter++;
        sprintf(str, "%s    #%d: ", prefix, queue_item_counter);
        temp.front().print(str);
        temp.pop();
    }
    if (queue_item_counter == 0)
    {
        LOG("%s    TX queue is empty.", prefix);
    }

    LOG("%s***********************************************", prefix);
}

bool CANManager::take_new_rx_frame(CANFrame &can_frame)
{
    if (!has_can_object(can_frame.get_id()))
        return false; // we don't have such CANObject

    _rx_frame_queue.push(can_frame);

    return true;
}

bool CANManager::take_new_rx_frame(can_id_t id, uint8_t *data, uint8_t data_length)
{
    CANFrame new_frame(id, data, data_length);
    return take_new_rx_frame(new_frame);
}

bool CANManager::take_new_rx_frame(CAN_RxHeaderTypeDef &header, uint8_t aData[])
{
    CANFrame new_frame(header.StdId, aData, header.DLC);
    return take_new_rx_frame(new_frame);
}

bool CANManager::pop_rx_frame_from_queue(CANFrame &can_frame)
{
    if (!has_rx_frames_in_queue())
        return false;

    can_frame.set_frame(_rx_frame_queue.front());
    _rx_frame_queue.pop();

    return true;
}

uint8_t CANManager::get_rx_queue_size()
{
    return _rx_frame_queue.size();
}

uint8_t CANManager::get_tx_queue_size()
{
    return _tx_frame_queue.size();
}

bool CANManager::has_rx_frames_in_queue()
{
    return get_rx_queue_size() > 0;
}

bool CANManager::add_tx_queue_item(CANFrame &can_frame)
{
    _tx_frame_queue.push(can_frame);
    return true;
}

bool CANManager::has_tx_frames_for_transmission()
{
    return get_tx_queue_size() > 0;
}

bool CANManager::give_tx_frame(CANFrame &can_frame)
{
    if (!has_tx_frames_for_transmission())
        return false;

    can_frame.set_frame(_tx_frame_queue.front());
    _tx_frame_queue.pop();

    return true;
}

bool CANManager::give_tx_frame(can_id_t &id, uint8_t *data, uint8_t &data_length)
{
    if (!has_tx_frames_for_transmission())
        return false;

    CANFrame can_frame;
    give_tx_frame(can_frame);

    id = can_frame.get_id();
    data_length = can_frame.get_data_length();
    memcpy(data, can_frame.get_data_pointer(), data_length);

    return true;
}

bool CANManager::give_tx_frame(CAN_TxHeaderTypeDef &header, uint8_t aData[])
{
    if (!has_tx_frames_for_transmission())
        return false;

    CANFrame can_frame;
    give_tx_frame(can_frame);

    header.DLC = can_frame.get_data_length();
    header.StdId = can_frame.get_id();
    can_frame.copy_frame_data_to(aData, 8);

    return true;
}

uint8_t CANManager::get_can_objects_count()
{
    return _can_objects_list.size();
}

CANObject *CANManager::add_can_object(can_id_t id, const char *name)
{
    CANObject *new_can_object = get_can_object_by_can_id(id);

    if (new_can_object != nullptr)
        return new_can_object;

    CANObject co;
    _can_objects_list.push_back(co);

    CANObject *pco = &_can_objects_list.back();
    pco->set_id(id);
    pco->set_parent(*this);
    pco->update_state();
    // TODO: we are setting name here because in CANObject name is dynamic char array and it will not be copied due the insertion into the list.
    // The other one member with the same problem is CANObject::_functions_list because it stores pointers.
    // It can be fixed by implementation of a copy constructor and operator= (assignment). It may be done later if we will need it.
    // Other class members are correctly copied by default stuff
    pco->set_name(name);

    return pco;
}

CANObject *CANManager::get_can_object_by_index(uint8_t index)
{
    if (index >= get_can_objects_count())
        return nullptr;

    std::list<CANObject>::iterator it = _can_objects_list.begin();
    std::advance(it, index);

    return &(*it);
}

CANObject *CANManager::get_can_object_by_can_id(can_id_t id)
{
    if (!has_can_objects())
        return nullptr;

    for (CANObject &i : _can_objects_list)
    {
        if (id == i.get_id())
            return &i;
    }

    return nullptr;
}

bool CANManager::delete_can_object(can_id_t id)
{
    if (!has_can_objects())
        return false;

    for (CANObject &i : _can_objects_list)
    {
        if (id == i.get_id())
        {
            _can_objects_list.remove(i);
            return true;
        }
    }
    return false;
}

bool CANManager::has_can_objects()
{
    return (get_can_objects_count() != 0);
}

bool CANManager::has_can_object(can_id_t id)
{
    return (nullptr != get_can_object_by_can_id(id));
}
