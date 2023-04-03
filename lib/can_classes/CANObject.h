#ifndef CANOBJECT_H
#define CANOBJECT_H
// #pragma once

#include <stdint.h>
#include <list>

#include "CAN_common.h"
#include "DataField.h"
#include "CANFunction.h"
#include "CANFrame.h"
#include "CANManager.h"

class CANManager;
class CANFunctionInterface;
class CANFunctionBase;

/******************************************************************************************************************************
 *
 * CANObject class: CAN data object with ID and several DataFields
 *
 ******************************************************************************************************************************/
class CANObject
{
public:
    CANObject();
    CANObject(uint16_t id, CANManager &parent, const char *name = nullptr);
    ~CANObject();

    bool operator==(const CANObject &frame);

    can_id_t get_id();
    void set_id(can_id_t id, const char *name = nullptr);

    CANManager *get_parent();
    void set_parent(CANManager &parent);

    uint32_t get_tick();

    uint8_t get_data_fields_count();
    bool has_data_fields();

    DataField *add_data_field();
    DataField *add_data_field(data_field_t type, void *data, uint32_t array_item_count = 1);
    bool delete_data_field(uint8_t index);
    DataField *get_data_field(uint8_t index);
    bool has_data_fields_alarm();

    uint8_t calculate_all_data_size();

    can_object_state_t get_state();
    const char *get_state_name();
    bool is_state_ok();
    DataField *get_first_erroneous_data_field();

    // if function already exists and it has responding or blended type then existing one will be returned
    // for automatic and indirect functions there are no such limitations
    CANFunctionBase *add_function(CAN_function_id_t id);
    CANFunctionBase *get_function(CAN_function_id_t func_id);
    CANFunctionBase *get_function(CANFunctionBase &function);
    bool has_function(CAN_function_id_t func_id);
    bool has_function(CANFunctionBase &function);
    uint8_t get_functions_count();

    const char *get_name();
    void set_name(const char *name);
    bool has_name();
    void delete_name();

    // only updates states of data fields and transfer CANObject in error state if there is at least one erroneous DataField
    can_object_state_t update_state();
    // updates local data storage, performs active and automatic/blended functions calls
    bool update();
    // updates local data storage only
    bool update_local_data();

    void print(const char *prefix);

    // only fills specified CANFrame with data from local storage; no error state checks, data updates or other additional stuff
    bool fill_can_frame_with_data(CANFrame &can_frame, CAN_function_id_t func);

    // it goes through all CANFunctions and perform their process(CANFrame) if function id matches
    bool process_incoming_frame(CANFrame &can_frame);

protected:
    void _set_state(can_object_state_t state);

    bool _delete_data_local();
    bool _resize_data_local(uint8_t new_size);
    bool _fit_data_local_to_data_fields();
    void *_get_data_local();

    bool _copy_data_field_to_local(DataField &data_field, uint8_t dest_byte_offset);

private:
    can_id_t _id = 0;
    CANManager *_parent = nullptr;
    can_object_state_t _state = COS_UNKNOWN_ERROR;

    std::list<DataField> _data_fields_list;
    std::list<CANFunctionBase *> _functions_list;

    uint8_t *_data_local = nullptr;

    // object name for logging
    char *_name = nullptr;

    // 'unknown' for logging
    static const char *_value_unknown;
    // the object state names for logging
    static const char *_state_object_ok;
    static const char *_state_data_field_error;
    static const char *_state_local_data_buffer_size_error;
    static const char *_state_unknown_error;
};

#endif // CANOBJECT_H