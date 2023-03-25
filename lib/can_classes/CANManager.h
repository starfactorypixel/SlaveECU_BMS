#ifndef CANMANAGER_H
#define CANMANAGER_H
//#pragma once

#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "CAN_common.h"
#include "CANFrame.h"
#include "CANObject.h"

/******************************************************************************************************************************
 *
 * CANManagerInterface: interface of CAN manager
 * The CAN manager stores and manages CAN objects.
 * Provides data transport from the CAN bus to the CAN objects and vice versa.
 * 
 * The interface was created only to reduce the number of methods available for viewing in the main program.
 *
 ******************************************************************************************************************************/
class CANManagerInterface
{
public:
    virtual ~CANManagerInterface() = default;

    virtual void set_tick_func(get_ms_tick_function_t tick_func) = 0;
    virtual uint32_t get_tick() = 0;

    // rx frames processing, tx frames preparing 
    virtual bool process() = 0;
    virtual void print(const char *prefix) = 0;

    virtual bool take_new_rx_frame(CANFrame &can_frame) = 0;
    virtual bool take_new_rx_frame(can_id_t id, uint8_t *data, uint8_t data_length) = 0;
    virtual bool take_new_rx_frame(CAN_RxHeaderTypeDef &header, uint8_t aData[]) = 0;

    virtual bool has_tx_frames_for_transmission() = 0;

    virtual bool give_tx_frame(CANFrame &can_frame) = 0;
    virtual bool give_tx_frame(can_id_t &id, uint8_t *data, uint8_t &data_length) = 0;
    virtual bool give_tx_frame(CAN_TxHeaderTypeDef &header, uint8_t aData[]) = 0;

    virtual uint8_t get_can_objects_count() = 0;
    virtual bool has_can_objects() = 0;
    virtual bool has_can_object(can_id_t id) = 0;
    virtual CANObject *add_can_object() = 0;
    virtual CANObject *add_can_object(can_id_t id) = 0;
    virtual CANObject *get_can_object_by_index(uint8_t index) = 0;
    virtual CANObject *get_can_object_by_can_id(can_id_t id) = 0;
    virtual bool delete_can_object(can_id_t id) = 0;

};


/******************************************************************************************************************************
 * 
 * CANManager class: implements CAN Manager interface
 * 
 ******************************************************************************************************************************/
class CANManager : CANManagerInterface
{
public:
    //CANManager();
    CANManager(get_ms_tick_function_t tick_func);
    ~CANManager() override;

    void set_tick_func(get_ms_tick_function_t tick_func) override;
    uint32_t get_tick() override;

    bool process() override;
    void print(const char *prefix) override;

    bool take_new_rx_frame(CANFrame &can_frame) override;
    bool take_new_rx_frame(can_id_t id, uint8_t *data, uint8_t data_length) override;
    bool take_new_rx_frame(CAN_RxHeaderTypeDef &header, uint8_t aData[]) override;

    bool has_tx_frames_for_transmission() override;

    bool give_tx_frame(CANFrame &can_frame) override;
    bool give_tx_frame(can_id_t &id, uint8_t *data, uint8_t &data_length) override;
    bool give_tx_frame(CAN_TxHeaderTypeDef &header, uint8_t aData[]) override;

    uint8_t get_can_objects_count() override;
    bool has_can_objects() override;
    bool has_can_object(can_id_t id) override;
    CANObject *add_can_object() override;
    CANObject *add_can_object(can_id_t id) override;
    CANObject *get_can_object_by_index(uint8_t index) override;
    CANObject *get_can_object_by_can_id(can_id_t id) override;
    bool delete_can_object(can_id_t id) override;

protected:
    CANFrame _rx_can_frame;
    CANFrame _tx_can_frame;

    bool _get_can_object_index(can_id_t id, uint8_t &index);

private:
    CANObject **_can_objects = nullptr;
    uint8_t _can_objects_count = 0;

    get_ms_tick_function_t _tick_func = nullptr;
};

#endif // CANMANAGER_H