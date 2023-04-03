#ifndef CAN_COMMON_H
#define CAN_COMMON_H
// #pragma once

#include <stdint.h>
#include <queue>
#include <list>
#include <typeinfo>

/******************************************************************************************************************************
 *
 * Common CAN related types
 *
 ******************************************************************************************************************************/
using get_ms_tick_function_t = uint32_t (*)();

/******************************************************************************************************************************
 *
 * DataField related data types
 *
 ******************************************************************************************************************************/
union data_mapper_t
{
    uint8_t u8arr[4];
    int8_t i8arr[4];
    uint8_t u8;
    int8_t i8;
    uint16_t u16arr[2];
    int16_t i16arr[2];
    uint16_t u16;
    int16_t i16;
    uint32_t u32;
    int32_t i32;
};

enum data_field_t : uint8_t
{
    DF_UNKNOWN = 0x00,
    DF_INT8 = 0x01,
    DF_UINT8 = 0x02,
    DF_INT16 = 0x03,
    DF_UINT16 = 0x04,
    DF_INT32 = 0x05,
    DF_UINT32 = 0x06,
};

enum data_field_state_t : uint8_t
{
    DFS_OK = 0x00,
    DFS_ALARM = 0x0F,
    DFS_ERROR = 0xFF,
};

/******************************************************************************************************************************
 *
 * CANFrame types
 *
 ******************************************************************************************************************************/
#define CAN_MAX_PAYLOAD 8

// base CAN frame format uses 11-bit IDs (uint16)
// extended CAN frame format uses 29-bit IDs (uint32)
typedef uint16_t can_id_t;

enum can_frame_error_codes_t : uint8_t
{
    CAN_FRAME_OK = 0x00,
    CAN_FRAME_IS_NULL = 0x01,
    CAN_FRAME_SIZE_ERROR = 0x02,
    CAN_FRAME_UNKNOWN_ERROR = 0xFF,
};

/******************************************************************************************************************************
 *
 * CANObject related data types
 *
 ******************************************************************************************************************************/
enum can_object_state_t : uint8_t
{
    COS_OK = 0x00,
    COS_DATA_FIELD_ERROR = 0x01,
    COS_LOCAL_DATA_BUFFER_SIZE_ERROR = 0x02,
    COS_UNKNOWN_ERROR = 0xFF,
};

/******************************************************************************************************************************
 *
 * CANManager related types
 *
 ******************************************************************************************************************************/

/******************************************************************************************************************************
 *
 * CANFunction related types
 *
 ******************************************************************************************************************************/
// CAN functon handler
class CANObject;
class CANFrame;
class CANFunctionBase;

// CAN Function result codes
enum CAN_function_result_t : uint8_t
{
    // nothing should be done, function will process this stuff next time
    CAN_RES_NONE = 0x00,

    // the next_ok_function will be called after handler
    CAN_RES_NEXT_OK = 0x01,

    // the next_err_function will be called after handler
    CAN_RES_NEXT_ERR = 0x02,

    // function processed all the things but won't to call next handler
    CAN_RES_FINAL = 0xFF,
};

using CAN_function_handler_t = CAN_function_result_t (*)(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);

// CAN Function IDs
enum CAN_function_id_t : uint8_t
{
    CAN_FUNC_NONE = 0x00,

    CAN_FUNC_SET_IN = 0x01,
    CAN_FUNC_SET_OUT_OK = 0x41,
    CAN_FUNC_SET_OUT_ERR = 0xC1,

    CAN_FUNC_REQUEST_IN = 0x11,
    CAN_FUNC_REQUEST_OUT_OK = 0x51,
    CAN_FUNC_REQUEST_OUT_ERR = 0xD1,

    CAN_FUNC_TIMER_NORMAL = 0x61,

    CAN_FUNC_TIMER_ATTENTION = 0x62,

    CAN_FUNC_TIMER_CRITICAL = 0x63,

    CAN_FUNC_SIMPLE_SENDER = 0xC0,

    CAN_FUNC_EVENT_ERROR = 0xE6,
};

// function type { responding, automatic, blended, indirect }
// Responding type - activated if incoming CAN frame received
// Automatic type - activated by CANObject whenever the update() method is called.
// Blended type - hybrid of responding and automatic; will be activated both by incoming CANFrame and by CANObject.update()
// Indirect type - this function will never activated by CANFrames or CANObject.update(). It will be called by other functions only.
enum CAN_function_type_t : uint8_t
{ // it is actually a bit mask
    CAN_FT_RESPONDING = 0b0001,
    CAN_FT_AUTOMATIC = 0b0010,
    CAN_FT_BLENDED = 0b0011,
    CAN_FT_INDIRECT = 0b0100,
};

// CAN Function state codes
enum CAN_function_state_t : uint8_t
{
    // function is OFF: it doesn't work, doesn't send any error replay
    // for example: timer is stopped; event is suppressed
    CAN_FS_STOPPED = 0x00,

    // function is working in normal mode
    CAN_FS_ACTIVE = 0x01,
};

// CAN Function error codes
enum CAN_function_error_t : uint8_t
{
    CAN_FUNC_ERROR_NO_EXTERNAL_HANDLER = 0x01,
    CAN_FUNC_ERROR_UNKNOWN_SETTER_ERROR = 0x02,
    CAN_FUNC_ERROR_READONLY_OBJECT = 0x03, // only objects with exactly one data field are writable
};

/******************************************************************************************************************************
 *
 * Pixel related types
 *
 ******************************************************************************************************************************/
enum pixel_error_codes_t : uint8_t
{
    PIX_ERR_NONE = 0x00,
    PIX_ERR_CAN_FRAME = 0x01,
    PIX_ERR_CAN_OBJECT = 0x02,
    PIX_ERR_DATA_FIELD = 0x03,
    PIX_ERR_FUNCTION = 0x04,
    PIX_ERR_CAN_MANAGER = 0x05,
};

#endif // CAN_COMMON_H