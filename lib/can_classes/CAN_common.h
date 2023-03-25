#ifndef CAN_COMMON_H
#define CAN_COMMON_H
// #pragma once

#include <stdint.h>

/******************************************************************************************************************************
 *
 * Common CAN related types
 *
 ******************************************************************************************************************************/
using get_ms_tick_function_t = uint32_t (*)();

/******************************************************************************************************************************
 *
 * Data types for DataField class
 *
 ******************************************************************************************************************************/
enum data_field_t : uint8_t
{
    DF_UNKNOWN = 0x00,
    DF_INT8 = 0x01,
    DF_UINT8 = 0x02,
    DF_INT16 = 0x03,
    DF_UINT16 = 0x04,
    DF_INT32 = 0x05,
    DF_UINT32 = 0x06,
    // DF_VARIOUS = 0xFF,
};

enum data_field_state_t : uint8_t
{
    DFS_OK = 0x00,
    DFS_ERROR = 0xFF,
};

/******************************************************************************************************************************
 *
 * CAN frame types
 *
 ******************************************************************************************************************************/
#define CAN_MAX_PAYLOAD 8

// base CAN frame format uses 11-bit IDs (uint16)
// extended CAN frame format uses 29-bit IDs (uint32)
typedef uint16_t can_id_t;

/******************************************************************************************************************************
 *
 * PixelCANFrame: Pixel specific CAN frame
 *
 ******************************************************************************************************************************/
enum func_type_t : uint8_t
{
    CAN_FT_NONE = 0x00,

    CAN_FT_SET_BOOL_IN = 0x01,
    CAN_FT_SET_BOOL_OUT_OK = 0x41,
    CAN_FT_SET_BOOL_OUT_ERR = 0xC1,

    CAN_FT_SET_VALUE_IN = 0x02,
    CAN_FT_SET_VALUE_OUT_OK = 0x42,
    CAN_FT_SET_VALUE_OUT_ERR = 0xC2,

    CAN_FT_REQUEST_IN = 0x11,
    CAN_FT_REQUEST_OUT_OK = 0x51,
    CAN_FT_REQUEST_OUT_ERR = 0xD1,

    CAN_FT_TIMER_NORMAL = 0x61,

    CAN_FT_TIMER_ATTENTION = 0x62,

    CAN_FT_TIMER_CRITICAL = 0x63,

    CAN_FT_EVENT_ERROR = 0xE6
};

/******************************************************************************************************************************
 *
 * Data types for CANObject class
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

#endif // CAN_COMMON_H