/*
    Pixel related CAN common functions and constants
*/
#pragma once

#include "stm32f1xx_hal.h"  // used only for HAL_GetTick() inside of CAN_register_update_timer()



/**************************************************************************************************************
 * CAN packet types
 **************************************************************************************************************/

// PIXEL_CAN_MSG_TYPE = PC_MT
#define PC_MT_GET_VALUE_REQUEST   0x11
#define PC_MT_GET_VALUE_RESP_OK   0x51
#define PC_MT_GET_VALUE_RESP_ERR  0xD1
/*
// TODO: will do it later
enum pix_can_get_value : uint8_t
{
    PC_MT_GET_VALUE_REQUEST = 0x11,
    PC_MT_GET_VALUE_RESP_OK = 0x51,
    PC_MT_GET_VALUE_RESP_ERR = 0xD1,
};
typedef enum pix_can_get_value pix_can_get_value_t;
*/



/**************************************************************************************************************
 * CAN packet structure
 **************************************************************************************************************/

typedef struct 
{
    uint16_t ID;                // ID пакета
    uint8_t type;               // Тип параметра
    // TODO: зачем нам массив на 8 байт, если первый байт всегда = type?
    uint8_t data[8];            // Данные параметра
    uint8_t length;             // Длина данных + 1(type)
    uint16_t period_ms;         // интервал отправки параметра в ms
    uint32_t current_timer;     // текущее значение таймера 
    // TODO: какие могут быть значения в state?
    uint8_t state;              // Состояние
} _params_v;



/**************************************************************************************************************
 * CAN registers common functions
 **************************************************************************************************************/

// Updates current_timer attribute of the register
void CAN_register_update_timer(_params_v* CAN_register){
    if(CAN_register == nullptr) return;

    // Don't update disabled timer
    // TODO: should rewrite hardcoded constant 0x01
    if (CAN_register->state != 0x01) return;

    CAN_register->current_timer = HAL_GetTick();
}


// Initialization of the CAN register
void CAN_register_init(_params_v* CAN_register, uint16_t reg_ID, uint8_t reg_type, uint16_t reg_period_ms){
    if(CAN_register == nullptr) return;

    CAN_register->ID = reg_ID;
    CAN_register->type = reg_type;
    CAN_register->period_ms = reg_period_ms;
    CAN_register_update_timer(CAN_register);
}


// Fills CAN register data with specified values
void CAN_register_fill_data(_params_v* CAN_register, uint8_t reg_data_length, uint8_t* reg_data){
    if (CAN_register == nullptr) return;

    if (reg_data_length > 7) return;

    // стоит очищать, чтобы мусор от старых значений не делал нам мозг, если где-то ошибёмся с размером данных
    memset( (CAN_register->data), 0, 8 );

    // TODO: потом надо структуру _params_v переделать так, чтобы CAN_register->data не был на 1 байт больше нужного
    // У нас тип хранится дважды: в type и в data[0]  
    CAN_register->data[0] = CAN_register->type;
    CAN_register->length = reg_data_length + 1; // 1 byte for data type field
    memcpy( &(CAN_register->data[1]), reg_data, reg_data_length );
}


// Fills CAN register with three uint16_t values
void CAN_register_fill_uint16x3(_params_v* CAN_register, uint16_t reg_val1, uint16_t reg_val2, uint16_t reg_val3){
    if (CAN_register == nullptr) return;

    uint8_t data[6];
    memcpy( &(data[0]), &reg_val1, sizeof(uint16_t) );
    memcpy( &(data[2]), &reg_val2, sizeof(uint16_t) );
    memcpy( &(data[4]), &reg_val3, sizeof(uint16_t) );
    CAN_register_fill_data(CAN_register, 3*sizeof(uint16_t), data );
}
