#ifndef LOGGER_H
#define LOGGER_H
//#pragma once

#include <stdio.h>
#include <string.h>


#ifdef DEBUG

    #ifdef DEBUG_X64
        #define LOGstring(fmt, ...) do { fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)
    #else
        #define LOGstring(fmt, ...) do { extern UART_HandleTypeDef huart1; char str_buff[255]; sprintf(str_buff, fmt, ##__VA_ARGS__); HAL_UART_Transmit(&huart1, (uint8_t *)str_buff, strlen(str_buff), 100); } while (0)
    #endif

    #ifdef DEBUG_DETAILED
        #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
        #define LOGwoN(fmt, ...) LOGstring("[%s: %d] " fmt, __FILENAME__, __LINE__, ##__VA_ARGS__)
    #else
        #define LOGwoN(fmt, ...) LOGstring(fmt, ##__VA_ARGS__)
    #endif

    #define LOG(fmt, ...) LOGwoN(fmt "\n", ##__VA_ARGS__)
#else
    #define LOGstring(fmt, ...)
    #define LOGwoN(fmt, ...)
    #define LOG(fmt, ...)
#endif


#endif // LOGGER_H