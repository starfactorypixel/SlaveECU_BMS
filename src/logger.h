#ifndef LOGGER_H
#define LOGGER_H
//#pragma once

//#include <iostream>
#include <string.h>
#include <stdio.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/*
#ifdef DEBUG
    #define LOGstring(fmt, ...) do { fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)
    #define LOGwoN(fmt, ...) LOGstring("[%s: %d] " fmt, __FILENAME__, __LINE__, ##__VA_ARGS__)
    #define LOG(fmt, ...) LOGwoN(fmt "\n", ##__VA_ARGS__)
#else
*/
    #define LOGstring(fmt, ...)
    #define LOGwoN(fmt, ...)
    #define LOG(fmt, ...)
//#endif


#endif // LOGGER_H