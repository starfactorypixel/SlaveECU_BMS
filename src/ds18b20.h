#ifndef DS18B20_H_
#define DS18B20_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------
#define SKIP_ROM 0
#define NO_SKIP_ROM 1
//--------------------------------------------------
#define RESOLUTION_9BIT 0x1F
#define RESOLUTION_10BIT 0x3F
#define RESOLUTION_11BIT 0x5F
#define RESOLUTION_12BIT 0x7F
//--------------------------------------------------
#define DS_PORT  GPIOB   //указать порт, к которому подключены датчики
#define DS_PIN 15

#define GPIO_DS_PIN GPIO_PIN_15
#define GPIO_DS_PIN_OUT GPIO_ODR_ODR15
#define GPIO_DS_PIN_READ GPIO_IDR_IDR15
//--------------------------------------------------
void DelayMicr(uint32_t cn);
void port_init(void);
uint8_t ds18b20_init(uint8_t mode);
void ds18b20_MeasureTemperCmd(uint8_t mode, uint8_t DevNum);
void ds18b20_ReadStratcpad(uint8_t mode, uint8_t *Data, uint8_t DevNum);
bool ds18b20_GetSign(uint16_t dt);
float ds18b20_Convert(uint16_t dt);
//--------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif /* DS18B20_H_ */
