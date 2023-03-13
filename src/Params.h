#ifndef PARAMS_H_
#define PARAMS_H_

#include "stm32f1xx_hal.h"

#include "pixel_CAN.h"
#include "pixel_CAN_BMS.h"

// Макрос для сообщений об ошибке на этапе компиляции
// удобно проверять соответствие фактических размеров структур и выделенных под них буфферов 
#define CASSERT(predicate, file) _impl_CASSERT_LINE(predicate,__LINE__,file)
#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
    typedef char _impl_PASTE(assertion_failed_##file##_,line)[2*!!(predicate)-1];

// max of two values
#define max(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

// min of two values
#define min(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; })


#ifdef __cplusplus
extern "C" {
#endif

#define ThresholdTemperature1   60      // пороговое значение предельной температуры

#define BMS_PACKET_SIZE   140     // 140 байт в пакете данных, получаемых от BMS
#define BMS_PACKET_HEADER 0xAA55AAFF    // from SlaveECU github

#pragma pack(push, 1)   // pack this struct with byte alignment
struct params_t
        {
			uint32_t header;            // Заголовок пакета.
			uint16_t voltage;			// Напряжение на всей АКБ, x10 мВ.
            // TODO: by wosk - 32x uint16_t voltage_x should be changed to uint16_t voltage_cell[32]
			uint16_t voltage_1;         // Напряжение на 1 ячейке, мВ.
			uint16_t voltage_2;         // Напряжение на 2 ячейке, мВ.
			uint16_t voltage_3;         // Напряжение на 3 ячейке, мВ.
			uint16_t voltage_4;         // Напряжение на 4 ячейке, мВ.
			uint16_t voltage_5;         // Напряжение на 5 ячейке, мВ.
			uint16_t voltage_6;         // Напряжение на 6 ячейке, мВ.
			uint16_t voltage_7;         // Напряжение на 7 ячейке, мВ.
			uint16_t voltage_8;         // Напряжение на 8 ячейке, мВ.
			uint16_t voltage_9;         // Напряжение на 9 ячейке, мВ.
			uint16_t voltage_10;        // Напряжение на 10 ячейке, мВ.
			uint16_t voltage_11;        // Напряжение на 11 ячейке, мВ.
			uint16_t voltage_12;        // Напряжение на 12 ячейке, мВ.
			uint16_t voltage_13;        // Напряжение на 13 ячейке, мВ.
			uint16_t voltage_14;        // Напряжение на 14 ячейке, мВ.
			uint16_t voltage_15;        // Напряжение на 15 ячейке, мВ.
			uint16_t voltage_16;        // Напряжение на 16 ячейке, мВ.
			uint16_t voltage_17;        // Напряжение на 17 ячейке, мВ.
			uint16_t voltage_18;        // Напряжение на 18 ячейке, мВ.
			uint16_t voltage_19;        // Напряжение на 19 ячейке, мВ.
			uint16_t voltage_20;        // Напряжение на 20 ячейке, мВ.
			uint16_t voltage_21;        // Напряжение на 21 ячейке, мВ.
			uint16_t voltage_22;        // Напряжение на 22 ячейке, мВ.
			uint16_t voltage_23;        // Напряжение на 23 ячейке, мВ.
			uint16_t voltage_24;        // Напряжение на 24 ячейке, мВ.
			uint16_t voltage_25;        // Напряжение на 25 ячейке, мВ.
			uint16_t voltage_26;        // Напряжение на 26 ячейке, мВ.
			uint16_t voltage_27;        // Напряжение на 27 ячейке, мВ.
			uint16_t voltage_28;        // Напряжение на 28 ячейке, мВ.
			uint16_t voltage_29;        // Напряжение на 29 ячейке, мВ.
			uint16_t voltage_30;        // Напряжение на 30 ячейке, мВ.
			uint16_t voltage_31;        // Напряжение на 31 ячейке, мВ.
			uint16_t voltage_32;        // Напряжение на 32 ячейке, мВ.
			uint16_t reserved_70;		// 
			int16_t current;            // Ток разряда или заряда, мА.
			uint8_t percent;            // Оставшаяся ёмкость АКБ, проценты.
			uint32_t phy_capacity;      // Фактическая ёмкость АКБ, мА/ч.
			uint32_t rem_capacity;      // Оставшаяся ёмкость АКБ, мА/ч.
			uint32_t cycle_capacity;    // WAT?
			uint32_t uptime;            // Кол-во времени с питанием, секунд.
			int16_t temp_mos;           // Температура датчика MOSFET, Градусы.
			int16_t temp_bal;           // Температура датчика Balance, Градусы.
			int16_t temp_1;             // Температура датчика 1, Градусы.
			int16_t temp_2;             // Температура датчика 2, Градусы.
			int16_t temp_3;             // Температура датчика 3, Градусы.
			int16_t temp_4;             // Температура датчика 4, Градусы.
			uint8_t charge_fet;         // Флаг состояние ключа зарядки.
			uint8_t dcharge_fet;        // Флаг состояние ключа разрядки.
			uint8_t balanced;           // Флаг состояние балансировки.
			uint16_t tire_length;       // WAT?
			uint16_t pulses_num;        // WAT?
			uint8_t relay;              // WAT?
			uint32_t power;             // Текущая мощность, Вт.
			uint8_t vmax_bat;           // Номер батарее с максимальный напряжением.
			uint16_t vmax;              // Напряжение батареи с максимальным напряжением, мВ.
			uint8_t vmin_bat;           // Номер батарее с минимальным напряжением.
			uint16_t vin;               // Напряжение батареи с минимальным напряжением, мВ.
			uint16_t vmid;              // Среднее напряжение батареи, мВ.
			uint8_t wat_123;            // WAT?
			uint16_t dcharge_fet_lost;  // Падение на транзисторе нагрузки, В.
			uint16_t dcharge_fet_v;     // Напряжение на затворе транзистора нагрузки, В.
			uint16_t charge_fet_v;      // Напряжение на затворе транзистора зарядки, В.
			uint16_t wat_130131;        // WAT?
			uint32_t balance_bits;      // Флаги сбалансированных ячеек.
			uint16_t logs;              // WAT?
			uint16_t crc;               // Контрольная сумма.
        } params;		// 	140 bytes = BMS_PACKET_SIZE
#pragma pack(pop)

// compile time check of the params_t structure size
// it will raise a compilation error if the BMS_PACKET_SIZE will not equal to the params_t size
// выдаст ошибку на этапе компиляции, если размер структуры не будет соответствовать BMS_PACKET_SIZE
CASSERT( sizeof(params) == BMS_PACKET_SIZE, Params_h );

_params_v BlockInfo =       {BlockInfo_ID};
_params_v BlockHealth =     {BlockHealth_ID};
_params_v BlockCfg =        {BlockCfg_ID};
_params_v BlockError =      {BlockError_ID};
_params_v HighVoltage =     {HighVoltage_ID};
_params_v HighCurrent =     {HighCurrent_ID};
_params_v MaxTemperature =  {MaxTemperature_ID};
_params_v Temperature1 =    {Temperature1_ID};
_params_v Temperature2 =    {Temperature2_ID};
_params_v Temperature3 =    {Temperature3_ID};
_params_v LowVoltage1_3 =   {LowVoltage1_3_ID};
_params_v LowVoltage4_6 =   {LowVoltage4_6_ID};
_params_v LowVoltage7_9 =   {LowVoltage7_9_ID};
_params_v LowVoltage10_12 = {LowVoltage10_12_ID};
_params_v LowVoltage13_15 = {LowVoltage13_15_ID};
_params_v LowVoltage16_18 = {LowVoltage16_18_ID};
_params_v LowVoltage19_21 = {LowVoltage19_21_ID};
_params_v LowVoltage22_24 = {LowVoltage22_24_ID};
_params_v LowVoltageMinMaxDelta ={LowVoltageMinMaxDelta_ID};

#ifdef __cplusplus
}
#endif

#endif /* PARAMS_H_ */
