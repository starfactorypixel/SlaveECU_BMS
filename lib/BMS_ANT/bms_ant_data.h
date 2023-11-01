#pragma once

#include <stdint.h>

namespace BMSANTLib
{
	static constexpr uint8_t PacketSize = 140U;
	static constexpr uint8_t PacketHeader[] = {0xAA, 0x55, 0xAA, 0xFF};
	static constexpr uint8_t PacketRequest[] = {0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00};
	static constexpr uint8_t MaxCellsNumber = 32U;
	
	enum charge_mosfet_status_t : uint8_t
	{
		CHARGE_OFF = 0U,
		CHARGE_ON = 1U,
		CHARGE_CELL_VOLTAGE_HIGH = 2U,
		CHARGE_OVER_CURRENT = 3U,
		CHARGE_TOTAL_VOLTAGE_HIGH = 5U,
		CHARGE_TEMPERATURE_HIGH = 6U,
		CHARGE_MOS_TEMPERATURE_HIGH = 7U,
		CHARGE_CURRENT_ERROR = 8U,
		CHARGE_WIRE_DISCONNECTED_ERROR = 9U,
		CHARGE_BOARD_HIGH_TEMPERATURE = 10U,
		CHARGE_OPEN_FAILURE = 12U,
		CHARGE_CHARGE_MOS_ERROR = 13U,
		CHARGE_WAITING = 14U,
		CHARGE_MANUAL_OFF = 15U,
		CHARGE_LEVEL2_CELL_VOLT_HIGH = 16U,
		CHARGE_LOW_TEMPERATURE = 17U,
		CHARGE_CELL_VOLT_DIFF = 18U,
		CHARGE_CELL_NUMBER_ERROR = 22U
	};
	
	enum discharge_mosfet_status_t : uint8_t
	{
		DISCHARGE_OFF = 0U,
		DISCHARGE_ON = 1U,
		DISCHARGE_CELL_VOLTAGE_LOW = 2U,
		DISCHARGE_OVER_CURRENT = 3U,
		DISCHARGE_LEVEL2_OVER_CURRENT = 4U,
		DISCHARGE_TOTAL_VOLTAGE_LOW = 5U,
		DISCHARGE_TEMPERATURE_HIGH = 6U,
		DISCHARGE_MOS_TEMPERATURE_HIGH = 7U,
		DISCHARGE_CURRENT_ERROR = 8U,
		DISCHARGE_WIRE_DISCONNECTED_ERROR = 9U,
		DISCHARGE_BOARD_HIGH_TEMPERATURE = 10U,
		DISCHARGE_SHORT_CIRCUIT = 12U,
		DISCHARGE_MOS_ERROR = 13U,
		DISCHARGE_PRECHARGE_FAILURE = 14U,
		DISCHARGE_MANUAL_OFF = 15U,
		DISCHARGE_LEVEL_2_CELL_VOLT_LOW = 16U,
		DISCHARGE_LOW_TEMPERATURE = 17U,
		DISCHARGE_CELL_VOLT_DIFF = 18U,
		DISCHARGE_CELL_NUMBER_ERROR = 22U
	};
	
	enum balancer_status_t : uint8_t
	{
		BALANCER_OFF = 0U,
		BALANCER_CELL_VOLT_LIMIT_BALANCE = 1U,
		BALANCER_CELL_DIFF_BALANCE = 2U,
		BALANCER_BALANCE_TEMP_HIGH = 3U,
		BALANCER_AUTO_BALANCE = 4U,
		BALANCER_MOS_TEMP_HIGH = 10U
	};
	
	struct packet_raw_t
	{
		uint32_t header;				// Заголовок пакета.
		uint16_t total_voltage;			// Напряжение на всей АКБ, 0.1 V
		uint16_t cell_voltage_1;		// Напряжение на  1 ячейки, 0.001 V
		uint16_t cell_voltage_2;		// Напряжение на  2 ячейки, 0.001 V
		uint16_t cell_voltage_3;		// Напряжение на  3 ячейки, 0.001 V
		uint16_t cell_voltage_4;		// Напряжение на  4 ячейки, 0.001 V
		uint16_t cell_voltage_5;		// Напряжение на  5 ячейки, 0.001 V
		uint16_t cell_voltage_6;		// Напряжение на  6 ячейки, 0.001 V
		uint16_t cell_voltage_7;		// Напряжение на  7 ячейки, 0.001 V
		uint16_t cell_voltage_8;		// Напряжение на  8 ячейки, 0.001 V
		uint16_t cell_voltage_9;		// Напряжение на  9 ячейки, 0.001 V
		uint16_t cell_voltage_10;		// Напряжение на 10 ячейки, 0.001 V
		uint16_t cell_voltage_11;		// Напряжение на 11 ячейки, 0.001 V
		uint16_t cell_voltage_12;		// Напряжение на 12 ячейки, 0.001 V
		uint16_t cell_voltage_13;		// Напряжение на 13 ячейки, 0.001 V
		uint16_t cell_voltage_14;		// Напряжение на 14 ячейки, 0.001 V
		uint16_t cell_voltage_15;		// Напряжение на 15 ячейки, 0.001 V
		uint16_t cell_voltage_16;		// Напряжение на 16 ячейки, 0.001 V
		uint16_t cell_voltage_17;		// Напряжение на 17 ячейки, 0.001 V
		uint16_t cell_voltage_18;		// Напряжение на 18 ячейки, 0.001 V
		uint16_t cell_voltage_19;		// Напряжение на 19 ячейки, 0.001 V
		uint16_t cell_voltage_20;		// Напряжение на 20 ячейки, 0.001 V
		uint16_t cell_voltage_21;		// Напряжение на 21 ячейки, 0.001 V
		uint16_t cell_voltage_22;		// Напряжение на 22 ячейки, 0.001 V
		uint16_t cell_voltage_23;		// Напряжение на 23 ячейки, 0.001 V
		uint16_t cell_voltage_24;		// Напряжение на 24 ячейки, 0.001 V
		uint16_t cell_voltage_25;		// Напряжение на 25 ячейки, 0.001 V
		uint16_t cell_voltage_26;		// Напряжение на 26 ячейки, 0.001 V
		uint16_t cell_voltage_27;		// Напряжение на 27 ячейки, 0.001 V
		uint16_t cell_voltage_28;		// Напряжение на 28 ячейки, 0.001 V
		uint16_t cell_voltage_29;		// Напряжение на 29 ячейки, 0.001 V
		uint16_t cell_voltage_30;		// Напряжение на 30 ячейки, 0.001 V
		uint16_t cell_voltage_31;		// Напряжение на 31 ячейки, 0.001 V
		uint16_t cell_voltage_32;		// Напряжение на 32 ячейки, 0.001 V
		int32_t  total_current;			// Ток разряда иди заряда, 0.1 A
		uint8_t  capacity_percent;		// Оставшаяся ёмкость АКБ, 1.0 %
		uint32_t capacity_phy;			// Фактическая ёмкость АКБ, 0.000001 Ah
		uint32_t capacity_rem;			// Оставшаяся ёмкость АКБ, 0.000001 Ah
		uint32_t capacity_cycle;		// , 0.001 Ah
		uint32_t uptime;				// Кол-во времени с питанием, 1.0 s
		int16_t  temp_mosfet;			// Температура датчика mosfet, 1.0 C
		int16_t  temp_balans;			// Температура датчика balance, 1.0 C
		int16_t  temp_1;				// Температура датчика 1, 1.0 C
		int16_t  temp_2;				// Температура датчика 2, 1.0 C
		int16_t  temp_3;				// Температура датчика 3, 1.0 C
		int16_t  temp_4;				// Температура датчика 4, 1.0 C
		uint8_t  status_charge_fet;		// Флаг состояние ключа зарядки, charge_mosfet_status_t
		uint8_t  status_dcharge_fet;	// Флаг состояние ключа разрядки, discharge_mosfet_status_t
		uint8_t  status_balancer;		// Флаг состояние балансировки, balancer_status_t
		uint16_t tire_length;			// WAT?
		uint16_t pulses_num;			// WAT?
		uint8_t  relay;					// WAT?
		uint32_t total_power;			// Текущая мощность, 1.0 W
		uint8_t  cell_vmax_num;			// Номер батарее с максимальный напряжением.
		uint16_t cell_vmax_volt;		// Напряжение батареи с максимальным напряжением, 0.001 V
		uint8_t  cell_vmin_num;			// Номер батарее с минимальным напряжением.
		uint16_t cell_vmin_volt;		// Напряжение батареи с минимальным напряжением, 0.001 V
		uint16_t cell_vmid_volt;		// Среднее напряжение батареи, 0.001 V
		uint8_t  cell_count_num;		// Кол-во ячеек в АКБ.
		uint16_t dcharge_fet_lost;		// Падение на транзисторе нагрузки, В.
		uint16_t dcharge_fet_v;			// Напряжение на затворе транзистора нагрузки, В.
		uint16_t charge_fet_v;			// Напряжение на затворе транзистора зарядки, В.
		uint16_t wat_130131;			// WAT?
		uint32_t balance_bits;			// Флаги сбалансированных ячеек.
		uint16_t logs;					// WAT?
		uint16_t crc;					// Контрольная сумма.
	};
	
}
