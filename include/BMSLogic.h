#pragma once

#include "BMS_low_level_abstraction.h"

extern UART_HandleTypeDef hBmsUart;

namespace BMSLogic
{
	//*********************************************************************
	// BMS firmware settings
	//*********************************************************************

	/// @brief BMS request period
	static constexpr uint32_t CFG_BMSRequestPeriod = 250;

    /// @brief BMS request timeout
    static constexpr uint32_t CFG_BMSRequestTimeout = 250;


	//*********************************************************************
	// BMS firmware constants
	//*********************************************************************

    /// @brief BMS get info request
    const uint8_t RequestData[6] = {0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00};


	//*********************************************************************
	// BMS firmware functions
	//*********************************************************************

	inline void Loop(uint32_t &current_time)
	{
		static uint32_t iter = 0;
        
		if(current_time - iter > CFG_BMSRequestPeriod)
		{
			iter = current_time;
            HAL_UART_Transmit(&hBmsUart, (uint8_t *)RequestData, sizeof(RequestData), CFG_BMSRequestTimeout);
		}
		
		current_time = HAL_GetTick();
	}

}