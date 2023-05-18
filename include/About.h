#pragma once

namespace About
{
	static constexpr char name[] = "BMS_ECU";
	static constexpr char desc[] = "BMS interface board for Pixel project";
	static constexpr char hw_ver[] = "1.0.0";
	static constexpr char sw_ver[] = "0.1.1";
	static constexpr char git[] = "https://github.com/starfactorypixel/SlaveECU_BMS";
	
	inline void Setup()
	{
		Serial::Print("\r\n\r\n");
		Serial::Printf<128>("+INFO=%s, hw:%s, sw:%s\r\n", name, hw_ver, sw_ver);
		Serial::Printf<128>("+INFO=Desc: %s\r\n", desc);
		Serial::Printf<128>("+INFO=Build: %s %s\r\n", __DATE__, __TIME__);
		Serial::Printf<128>("+INFO=GitHub: %s\r\n", git);
		Serial::Print("+READY\r\n\r\n");
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		return;
	}
}
