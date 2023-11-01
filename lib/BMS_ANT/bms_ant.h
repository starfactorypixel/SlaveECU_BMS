/*
	
*/

#pragma once

#include <stdint.h>
#include <cstring>
#include <bms_ant_data.h>

class bms_ant
{
	using callback_tx_t = void (*)(uint8_t *data, uint8_t length);
	
	public:
		bms_ant(callback_tx_t tx_func) : _callback_tx(tx_func)
		{

		}
		
		void Init()
		{
//            memset(&params, 0x00, sizeof(params));
            _params_idx = 0;
            _ready = false;
            
            return;
        }
        
        /*
            Пакет получен, распарсен, проверен и готов к выдаче данных.
        */
        bool IsReady()
        {
            return _ready;
        }


		void InputData(uint8_t *data, uint8_t length, uint32_t time)
		{
			if(length != BMSANTLib::PacketSize) return;
			
			if(time - _last_inputdata_time > 250)
			{
				//reset
			}
			_last_inputdata_time = time;
			
			memcpy(_buffer, data, length);
			for(uint8_t li = 0, ri = 0; li < BMSANTLib::PacketSize / 2; li++)
			{
				ri = BMSANTLib::PacketSize - li - 1;
				uint8_t tmp = _buffer[li];
				_buffer[li] = _buffer[ri];
				_buffer[ri] = tmp;
			}
			
			_CheckBuffer();


			



			
		}


        
	private:
		
		void _CheckBuffer()
		{
			BMSANTLib::packet_raw_t *packet = (BMSANTLib::packet_raw_t *) _buffer;
			
			if(packet->header == *((uint32_t *) BMSANTLib::PacketHeader))
			{
				if(packet->crc == _CRCBuffer())
				{
					_state = STATE_RECEIVED;
				}
				else
				{
					_error = ERROR_CRC;
				}
			}
			else
			{
				_error = ERROR_FORMAT;
			}
			
			return;
		}
		
		uint16_t _CRCBuffer()
		{
			uint16_t result = 0x0000;
			
			for(uint8_t i = 2; i < BMSANTLib::PacketSize - 4; ++i)
			{
				result += _buffer[i];
			}
			
			return result;
		}
        
        uint8_t _params_idx;
        bool _ready;


		callback_tx_t _callback_tx;

		uint8_t _buffer[BMSANTLib::PacketSize];
		
		uint32_t _last_inputdata_time;

		enum { STATE_IDLE, STATE_RECEIVED, STATE_PARSED } _state;
		enum { ERROR_NONE, ERROR_CRC, ERROR_FORMAT } _error;


};
