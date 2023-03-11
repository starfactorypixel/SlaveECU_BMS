#include "stm32f1xx_hal.h"
#include "flash.h"
#include <string.h>

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////// РАБОТА с ФЛЕШЕМ //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  #define ADDR_Flash ADDR_FLASH_PAGE_64
	uint32_t address = ADDR_Flash; // запись в начало страницы

	
extern 	uint16_t write_data16[];
extern	uint16_t read_data16[];	
	
	uint8_t Erase_flash(uint32_t Erase_address){
  static FLASH_EraseInitTypeDef EraseInitStruct; // структура для очистки флеша

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // постраничная очистка, FLASH_TYPEERASE_MASSERASE - очистка всего флеша
  EraseInitStruct.PageAddress = ADDR_Flash; // адрес  страницы
  EraseInitStruct.NbPages = 1;                       // кол-во страниц для стирания
  //EraseInitStruct.Banks = FLASH_BANK_1; // FLASH_BANK_2 - банк №2, FLASH_BANK_BOTH - оба банка

  uint32_t page_error = 0; // переменная, в которую запишется адрес страницы при неудачном стирании

  ////////////////////////////// ОЧИСТКА ///////////////////////////////////

  HAL_FLASH_Unlock(); // разблокировать флеш

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK)
  {
		HAL_FLASH_Lock(); // заблокировать флеш
		return 0;
  }
  HAL_FLASH_Lock(); // заблокировать флеш
	return 1;
}
  ///////////////////////// КОНЕЦ ОЧИСТКИ //////////////////////////////

  // 0x00U - No error
  // 0x01U - Programming error
  // 0x02U - Write protection error
  // 0x04U - Option validity error
// Если записывать 32-х (FLASH_TYPEPROGRAM_WORD) битное число, то увеличим адрес на четыре, а если 64-х (FLASH_TYPEPROGRAM_DOUBLEWORD) битное, то на 8. 
// Если в дальнейшем захотите добавить в эту страницу ещё что-то, то нужно запомнить адрес.
  ////////////////////////////// ЗАПИСЬ числа 16 bits ///////////////////////////////////
	uint8_t Write_flash_16b(uint8_t len){
  HAL_FLASH_Unlock(); // разблокировать флеш

  uint32_t address = ADDR_Flash; // запись в начало страницы 

  for(uint8_t i = 0; i < len; i++)
  {
	  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, write_data16[i]) != HAL_OK)
	  {
			HAL_FLASH_Lock(); // заблокировать флеш
			return 0;
	  }

	  address = address + 2;
  }
  HAL_FLASH_Lock(); // заблокировать флеш
	return 1;
}
  /////////////////////// КОНЕЦ ЗАПИСИ числа 16 bits ///////////////////////////


  ////////////////////////////// ЧТЕНИЕ числа 16 bits ///////////////////////////////////
void Read_flash_16b(uint8_t len){
  address = ADDR_Flash;

  for(uint16_t i = 0; i < len; i++)
  {
	  read_data16[i] = *(uint16_t*)address; // читаем число по адресу
	  address = address + 2;
  }
}
  //////////////////////// КОНЕЦ ЧТЕНИЯ числа 16 bits ///////////////////////////////////


  ////////////////////////////// ЗАПИСЬ строки 8 bits 	должны быть четными
	uint32_t Write_flash_8b_str(uint8_t* data, uint8_t len){
  HAL_FLASH_Unlock(); // разблокировать флеш

  address = ADDR_Flash + 4; // смещаем адрес 

  uint16_t data16 = 0, index0 = 0, index1 = 1;

  if(len % 2 != 0) // проверка кол-ва символов в массиве на чётность
  {
		return 0;
  }

  for(uint16_t i = 0; i < len / 2; i++)
  {
	  data16 = ((uint16_t)data[index1] << 8) | data[index0];
	  index0 = index0 + 2;
	  index1 = index1 + 2;

	  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data16) != HAL_OK)
	  {
		  uint32_t er = HAL_FLASH_GetError();
			return er;
	  }

	  address = address + 2;
  }

  HAL_FLASH_Lock(); // заблокировать флеш

return 1;
}
  /////////////////////// КОНЕЦ ЗАПИСИ строки 8 bits //////////////////////////


  ////////////////////////////// ЧТЕНИЕ строки 8 bits ///////////////////////////////////
void Read_flash_8b_str(void){
  address = ADDR_Flash + 4;
  char buf[64] = {0,};
	
  for(uint16_t i = 0; i < 8; i++)
  {
	  buf[i] = *(uint32_t*)address++;
  }
}
  //////////////////////// КОНЕЦ ЧТЕНИЯ строки 8 bits ///////////////////////////////////


  //////////////////////// ПРОЧИТАТЬ ПОЛЬЗОВАТЕЛЬСКИЕ БИТЫ - User Data  /////////////////////////////

//  uint32_t user_bit = HAL_FLASHEx_OBGetUserData(OB_DATA_ADDRESS_DATA0);
//  snprintf(str, 64, "User_bit_0: %lu\n", user_bit);
//  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);

//  user_bit = HAL_FLASHEx_OBGetUserData(OB_DATA_ADDRESS_DATA1);
//  snprintf(str, 64, "User_bit_1: %lu\n", user_bit);
//  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);

  //////////////////////// КОНЕЦ ЧТЕНИЯ /////////////////////////////
