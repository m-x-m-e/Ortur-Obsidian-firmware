/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfig.h"

#if EITHER(USE_WIRED_EEPROM, FLASH_EEPROM_SPIFLASH)

#include "../shared/eeprom_api.h"

#define USE_WIRED_EEPROM 1
#ifndef E2END
#define E2END 0xFFF // EEPROM end address (4K)
#endif
#define EEPROM_ADDR (2*1024*1024-E2END) //定义模拟eeprom的起始地址

extern "C"{

extern void W25QXX_Init(void);
extern void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
extern void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);

extern void	system_suspend();
extern void	system_resume();

}

bool PersistentStore::access_start() {
//#ifdef USE_FREERTOS
//	system_suspend();
//#endif
  W25QXX_Init();
  return true;
}

bool PersistentStore::access_finish() {
//#ifdef USE_FREERTOS
//	system_resume();
//#endif
  return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
//  while (size--) {
//    uint8_t v = *value;
//
//    // Save to either external EEPROM, program flash or Backup SRAM
//    #if USE_WIRED_EEPROM
//      // EEPROM has only ~100,000 write cycles,
//      // so only write bytes that have changed!
//      uint8_t * const p = (uint8_t * const)pos;
//      if (v != eeprom_read_byte(p)) {
//        eeprom_write_byte(p, v);
//        if (eeprom_read_byte(p) != v) {
//          SERIAL_ECHO_MSG(STR_ERR_EEPROM_WRITE);
//          return true;
//        }
//      }
//    #else
//      *(__IO uint8_t *)(BKPSRAM_BASE + (uint8_t * const)pos) = v;
//    #endif
//
//    crc16(crc, &v, 1);
//    pos++;
//    value++;
//  };
//	return false;

	W25QXX_Write((uint8_t*)value,(uint32_t)(EEPROM_ADDR+pos),(uint16_t)size);
    crc16(crc, value, size);
    pos+=size;
    return true;
}

bool PersistentStore::read_data(int &pos, uint8_t* value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
//  do {
//
//    // Read from either external EEPROM, program flash or Backup SRAM
//
//    const uint8_t c = (0
//      #if USE_WIRED_EEPROM
//        //eeprom_read_byte((uint8_t*)pos)
//      #else
//        (*(__IO uint8_t *)(BKPSRAM_BASE + ((uint8_t*)pos)))
//      #endif
//    );
//    extern void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);
//    W25QXX_Read(&c,(uint8_t*)pos,1);
//    if (writing) *value = c;
//    crc16(crc, &c, 1);
//    pos++;
//    value++;
//  } while (--size);
//	return false;

    W25QXX_Read(value,(uint32_t)(EEPROM_ADDR+pos),(uint16_t)size);
    crc16(crc, value, size);
    pos+=size;
    return true;
}

size_t PersistentStore::capacity() {
  return (
    #if USE_WIRED_EEPROM
      E2END + 1
    #else
      2048 // 4kB
    #endif
  );
}

#endif // USE_WIRED_EEPROM || SRAM_EEPROM_EMULATION
#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC

