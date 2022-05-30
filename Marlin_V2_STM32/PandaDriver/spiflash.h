#include <stdint.h>

#ifndef E2END
#define E2END 0xFFF // EEPROM end address (4K)
#endif
#define EEPROM_ADDR (2*1024*1024-E2END) //定义模拟eeprom的起始地址

#ifdef __cplusplus
extern "C"
{
#endif

void W25QXX_Init(void);
uint16_t  W25QXX_ReadID(void);  	    //读取FLASH ID
uint8_t	 W25QXX_ReadSR(void);        //读取状态寄存器
void W25QXX_Write_SR(uint8_t sr);  	//写状态寄存器
void W25QXX_Write_Enable(void);  //写使能
void W25QXX_Write_Disable(void);	//写保护
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   //读取flash
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);//写入flash
void W25QXX_Erase_Chip(void);    	  //整片擦除
void W25QXX_Erase_Sector(uint32_t Dst_Addr);//扇区擦除
void W25QXX_Wait_Busy(void);           //等待空闲
void W25QXX_PowerDown(void);           //进入掉电模式
void W25QXX_WAKEUP(void);			  //唤醒

void calcu_crc16(uint16_t *crc, const void * const data, uint16_t cnt) ; //计算CRC

#ifdef __cplusplus
}
#endif
