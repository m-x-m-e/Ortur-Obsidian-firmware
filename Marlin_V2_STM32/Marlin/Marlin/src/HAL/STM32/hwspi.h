/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
#pragma once

#include "../shared/Marduino.h"

#ifndef FORCE_INLINE
  #define FORCE_INLINE inline __attribute__((always_inline))
#endif

//#define USE_SPI_DMA
#define DMA_BUF_SIZE 1024

/**
 * @class HWSPI
 * @brief Fast HardWare SPI.
 */
template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin, uint8_t Mode = 0>
class HWSPI {
private:
	bool inited = false;
 public:

#ifdef USE_SPI_DMA
	static uint8_t* DmaRecBuf()
	{
		__attribute__((aligned(4))) static uint8_t Spi_DmaRecBuf[DMA_BUF_SIZE]={0};
		return Spi_DmaRecBuf;
	}

	static uint8_t* DmaSendBuf()
	{
		__attribute__((aligned(4))) static uint8_t Spi_DmaSendBuf[DMA_BUF_SIZE]={0};
		return Spi_DmaSendBuf;
	}
#endif

	void begin(uint8_t rate) {
		int clock=4000000;
		unsigned char SpeedSet=1;
		begin();
		switch (rate)
		{
		case SPI_FULL_SPEED:    clock = 20000000;SpeedSet=1; break; // 13.9mhz=20000000  6.75mhz=10000000  3.38mhz=5000000  .833mhz=1000000
		case SPI_HALF_SPEED:    clock =  5000000;SpeedSet=2; break;
		case SPI_QUARTER_SPEED: clock =  2500000;SpeedSet=3; break;
		case SPI_EIGHTH_SPEED:  clock =  1250000;SpeedSet=4; break;
		case SPI_SPEED_5:       clock =   625000;SpeedSet=7; break;
		case SPI_SPEED_6:       clock =   300000;SpeedSet=7; break;
		default:
		  clock = 4000000;SpeedSet=3; // Default from the SPI library
		}

		SpeedSet&=0X07;			//限制范围
		SPI1->CR1&=0XFFC7;
		SPI1->CR1|=SpeedSet<<3;	//设置SPI1速度
		SPI1->CR1|=1<<6; 		//SPI设备使能
	}

  /** Initialize SoftSPI pins. */
  void begin() {

	if(inited)
		return;

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	LL_GPIO_AF_EnableRemap_SPI1();
	LL_GPIO_AF_Remap_SWJ_NOJTAG();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_3 ;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;  //复用推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin =  GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;  //复用推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_4 ;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,GPIO_PIN_SET);

	SPI1->CR1=0;
	SPI1->CR1|=0<<10;//全双工模式
	SPI1->CR1|=1<<9; //软件nss管理
	SPI1->CR1|=1<<8;

	SPI1->CR1|=1<<2; //SPI主机
	SPI1->CR1|=0<<11;//8bit数据格式
	SPI1->CR1|=1<<1; //空闲模式下SCK为1 CPOL=1
	SPI1->CR1|=1<<0; //数据采样从第二个时间边沿开始,CPHA=1
	SPI1->CR1|=1<<3; //Fsck=Fcpu/2^1
	SPI1->CR1|=0<<7; //MSBfirst
	SPI1->CR1|=1<<6; //SPI设备使能

#ifdef USE_SPI_DMA
		dmaInit();
#endif

	transfer(0xff);

	inited = true;
  }

#ifdef USE_SPI_DMA

  void dmaInit(void)
	{
		//RCC->AHBENR|=1<<0;			//开启DMA1时钟
		__HAL_RCC_DMA1_CLK_ENABLE();

		DMA1_Channel3->CPAR=(uint32_t)&SPI1->DR; 	 	//DMA1 外设地址
		//DMA_CHx->CMAR=(u32)cmar; 	//DMA1,存储器地址
		//DMA1_MEM_LEN=cndtr;      	//保存DMA传输数据量
		//DMA1_Channe3->CNDTR=cndtr;    	//DMA1,传输数据量
		DMA1_Channel3->CCR=0X00000000;	//复位
		DMA1_Channel3->CCR|=1<<4;  		//从存储器读
		DMA1_Channel3->CCR|=0<<5;  		//普通模式
		DMA1_Channel3->CCR|=0<<6; 		//外设地址非增量模式
		DMA1_Channel3->CCR|=1<<7; 	 	//存储器增量模式
		DMA1_Channel3->CCR|=0<<8; 	 	//外设数据宽度为8位
		DMA1_Channel3->CCR|=0<<10; 		//存储器数据宽度8位
		DMA1_Channel3->CCR|=1<<12; 		//中等优先级
		DMA1_Channel3->CCR|=0<<14; 		//非存储器到存储器模式

		DMA1_Channel2->CPAR=(uint32_t)&SPI1->DR; 	//DMA1 外设地址
		DMA1_Channel2->CCR=0X00000000;	//复位
		DMA1_Channel2->CCR|=0<<4;  		//从外设读
		DMA1_Channel2->CCR|=0<<5;  		//普通模式
		DMA1_Channel2->CCR|=0<<6; 		//外设地址非增量模式
		DMA1_Channel2->CCR|=1<<7; 	 	//存储器增量模式
		DMA1_Channel2->CCR|=0<<8; 	 	//外设数据宽度为8位
		DMA1_Channel2->CCR|=0<<10; 		//存储器数据宽度8位
		DMA1_Channel2->CCR|=1<<12; 		//中等优先级
		DMA1_Channel2->CCR|=0<<14; 		//非存储器到存储器模式
	}
  /*确保recbufptr是4字节对齐地址*/
   uint8_t* dmaTransfer(uint8_t* recbufptr,int len)
   {

    SPI1->CR2 |= 1<<0;
    SPI1->CR2 |= 1<<1;

    DMA1_Channel3->CCR&=~(1<<7);       //存储器非增量模式

    /*开启spi接收dma*/
    DMA1_Channel2->CMAR=(uint32_t)recbufptr;   //DMA1,存储器地址
    DMA1_Channel2->CNDTR=len;       //DMA1,传输数据量

    /*开启spi发送dma*/
    DMA1_Channel3->CMAR=(uint32_t)DmaSendBuf();  //DMA1,存储器地址
    DMA1_Channel3->CNDTR=len;       //DMA1,传输数据量

    DMA1->IFCR|=(0X0F<<4);
    DMA1->IFCR|=(0X0F<<8);

    DMA1_Channel2->CCR|=1<<0;              //开启DMA传输
    while((SPI1->SR&0X02)==0);
    DMA1_Channel3->CCR|=1<<0;              //开启DMA传输

    while((DMA1->ISR&(1<<9))==0);
    DMA1_Channel3->CCR&=~(1<<0);           //关闭DMA传输

    while((DMA1->ISR&(1<<5))==0);
    DMA1_Channel2->CCR&=~(1<<0);           //关闭DMA传输

    return recbufptr;
   }
  	/*发送buf和接收buf大小需保持一致*/
  	uint8_t dmaTransfer(uint8_t* sendbufptr,uint8_t* recbufptr,int len)
  	{
  		uint32_t i=0;
  		uint32_t rest=0;
  		uint32_t send=0;
  		uint32_t sent=0;
  		rest=len;
  		SPI1->CR2 |= 1<<0;
  		SPI1->CR2 |= 1<<1;
  		while(rest)
  		{
  			if(rest>DMA_BUF_SIZE)
  			{
  				memcpy(DmaSendBuf(),&sendbufptr[sent],DMA_BUF_SIZE);
  				rest-=DMA_BUF_SIZE;
  				send=DMA_BUF_SIZE;
  				//sent+=DMA_BUF_SIZE;
  			}
  			else
  			{
  				memcpy(DmaSendBuf(),&sendbufptr[sent],rest);
  				send=rest;
  				rest=0;
  				//sent+=rest;
  			}

  			/*开启spi接收dma*/
  			DMA1_Channel2->CMAR=(uint32_t)&DmaRecBuf()[sent]; 	//DMA1,存储器地址
  			DMA1_Channel2->CNDTR=send; 								//DMA1,传输数据量

  			/*开启spi发送dma*/
  			DMA1_Channel3->CMAR=(uint32_t)DmaSendBuf(); 	//DMA1,存储器地址
  			DMA1_Channel3->CNDTR=send; 						//DMA1,传输数据量

  			DMA1->IFCR|=(0X0F<<4);
  			DMA1->IFCR|=(0X0F<<8);
  			DMA1_Channel2->CCR|=1<<0;          			//开启DMA传输
  			while((SPI1->SR&0X02)==0);
  			DMA1_Channel3->CCR|=1<<0;          			//开启DMA传输

  			while((DMA1->ISR&(1<<9))==0);
  			DMA1_Channel3->CCR&=~(1<<0);       			//关闭DMA传输

  			while((DMA1->ISR&(1<<5))==0);
  			DMA1_Channel2->CCR&=~(1<<0);       			//关闭DMA传输

  			memcpy(recbufptr,DmaRecBuf(),send);
  			sent+=send;
  		}
  		return DmaRecBuf()[0];
  	}


//  	/*发送buf和接收buf大小需保持一致*/
//  	uint8_t dmaTransfer(uint8_t* sendbufptr,uint8_t* recbufptr,int len)
//  	{
//  		SPI1->CR2 |= 1<<0;
//  		SPI1->CR2 |= 1<<1;
//
//  		DMA1_Channel3->CCR|=(1<<7);       //存储器增量模式
//		/*开启spi接收dma*/
//		DMA1_Channel2->CMAR=(uint32_t)recbufptr; 	//DMA1,存储器地址
//		DMA1_Channel2->CNDTR=len; 					//DMA1,传输数据量
//
//		/*开启spi发送dma*/
//		DMA1_Channel3->CMAR=(uint32_t)sendbufptr; 	//DMA1,存储器地址
//		DMA1_Channel3->CNDTR=len; 					//DMA1,传输数据量
//
//		DMA1->IFCR|=(0X0F<<4);
//		DMA1->IFCR|=(0X0F<<8);
//		DMA1_Channel2->CCR|=1<<0;          			//开启DMA传输
//		while((SPI1->SR&0X02)==0);
//		DMA1_Channel3->CCR|=1<<0;          			//开启DMA传输
//
//		while((DMA1->ISR&(1<<9))==0);
//		DMA1_Channel3->CCR&=~(1<<0);       			//关闭DMA传输
//
//		while((DMA1->ISR&(1<<5))==0);
//		DMA1_Channel2->CCR&=~(1<<0);       			//关闭DMA传输
//
//  		return recbufptr[0];
//  	}

#endif

  /**
   * Soft SPI receive byte.
   * @return Data byte received.
   */
  FORCE_INLINE uint8_t receive() {
	  uint8_t data = 0xff;
	  return transfer(data);
  }

  /**
   * Soft SPI send byte.
   * @param[in] data Data byte to send.
   */
  FORCE_INLINE void send(uint8_t data) {
	   transfer(data);
  }

  /**
   * Soft SPI transfer byte.
   * @param[in] txData Data byte to send.
   * @return Data byte received.
   */
  FORCE_INLINE uint8_t transfer(uint8_t txData) {
#ifdef USE_SPI_DMA
	   uint8_t recbuf[2]={0};
	  return dmaTransfer(&txData,recbuf,1);
#else

	  unsigned short retry=0;
		while((SPI1->SR&1<<1)==0)//等待发送区空
		{
			retry++;
			if(retry>0XFFFE)return 0;
		}
		SPI1->DR=txData;	 	  //发送一个byte
		retry=0;
		while((SPI1->SR&1<<0)==0) //等待接收完一个byte
		{
			retry++;
			if(retry>0XFFFE)return 0;
		}
		return SPI1->DR;          //返回收到的数据
#endif
  }
};
