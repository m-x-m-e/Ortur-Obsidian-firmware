
#include "mcuvotage.h"
#include <Arduino.h>
//#include <MarlinCore.h>

ADC_HandleTypeDef hadc1;

//NOTE: 由于内核电压的读取会干扰温度传感器,
//所以记得在读取完电压值后还原温度通道的设置
void mcuVotage_init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Common config
  */
  __HAL_RCC_ADC1_CLK_ENABLE();
  RCC->APB2ENR|=1<<9;    //ADC1时钟使能
  	RCC->APB2RSTR|=1<<9;   //ADC1复位
  	RCC->APB2RSTR&=~(1<<9);//复位结束
  	RCC->CFGR&=~(3<<14);   //分频因子清零
  	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
  		//否则将导致ADC准确度下降!
  		RCC->CFGR|=2<<14;

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  ADC_Enable(&hadc1);
  ADC1->CR2|=1<<23;
  ADC1->CR2|=1<<3;       //使能复位校准
  while(ADC1->CR2&1<<3); //等待校准结束
  //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。
  ADC1->CR2|=1<<2;        //开启AD校准
  while(ADC1->CR2&1<<2);  //等待校准结束
}

uint32_t mcuVotage(void)
{
	mcuVotage_init();

	uint32_t adc_value=1;
	uint32_t votage=0;
	HAL_ADC_Start(&hadc1);
	while(ADC1->SR&0X02==0);
	adc_value=HAL_ADC_GetValue(&hadc1);
	if(adc_value)
		votage=1200*4096/adc_value;
	else
		votage = 0;

	//printf("mcuVotage : %d\n",votage);
	return votage;
}
