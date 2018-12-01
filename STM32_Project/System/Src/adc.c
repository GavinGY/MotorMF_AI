#include "adc.h"
#include "delay.h"
#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//ADC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/5/29
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

ADC_HandleTypeDef ADC1_Handler;		//ADC句柄
DMA_HandleTypeDef  ADC1_DMA1_Handler;       //DMA句柄

//初始化ADC
//ch: ADC_channels 
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
void MY_ADC_Init(void)
{ 
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
    ADC_ChannelConfTypeDef ADC1_ChanConf;
	/* ADC时钟配置 */
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC外设时钟
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//分频因子6时钟为72M/6=12MHz  RCC_ADCPCLK2_DIV6
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);					//设置ADC时钟
	// ADC功能配置
    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ScanConvMode=ADC_SCAN_DISABLE;                      //非扫描模式
    ADC1_Handler.Init.ContinuousConvMode=ENABLE;                 //开启连续转换
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
	ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
    ADC1_Handler.Init.NbrOfConversion=1;                         //1个转换在规则序列中 也就是只转换规则序列1 
    // ADC1_Handler.Init.NbrOfDiscConversion=0;                  //不连续采样通道数为0
    HAL_ADC_Init(&ADC1_Handler);                                 //初始化 
	// 配置采样通道
    ADC1_ChanConf.Channel=ADC_CHANNEL_1;                                   //通道
    ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_55CYCLES_5;//ADC_SAMPLETIME_239CYCLES_5;      //采样时间               
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置
	
}

//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
//hadc:ADC句柄
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //使能ADC1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	__HAL_RCC_DMA1_CLK_ENABLE();			//DMA1时钟使能 
	
    GPIO_Initure.Pin=GPIO_PIN_1;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
    //Tx DMA配置
    ADC1_DMA1_Handler.Instance=DMA1_Channel1;                          //通道选择
    ADC1_DMA1_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //数据传输方向：外设->内存 
    ADC1_DMA1_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式，外设地址不变 
    ADC1_DMA1_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式，内存地址递增
    ADC1_DMA1_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;//数据半字16位  DMA_PDATAALIGN_BYTE;//外设数据长度:8位
    ADC1_DMA1_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   //数据半字16位  DMA_MDATAALIGN_BYTE;//存储器数据长度:8位
    ADC1_DMA1_Handler.Init.Mode=DMA_CIRCULAR;                          //循环模式 DMA_CIRCULAR DMA_NORMAL;   //外设普通模式
    ADC1_DMA1_Handler.Init.Priority=DMA_PRIORITY_HIGH;                 //高优先级 
	HAL_DMA_Init(&ADC1_DMA1_Handler);
    /* 连接DMA */
	__HAL_LINKDMA(&ADC1_Handler,DMA_Handle,ADC1_DMA1_Handler);    		//将DMA与ADC联系起来(发送DMA) 
	/* 外设中断优先级配置和使能中断 */
    // HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * 函数功能: ADC外设反初始化配置
  * 输入参数: hadc：AD外设句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* 禁用ADC外设时钟 */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /* AD转换通道引脚反初始化 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* DMA外设反初始化*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
}

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
u16 Get_Adc(u32 ch)   
{
    HAL_ADC_Start(&ADC1_Handler);                               //开启ADC
	
    HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换
 
	return (u16)HAL_ADC_GetValue(&ADC1_Handler);	        	//返回最近一次ADC1规则组的转换结果
}
//获取指定通道的转换值，取times次,然后平均 
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u32 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		//delay_ms(5);
	}
	return temp_val/times;
} 


