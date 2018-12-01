#include "adc.h"
#include "delay.h"
#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//ADC��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/5/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

ADC_HandleTypeDef ADC1_Handler;		//ADC���
DMA_HandleTypeDef  ADC1_DMA1_Handler;       //DMA���

//��ʼ��ADC
//ch: ADC_channels 
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
void MY_ADC_Init(void)
{ 
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
    ADC_ChannelConfTypeDef ADC1_ChanConf;
	/* ADCʱ������ */
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC����ʱ��
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//��Ƶ����6ʱ��Ϊ72M/6=12MHz  RCC_ADCPCLK2_DIV6
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);					//����ADCʱ��
	// ADC��������
    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ScanConvMode=ADC_SCAN_DISABLE;                      //��ɨ��ģʽ
    ADC1_Handler.Init.ContinuousConvMode=ENABLE;                 //��������ת��
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //��ֹ����������ģʽ
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //�������
	ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //�Ҷ���
    ADC1_Handler.Init.NbrOfConversion=1;                         //1��ת���ڹ��������� Ҳ����ֻת����������1 
    // ADC1_Handler.Init.NbrOfDiscConversion=0;                  //����������ͨ����Ϊ0
    HAL_ADC_Init(&ADC1_Handler);                                 //��ʼ�� 
	// ���ò���ͨ��
    ADC1_ChanConf.Channel=ADC_CHANNEL_1;                                   //ͨ��
    ADC1_ChanConf.Rank=1;                                       //��1�����У�����1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_55CYCLES_5;//ADC_SAMPLETIME_239CYCLES_5;      //����ʱ��               
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //ͨ������
	
}

//ADC�ײ��������������ã�ʱ��ʹ��
//�˺����ᱻHAL_ADC_Init()����
//hadc:ADC���
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //ʹ��ADC1ʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	__HAL_RCC_DMA1_CLK_ENABLE();			//DMA1ʱ��ʹ�� 
	
    GPIO_Initure.Pin=GPIO_PIN_1;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //ģ��
    GPIO_Initure.Pull=GPIO_NOPULL;          //����������
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
    //Tx DMA����
    ADC1_DMA1_Handler.Instance=DMA1_Channel1;                          //ͨ��ѡ��
    ADC1_DMA1_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //���ݴ��䷽������->�ڴ� 
    ADC1_DMA1_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ�������ַ���� 
    ADC1_DMA1_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ���ڴ��ַ����
    ADC1_DMA1_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;//���ݰ���16λ  DMA_PDATAALIGN_BYTE;//�������ݳ���:8λ
    ADC1_DMA1_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   //���ݰ���16λ  DMA_MDATAALIGN_BYTE;//�洢�����ݳ���:8λ
    ADC1_DMA1_Handler.Init.Mode=DMA_CIRCULAR;                          //ѭ��ģʽ DMA_CIRCULAR DMA_NORMAL;   //������ͨģʽ
    ADC1_DMA1_Handler.Init.Priority=DMA_PRIORITY_HIGH;                 //�����ȼ� 
	HAL_DMA_Init(&ADC1_DMA1_Handler);
    /* ����DMA */
	__HAL_LINKDMA(&ADC1_Handler,DMA_Handle,ADC1_DMA1_Handler);    		//��DMA��ADC��ϵ����(����DMA) 
	/* �����ж����ȼ����ú�ʹ���ж� */
    // HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * ��������: ADC���跴��ʼ������
  * �������: hadc��AD����������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* ����ADC����ʱ�� */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /* ADת��ͨ�����ŷ���ʼ�� */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* DMA���跴��ʼ��*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
}

//���ADCֵ
//ch: ͨ��ֵ 0~16��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
//����ֵ:ת�����
u16 Get_Adc(u32 ch)   
{
    HAL_ADC_Start(&ADC1_Handler);                               //����ADC
	
    HAL_ADC_PollForConversion(&ADC1_Handler,10);                //��ѯת��
 
	return (u16)HAL_ADC_GetValue(&ADC1_Handler);	        	//�������һ��ADC1�������ת�����
}
//��ȡָ��ͨ����ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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


