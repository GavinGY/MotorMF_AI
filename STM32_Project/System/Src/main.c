/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    14-April-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "dma.h"
#include "led.h"
#include "string.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int OutData[4]; 
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
void OutPut_Data()
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = OutData[i];
    temp1[i] = (u16)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (s8)(temp1[i]%256);
    databuf[i*2+1] = (s8)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  //for(i=0;i<10;i++)
  //uart_putchar(UART4,(char)databuf[i]); 
  HAL_UART_Transmit(&UART1_Handler,(uint8_t*)databuf,10,1000);	//发送接收到的数据
}


//将 int整形 转成 str字符串型
char textbuff[20]={0};
char *int2Str(unsigned int dat)
{
	char temp[20];
    unsigned char i=0,j=0;
	i=0;
	
	do { //从低位（个位）往高位取值，直到到达最高位
		temp[i]=dat%10+0x30;//取值，+0x30转成ASCII码
		i++;
		dat/=10;
	}while(dat); //先do后判断可以包含0的情况
	j=i;
	//倒转字符顺序
	for(i=0;i<j;i++)
		textbuff[i]=temp[j-i-1];
	textbuff[i++]='\0';//字符串最后加上字符结束符
	
	return textbuff;
}

float ADC_ConvertedValueLocal;
// AD转换结果值
uint16_t ADC_ConvertedValue[2000]={0};
uint16_t ADC_ConvertedValue_temp[1500]={0};
uint32_t DMA_Transfer_Complete_Count=0;

int quickSort(int *a,int left,int right);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  //HAL_Init();

  /* Configure the system clock to 64 MHz */
  //SystemClock_Config();


  /* Add your application code here
     */
 	u16 adcx[2]={},adcx_HI;
	u32 ADC_DMA_ConvertedValue[2]={};
	u32 temp[2]={};
	u8 len;	
	u16 times=0;
	u8 Test_RX_BUF[2]={}; 
	
    HAL_Init();                    	 	//初始化HAL库    
    Stm32_Clock_Init(RCC_PLL_MUL9);   	//设置时钟,72M
	delay_init(72);               		//初始化延时函数
	uart_init(115200);					//初始化串口
	//usmart_dev.init(84); 		  	  	//初始化USMART	
	LED_Init();							//初始化LED	
	
	
	MY_ADC_Init();                  	//初始化ADC1通道1
	HAL_ADCEx_Calibration_Start(&ADC1_Handler);					 //校准ADC
	HAL_ADC_Start_DMA(&ADC1_Handler,&ADC_ConvertedValue,sizeof(ADC_ConvertedValue)/sizeof(uint16_t));  
	delay_ms(500);
	
	int fanChek=0;
	
	// 启动 log
	delay_ms(500);
	USART1_Puts("\
	 \
	##########################################################################\
	-----                XB6/XB7 FAN Detection Fixture                   -----\
	-----              Detection of magnetic field motor                 -----\
	-----                                 Firmware Version V1.1-20190327 -----\
	-----    Tools by the Foxconn Cable SoftWare R&D team.    2018/11/21 -----\
	##########################################################################\
	HOW TO USE: \r\n \
	Command \" CHECK \":Normal mode, check the motor state. \r\n \
	Command \" EDIT \":Debug mode, check the value of the zero sliding rheostat. \r\n \
	\
	");
	
	/* Infinite loop */
	while(1)
	{
       // if(USART_RX_STA&0x8000)
		// {					   
			// len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			// printf("\r\n您发送的消息为:\r\n");
			// HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
			// while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
			// printf("\r\n\r\n");//插入换行
			// USART_RX_STA=0;
		// }else
		// {
			// times++;
			// if(times%5000==0)
			// {
				// printf("\r\nALIENTEK MiniSTM32开发板 串口实验\r\n");
				// printf("正点原子@ALIENTEK\r\n\r\n\r\n");
			// }
			// if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
			// if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
			// delay_ms(10);   
		// } 
        //adcx=0x0000;
		//adcx=Get_Adc(ADC_CHANNEL_1);//获取通道1的转换值，20次取平均
		//HAL_ADC_Start(&ADC1_Handler);                               //开启ADC
		//HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换
		//adcx=HAL_ADC_GetValue(&ADC1_Handler);
		
		//LCD_ShowxNum(134,130,adcx,4,16,0);    //显示ADCC采样后的原始值
		// temp=(float)adcx*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
		// adcx=temp;                            //赋值整数部分给adcx变量，因为adcx为u16整形
		//LCD_ShowxNum(134,150,adcx,1,16,0);    //显示电压值的整数部分，3.1111的话，这里就是显示3
		// temp-=adcx;                           //把已经显示的整数部分去掉，留下小数部分，比如3.1111-3=0.1111
		// temp*=1000;                           //小数部分乘以1000，例如：0.1111就转换为111.1，相当于保留三位小数。
		//LCD_ShowxNum(150,150,temp,3,16,0X80); //显示小数部分（前面转换为了整形显示），这里显示的就是111.
		//adcx_HI = adcx;
		//Test_RX_BUF[0] = (uint8_t)((adcx_HI>>8)&0x00ff);
		//Test_RX_BUF[1] = (uint8_t)(adcx&0x00ff);
		//HAL_UART_Transmit(&UART1_Handler,(uint8_t*)Test_RX_BUF,2,1000);	//发送接收到的数据
		//while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
		
		
		
		// u32 temp_val[2]={0};
		// u8 t;
		// for(t=0;t<5;t++)
		// {
		// HAL_ADC_Start_DMA(&ADC1_Handler,(uint32_t*)&ADC_DMA_ConvertedValue,2);
		// temp_val[0]+=ADC_DMA_ConvertedValue[0];
		// temp_val[1]+=ADC_DMA_ConvertedValue[1];
		// delay_ms(5);
		// }
		// temp[0]=temp_val[0]/5;
		// temp[1]=temp_val[1]/5;

		//ADC_ConvertedValueLocal =(float)(ADC_ConvertedValue&0xFFF)*3.3/4096; 	// ADC_ConvertedValue只取最低12有效数据
		//printf("AD转换原始值 = 0x%04X \r\n", ADC_ConvertedValue&0xFFF);     // ADC_ConvertedValue只取最低12有效数据
		//printf("计算得出电压值 = %f V \r\n",ADC_ConvertedValueLocal); 
		//printf("已经完成AD转换次数：%d\n",DMA_Transfer_Complete_Count);
		//DMA_Transfer_Complete_Count=0;
		
		// for(int i=0;i<100;i++)
		// {
			// OutData[0] = 1000;
			// OutPut_Data();
		// }
		// delay_ms(500);	
		
		// for(int i=0;i<5000;i++)
		// {
			// OutData[0] = (u16)(ADC_ConvertedValue[i]&0xFFF);
			// OutPut_Data();
		// }
		
		// for(int i=0;i<100;i++)
		// {
			// OutData[0] = 500;
			// OutPut_Data();
		// }
		
		
		// 连续完全采样一次
		// LED0=!LED0;
		// while(1){
			// LED0=1; //灭
			//复制前1500的数据（DMA采样值，循环采样会覆盖）
			// for(int i=0;i<1500;i++)
				// ADC_ConvertedValue_temp[i]=ADC_ConvertedValue[i];
			//对采样值进行快速排序
			// quickSort(ADC_ConvertedValue_temp,0,1500-1);
			//打印快速排序后的结果
			// for(int i=0;i<1500;i++){
				// OutData[0] = (u16)(ADC_ConvertedValue_temp[i]&0xFFF);
				// OutPut_Data();
			// }
			//打印计算结果
			// USART1_Puts(int2Str(abs(ADC_ConvertedValue_temp[1495] - ADC_ConvertedValue_temp[5]))); 
			// USART1_Puts("\n");
			// LED0=0;//亮
		    // delay_ms(500);	
		// }
		
		
		// 根据需求开启判断
		if(fanChek==1){ //正常模式
			int chekNumber=0;
			for(int j=0;j<10;j++){ //检测10次
				for(int i=0;i<1500;i++)
					ADC_ConvertedValue_temp[i]=ADC_ConvertedValue[i];
				quickSort(ADC_ConvertedValue_temp,0,1500-1);
				if((ADC_ConvertedValue_temp[1495] - ADC_ConvertedValue_temp[5]) > 90)
					chekNumber ++; //满足条件的次数统计
			}
			int chekMid = (ADC_ConvertedValue_temp[1495] + ADC_ConvertedValue_temp[5])/2;
			if(chekMid > 1700 || chekMid < 300){ // 检查电位器AD中间值的区间
			    USART1_Puts("Fixture Error ! Please check the value of the zero sliding rheostat! \n");
				delay_ms(50);	
				USART1_Puts("---> Value of the zero: ");
				USART1_Puts(int2Str(chekMid));
				USART1_Puts("\n");
				delay_ms(50);	
				USART1_Puts("---> send \"EDIT\" go to debug mode, suggest zero value: 1000 ! \n");
				delay_ms(50);	
				USART1_Puts("---> send \"CHECK\" to return to normal mode ! \n");
			}
			else if (chekNumber > 1) // 至少满足2次条件
				USART1_Puts("Detection: OK, FAN is Work ! \n");
			else
				USART1_Puts("Detection: Failed, FAN not Work !!! \n");
			chekNumber = 0;
			fanChek = 0;
		}
		else if (fanChek == 2) {// 调试模式
			for(int i=0;i<1500;i++)
				ADC_ConvertedValue_temp[i]=ADC_ConvertedValue[i];
			quickSort(ADC_ConvertedValue_temp,0,1500-1);
			USART1_Puts("Value of the zero(suggest value 1000): ");
			USART1_Puts(int2Str((ADC_ConvertedValue_temp[1495] + ADC_ConvertedValue_temp[5])/2));
			USART1_Puts("\n");
			delay_ms(500);
		}

	    
		// 测试串口接收和发送
		if((USART_RX_STA & 0xC000) == 0xC000){ //判断高两位是否为11
			USART_RX_BUF[USART_RX_STA & 0X3FFF] = '\0';//加上字符结束符
			//USART1_Puts("USART_RX_STA: ");
			//USART1_Puts(int2Str(USART_RX_STA & 0X3FFF)); //去掉高两位保留其他位数，即为接收的字节数
			USART_RX_STA=0;
			//USART1_Puts("\n");
			//delay_ms(100);
			//USART1_Puts("USART_RX Revive: ");
			//USART1_Puts(USART_RX_BUF); 
			//USART1_Puts("\n"); 
			//delay_ms(100);
			if(!strcasecmp(USART_RX_BUF,"CHECK")){
				USART1_Puts("CHECK Mode: ");
				fanChek = 1;
				delay_ms(100);
			}
			else if(!strcasecmp(USART_RX_BUF,"EDIT")){
				USART1_Puts("------------  EDIT Mode  ------------\n");
				fanChek = 2;
				delay_ms(100);
			}
		}

		
		
		// 连续抽取采样
		// while(1){
			// for(int i=0;i<100;i++)
			// {
				// OutData[0] = (u16)(ADC_ConvertedValue[i*50]&0xFFF);
				// OutPut_Data();
			// }
		// }

		
		//测试整形转字符串，并通过串口发送字符串
		// USART1_Puts(int2Str(0)); USART1_Puts("\n"); delay_ms(100);
		// USART1_Puts(int2Str(1)); USART1_Puts("\n"); delay_ms(100);
		// USART1_Puts(int2Str(10)); USART1_Puts("\n"); delay_ms(100);
		// USART1_Puts(int2Str(188888)); USART1_Puts("\n"); delay_ms(100);
		// USART1_Puts("abcd\n");
		// delay_ms(500);
		
		
		// for(int i = 1000;i>0;i--)
		// {
		  // OutData[0] = i;
		  // OutData[1] = i*2;
		  // OutData[2] = i*3;
		  // OutData[3] = i*4;
		  // OutPut_Data();
		  // delay_ms(100);
		// }
	
		// HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);		//PC13置1 			
		// Delay(0x7FFFFF);
		// HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);	//PC13置0
		// Delay(0x7FFFFF);
	}
}

//快速排序算法
void swap(int *x, int *y) {
    int tmp = *x;
    *x = *y;
    *y = tmp;
}
//分治法把数组分成两份
int patition(int *a, int left,int right) {
    int j = left;    //用来遍历数组
    int i = j - 1;    //用来指向小于基准元素的位置
    int key = a[right];    //基准元素(这里为数组的最后一个元素)
    //从左到右遍历数组，把小于等于基准元素的放到左边，大于基准元素的放到右边
    for (; j < right; ++j) {
        if (a[j] <= key)
            swap(&a[j], &a[++i]);
    }
    //把基准元素放到中间
    swap(&a[right], &a[++i]);
    //返回数组中间位置
    return i;
}
//快速排序 (递归，直到基准情形出现)
int quickSort(int *a,int left,int right) {
    if (left>=right)
        return 0;
    int mid = patition(a,left,right);
    quickSort(a, left, mid - 1);
    quickSort(a, mid + 1, right);
    return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
