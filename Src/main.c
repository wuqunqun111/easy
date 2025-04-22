/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32_dsp.h"
#include "table_fft.h"
#include "math.h"
#include "oled.h"
#include "config.h"
#include "bg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define NPT 256
#define PI2 6.28318530717959
//采样率计算
//分辨率：Fs/NPT 
//#define Fs	10000
#define Fs	9984
//取9984能出来整数的分辨率 9984/256 = 39Hz

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Creat_Single(void);
void GetPowerMag(void);
void Single_Get(void);
void display1(void);
void display2(void);
void Key_Scan(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t adc_buf[NPT]={0};

long lBufInArray[NPT];
long lBufOutArray[NPT/2];
long lBufMagArray[NPT/2];


uint8_t prt = 10;	            //量化显示的比例
#define SHOW_NUM 4				//显示函数的个数
uint8_t display_num = 2;	    //控制显示方式的
uint8_t auto_display_flag = 0;	//自动切换显示标志 1：自动切换 0:手动
uint8_t fall_pot[128];	        //记录下落点的坐标

#define SHOW_NUM_MAX  50        //比较最大值adc采样采样次数
uint8_t show_num_count = 0;     //采样计数
long    show_max_f = 0;         //每n次adc采样比较的最大值

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	printf("uart test! \r\n");
	
	/*初始化显示*/
	GUI_Initialize();
	/*设置前景色和背景色 这里用1和0代替*/
	GUI_SetColor(1,0);
	//GUI_LoadPic(0,0,(uint8_t *)&gImage_bg,128,64);
	GUI_PutString(20,32,gTITLE_bg);
	GUI_Exec();
	HAL_Delay(3000);
	
	//初始化下落点 把下落的点 初始化为最底部显示
	for(i=0;i<128;i++)
		fall_pot[i] = 63;
	
	/*启动ADC的DMA传输 配合下面定时器来触发ADC转换*/
	HAL_ADC_Start_DMA(&hadc1, adc_buf, NPT);
	/*开启定时器 用溢出事件来触发ADC转换*/
	HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		Key_Scan();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/************FFT相关*****************/
//测试用 生成一个信号
void Creat_Single(void)
{
	u16 i = 0;
	float fx=0.0;
	
	for(i=0; i<NPT; i++)
	{
		fx = 2048+2048*sin(PI2 * i * 200.0 / Fs)+
				 3100*sin(PI2 * i * 502.0 / Fs)+
				 1300*sin(PI2 * i * 990.0 / Fs);
		lBufInArray[i] = ((signed short)fx) << 16;		
	}
}
//获取FFT后的直流分量
void GetPowerMag(void)
{
    signed short lX,lY;
    float X,Y,Mag;
    unsigned short i;
    for(i=0; i<NPT/2; i++)
    {
        lX  = (lBufOutArray[i] << 16) >> 16;
        lY  = (lBufOutArray[i] >> 16);
			
				//除以32768再乘65536是为了符合浮点数计算规律
        X = NPT * ((float)lX) / 32768;
        Y = NPT * ((float)lY) / 32768;
        Mag = sqrt(X * X + Y * Y)*1.0/ NPT;
        if(i == 0)	
            lBufMagArray[i] = (unsigned long)(Mag * 32768);
        else
            lBufMagArray[i] = (unsigned long)(Mag * 65536);
    }
}
/*柱状显示*/
void display1(void)
{
	uint16_t i = 0;
	uint8_t x = 0;
	uint8_t y = 0;
	long lBufMagArray_all = 0;
	char show_text[32] = {0};
	
	/*******************显示*******************/
	GUI_ClearSCR();
	for(i = 0; i < 32; i++)	//间隔的取32个频率出来显示
	{
		x = (i<<2);	//i*4
		y = 63-(lBufMagArray[x+1]/prt)-2;	//加1是为了丢掉第一个直流分量
		if(y>63) y = 63;
		
		GUI_LineWith(x,y,x,63,3,1);
		
		//画下落的点
		if(fall_pot[i]>y) fall_pot[i] = y;
		else
		{
				if(fall_pot[i]>63) fall_pot[i]=63;
				GUI_LineWith(x,fall_pot[i],x,fall_pot[i]+3,3,1);
				fall_pot[i] += 2 ;
		}
		
		lBufMagArray_all += lBufMagArray[i];
	}

	sprintf(show_text, "frequency : %.2f", (float)lBufMagArray_all/32);
	GUI_PutString(0,0, show_text);
	
	GUI_Exec();
}
/*单柱状显示*/
void display2(void)
{
	uint16_t i = 0;
	uint8_t y = 0;
	//long lBufMagArray_all = 0;
	long show_current_max = 0;
	char show_text[32] = {0};
	
	/*******************显示*******************/
	GUI_ClearSCR();
	for(i = 1; i < 128; i++)	
	{
		y = 63-(lBufMagArray[i]/prt)-2;
		if(y>63) y = 63;
		
		GUI_RLine(i,y,63,1);
		//画下落的点
		if(fall_pot[i]>y) fall_pot[i] = y;
		else
		{
				if(fall_pot[i]>63) fall_pot[i]=63;
				GUI_RLine(i,fall_pot[i],fall_pot[i]+1,1);
				fall_pot[i] += 2 ;
		}
		
		//lBufMagArray_all += lBufMagArray[i];
		if(show_current_max < lBufMagArray[i])
			show_current_max = lBufMagArray[i];
	}
	
	//sprintf(show_text, "frequency : %.2f", (float)lBufMagArray_all/127);
	if(show_num_count++ > SHOW_NUM_MAX)
	{
		show_num_count = 0;
		show_max_f = 0;
	}
	if(show_max_f < show_current_max)
		show_max_f = show_current_max;
	sprintf(show_text, "cur:%ldHz | max:%ldHz", show_current_max, show_max_f);
	GUI_PutString(0,0, show_text);
	
	
	//if(lBufMagArray_current_max > 100)
	//	my_delay_nms(1000);
	
	GUI_Exec();
}
/*柱状显示 中间对称*/
void display3(void)
{
	uint16_t i = 0;
	uint8_t y = 0;
	
	/*******************显示*******************/
	GUI_ClearSCR();
	for(i = 0; i < 127; i++)	
	{
		y = 31-(lBufMagArray[i+1]/prt)-2;	//加1是为了丢掉第一个直流分量
		if(y>31) y = 31;
		
		GUI_RLine(i,32,y,1);
		GUI_RLine(i,32,63-y,1);
		
		//画下落的点
		if(fall_pot[i]>y) fall_pot[i] = y;
		else
		{
				if(fall_pot[i]>30) fall_pot[i]=30;
				GUI_RLine(i,fall_pot[i],fall_pot[i]+1,1);
				GUI_RLine(i,63-fall_pot[i],63-(fall_pot[i]+1),1);
				fall_pot[i] += 2 ;
		}
	}
	GUI_Exec();
}
/*单柱状显示 中间对称*/
void display4(void)
{
	uint16_t i = 0;
	uint8_t x = 0;
	uint8_t y = 0;
	
	/*******************显示*******************/
	GUI_ClearSCR();
	for(i = 0; i < 32; i++)	//间隔的取32个频率出来显示
	{
		x = (i<<2);	//i*4
		y = 31-(lBufMagArray[x+1]/prt)-2;	//加1是为了丢掉第一个直流分量
		if(y>31) y = 31;
		
		GUI_LineWith(x,y,x,32,3,1);
		GUI_LineWith(x,63-y,x,32,3,1);
		
		//画下落的点
		if(fall_pot[i]>y) fall_pot[i] = y;
		else
		{
				if(fall_pot[i]>31) fall_pot[i]=31;
				GUI_LineWith(x,fall_pot[i],x,fall_pot[i]+3,3,1);
				GUI_LineWith(x,63 - fall_pot[i],x,63 - fall_pot[i]-3,3,1);
				fall_pot[i] += 2 ;
		}
	}
	GUI_Exec();
}


//ADC DMA传输中断
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t i = 0;
	static uint16_t num = 0;
	
//	printf("adc dma interrupt \r\n");
	HAL_ADC_Stop_DMA(&hadc1);							         //完成一次测量 关闭DMA传输
	
	//填充数组
	for(i=0;i<NPT;i++)
		lBufInArray[i] = ((signed short)(adc_buf[i]-2048)) << 16;//这里因为单片机的ADC只能测正的电压 所以需要前级加直流偏执
	//加入直流偏执后 软件上减去2048即一半 达到负半周期测量的目的
	//cr4_fft_1024_stm32(lBufOutArray, lBufInArray, NPT);		 //FFT变换
	cr4_fft_256_stm32(lBufOutArray, lBufInArray, NPT);
	GetPowerMag();												 //取直流分量对应的AD值
//	//打印出来测试
//	for(i=0;i<NPT/2;i++)
//		printf("i:%3d, f:%.2f, Power:%10d\r\n", i, (float)i*Fs/NPT, lBufMagArray[i]);
	
	//自动显示
	if(auto_display_flag == 1)
	{
		if(num>300)
		{
			num = 0;
			display_num ++;
			if(display_num>SHOW_NUM) display_num = 1;
		}
	}
	num++;
	//显示
	switch(display_num)
	{
		case 1:
			display1();
			break;
		case 2:
			display2();
			break;
		case 3:
			display3();
			break;
		case 4:
			display4();
			break;
		default:
			display3();
			break;
	}
	HAL_ADC_Start_DMA(&hadc1, adc_buf, NPT);
}

#define K1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
#define K2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)
#define K3 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)

void Key_Scan(void)
{
	static uint8_t mode_num = 0;
	if(K1 == RESET)
	{
		HAL_Delay(10);
		if(K1 == RESET)
		{
			while(!K1);
			mode_num=!mode_num;
			if(mode_num == 1) //自动显示模式
			{
				auto_display_flag = 1;
				GUI_PutString(0,0,"Auto");
				GUI_Exec();
			}
			else							//正常显示模式 手动切换效果
			{
				auto_display_flag = 0;
				GUI_PutString(0,0,"Manual");
				GUI_Exec();
			}
		}
	}
	if(K2 == RESET)
	{
		HAL_Delay(10);
		if(K2 == RESET)
		{
			while(!K2);
			if(mode_num == 0)	//手动模式 
			{
				display_num ++;
				if(display_num > SHOW_NUM) display_num = 1;
			}
		}
	}
	if(K3 == RESET)
	{
		HAL_Delay(10);
		if(K3 == RESET)
		{
			while(!K3);
			if(mode_num == 0)	//手动模式 
			{
				if(display_num == 1) display_num = SHOW_NUM+1;
				display_num --;
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
