/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/*
static uint8_t delaying_times = 0;//叠加执行延时的次数
static uint16_t delaying_finish = 0;//记录最多16个的递归溢出事件中，每一个是否都已经记数溢出
void delay_ms(uint16_t nms)
{
	uint32_t last_systick_val;
	if(delaying_times != 0)//如果主程序在跑delay函数的过程中，发生中断并在中断中又进入了delay函数
	{
		last_systick_val = SysTick->VAL;//将上次的计数器的值保存下来以便退出中断后回去时可以从该值继续递减
		//如果上次记数已经溢出，代表着上次的delay已经记数完成，将该次溢出事件记录下来，以便出了中断回到原delay函数时，可以直接跳出while
		//delaying_finish是16位的，最多可以记录16次溢出事件，即16层的递归
		if(SysTick->CTRL & (1 << 16))delaying_finish |= (1 << (delaying_times - 1));
	}
	delaying_times ++;
	SysTick->LOAD = (uint32_t)fac_ms * nms;//自动重装载值
	SysTick->VAL = 0x00;//清除计时器的值
	SysTick->CTRL |= (1 << 0);//SysTick使能，使能后定时器开始倒数
	while(!(SysTick->CTRL & (1 << 16)))//判断是否减到0，减到0时CTRL的第16位会置1，读取后会自动置0
	{
		//如果在中断中计数器已经溢出，就退出while,并且对应中断位清零
		if(delaying_finish & (1 << (delaying_times- 1)))
		{
			delaying_finish &= ~(1 << (delaying_times- 1));
			break;
		}
	}
	delaying_times --;
	if(delaying_times == 0)
	{
		SysTick->CTRL &= ~(1 << 0);//关闭SysTick，关闭后记数器将不再倒数
		SysTick->VAL = 0x00;//清除计时器的值（执行关闭SysTick程序时，记数器又开始了新一轮的倒数，所以关闭后记数器的值不为0）
	}
	else
	{
		//读取CTRL寄存器的同时，CTRL的第16位会变为0，关闭SysTick后给VAL寄存器赋值再使能的原因
		//1.若未关闭SysTick，且先将CTRL的第16位清零后再给VAL寄存器赋值，则在赋值的过程中计数器可能会记数到0，从而导致CTRL的第16位又被置1
		//2.若未关闭SysTick，且先给VAL寄存器赋值后再将CTRL的第16位清零，则在清零的过程中计数器会继续递减并且可能在CTRL的第16位完成清零前就溢出
		//所以必须关闭SysTick，且赋值完需要再使能使得递归回原函数的while中计数器会继续递减
		
		SysTick->CTRL &= ~(1 << 0);//关闭SysTick，关闭后记数器将不再倒数
		SysTick->LOAD = last_systick_val;
		SysTick->VAL = 0x00;//清除计时器的值
		SysTick->CTRL |= (1 << 0);//SysTick使能，使能后定时器开始倒数
	}
}
*/

//1ms延时函数
void my_delay_nms(uint16_t time)
{
	uint16_t i = 0;
	while (time--)
	{
		i = 12000;
		while (i--);
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
