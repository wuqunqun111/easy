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
static uint8_t delaying_times = 0;//����ִ����ʱ�Ĵ���
static uint16_t delaying_finish = 0;//��¼���16���ĵݹ�����¼��У�ÿһ���Ƿ��Ѿ��������
void delay_ms(uint16_t nms)
{
	uint32_t last_systick_val;
	if(delaying_times != 0)//�������������delay�����Ĺ����У������жϲ����ж����ֽ�����delay����
	{
		last_systick_val = SysTick->VAL;//���ϴεļ�������ֵ���������Ա��˳��жϺ��ȥʱ���ԴӸ�ֵ�����ݼ�
		//����ϴμ����Ѿ�������������ϴε�delay�Ѿ�������ɣ����ô�����¼���¼�������Ա�����жϻص�ԭdelay����ʱ������ֱ������while
		//delaying_finish��16λ�ģ������Լ�¼16������¼�����16��ĵݹ�
		if(SysTick->CTRL & (1 << 16))delaying_finish |= (1 << (delaying_times - 1));
	}
	delaying_times ++;
	SysTick->LOAD = (uint32_t)fac_ms * nms;//�Զ���װ��ֵ
	SysTick->VAL = 0x00;//�����ʱ����ֵ
	SysTick->CTRL |= (1 << 0);//SysTickʹ�ܣ�ʹ�ܺ�ʱ����ʼ����
	while(!(SysTick->CTRL & (1 << 16)))//�ж��Ƿ����0������0ʱCTRL�ĵ�16λ����1����ȡ����Զ���0
	{
		//������ж��м������Ѿ���������˳�while,���Ҷ�Ӧ�ж�λ����
		if(delaying_finish & (1 << (delaying_times- 1)))
		{
			delaying_finish &= ~(1 << (delaying_times- 1));
			break;
		}
	}
	delaying_times --;
	if(delaying_times == 0)
	{
		SysTick->CTRL &= ~(1 << 0);//�ر�SysTick���رպ�����������ٵ���
		SysTick->VAL = 0x00;//�����ʱ����ֵ��ִ�йر�SysTick����ʱ���������ֿ�ʼ����һ�ֵĵ��������Թرպ��������ֵ��Ϊ0��
	}
	else
	{
		//��ȡCTRL�Ĵ�����ͬʱ��CTRL�ĵ�16λ���Ϊ0���ر�SysTick���VAL�Ĵ�����ֵ��ʹ�ܵ�ԭ��
		//1.��δ�ر�SysTick�����Ƚ�CTRL�ĵ�16λ������ٸ�VAL�Ĵ�����ֵ�����ڸ�ֵ�Ĺ����м��������ܻ������0���Ӷ�����CTRL�ĵ�16λ�ֱ���1
		//2.��δ�ر�SysTick�����ȸ�VAL�Ĵ�����ֵ���ٽ�CTRL�ĵ�16λ���㣬��������Ĺ����м�����������ݼ����ҿ�����CTRL�ĵ�16λ�������ǰ�����
		//���Ա���ر�SysTick���Ҹ�ֵ����Ҫ��ʹ��ʹ�õݹ��ԭ������while�м�����������ݼ�
		
		SysTick->CTRL &= ~(1 << 0);//�ر�SysTick���رպ�����������ٵ���
		SysTick->LOAD = last_systick_val;
		SysTick->VAL = 0x00;//�����ʱ����ֵ
		SysTick->CTRL |= (1 << 0);//SysTickʹ�ܣ�ʹ�ܺ�ʱ����ʼ����
	}
}
*/

//1ms��ʱ����
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
