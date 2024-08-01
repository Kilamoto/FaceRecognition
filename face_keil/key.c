#include "key.h"
#include "stm32f4xx.h"

//���ð��������ź���Ӧ���ж���,���������źţ�PG2��PG3��PG4��PG5
void key_init(void)
{
	//����ṹ�����
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* Enable GPIOG clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = KEY0_PIN | KEY1_PIN | KEY2_PIN | KEY3_PIN;	//ָ��Ҫ���õ����ź�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//����Ϊ����ģʽ
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//���ģʽ�ľ��巽ʽΪ���������ָ�����ܹ�ֱ������ߵ͵�ƽ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ߵ͵�ƽ�л����ٶ�
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//��������������Ϊ������Ҫ
	
	//void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
	GPIO_Init(GPIOG, &GPIO_InitStructure);	//����GPIO_Init()�����ݲ���2�ṹ�������ĳ�Ա��Ϣ�����ò���1�˿��飬
	
	//��ͨ���� �� �ж��� ����ӳ���ϵ
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource2 );
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource3 );
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource4 );
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource5 );
	
	/* Configure EXTI Line2\3\4\5 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line3 | EXTI_Line4 | EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI Line2\3\4\5 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//(����������)�ж�����������������ǲ��ɸģ��ڹٷ��������ļ����޶��ˣ������������������������ô��Σ��������÷��أ�
void EXTI2_IRQHandler(void)	//���Ӱ��� PG2 �� �ж���
{
	//�ж� �ж�2 ���ߣ��밴��PG2���ӵ��ߣ�  �Ƿ����жϱ仯
  //if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
  {
		
		//�ʵ�����ʱ
		delay_ms(20);
		
    /* ��������� */
		times -= 200;
    if(times == 0)
					times = 1000;
		
    /* Clear the EXTI line 2 pending bit */
		//����ж�2���ߵ��ж�״̬���Ա��´δ����ж��ˣ��ܹ��ٴμ�⵽�仯��
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}


void EXTI3_IRQHandler(void)
{

}

void EXTI4_IRQHandler(void)
{

}

void EXTI9_5_IRQHandler(void)
{

}

