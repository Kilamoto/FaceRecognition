#include "usart.h"
#include "stm32f4xx.h"
#include "beep.h"
#include <stdio.h>

void beep_init(void) //���÷�����
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	 //F8
	 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//�ٶ�Ϊ50HHZ
	 GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//������� 
	 GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//����
	 GPIO_Init(GPIOB,&GPIO_InitStructure);
	 //GPIO_SetBits(GPIOB,GPIO_Pin_5);  //���Ϊ1
	 GPIO_ResetBits(GPIOB,GPIO_Pin_5);  //���Ϊ0
}
