#include "usart.h"
#include "stm32f4xx.h"
#include "beep.h"
#include <stdio.h>

void beep_init(void) //配置蜂鸣器
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	 //F8
	 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//速度为50HHZ
	 GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出 
	 GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//下拉
	 GPIO_Init(GPIOB,&GPIO_InitStructure);
	 //GPIO_SetBits(GPIOB,GPIO_Pin_5);  //输出为1
	 GPIO_ResetBits(GPIOB,GPIO_Pin_5);  //输出为0
}
