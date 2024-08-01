#include "stm32f4xx.h"
#include "led.h"


//配置灯的引脚
void led_init(void)
{
	//1、定义结构体变量
	//int             a;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//2、开启具体端口组引脚的时钟，用以协调控制引脚的工作（高低电平变化）
	//调用代码段 配置好 引脚，即可让引脚输出指定电平
	/* GPIOG Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	//3、配置 指定的引脚号，配置为输出模式
	/* Configure PG6 and PG8 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN;	//指定要配置的引脚号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//配置为输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//输出模式的具体方式为推挽输出（指引脚能够直接输出高低电平）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//高低电平切换的速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//设置上下拉电阻为：不需要
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

