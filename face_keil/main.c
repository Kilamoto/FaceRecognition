#include <stdio.h>
#include "stm32f4xx.h"	//包含官方头文件，以便使用官方提供的相关代码（头文件作了函数、全局变量、宏定义的声明） 
#include "led.h"
#include "key.h"
#include "usart.h"
#include "beep.h"
#include <stdio.h>

//定义 时间 全局变量（整个.c可视/可用，如果在其他文件作了extern外部声明，则可在其他文件可用）
int times = 1000;

//粗略延时函数
void delay_ms(int times)
{
	int i, j;
	
	for(i=0; i<times; i++)
	{
		for(j=0; j<0x1FFF; j++);
	}
}

//主函数
int main(void)
{
	//配置灯的引脚
	led_init();
	
	//配置按键的引脚和相应的中断线
	key_init();
	
	//蜂鸣器
	beep_init();
	
	//串口配置函数
	usart_init();	//配置串口1的，用于与电脑进行通信的
	usart2_init();	//配置串口2的，用于与外部设备进行通信的
	
	printf("所有功能模块已配置好了。\r\n");
	LED(0, LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN);	//灭掉所有灯
	
	//单片机不允许程序结束，以便芯片一直检查是否有相关动作。
	while(1)
	{
//			LED(1, LED0_PIN);	//亮起0灯
//			delay_ms(times);
//		
//			LED(1, LED1_PIN);	//亮起1灯
//			delay_ms(times);
//		
//			LED(1, LED2_PIN);	//亮起2灯
//			delay_ms(times);
//		
//			LED(1, LED3_PIN);	//亮起3灯
//			delay_ms(times);
//		
//			LED(0, LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN);	//灭掉所有灯
//			delay_ms(times);
	}
}

//		/* Set PG6 and PG8 设置高电平 */
//		//GPIOG->BSRRL = LED1_PIN | LED2_PIN;
//		//void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//		//GPIO_SetBits(GPIOG, LED1_PIN);	//直接给指定的引脚 高电平（引脚为高电平：灭灯）
//				
//		
//		
//		/* Reset PG6 and PG8 设置低电平 */
//		//GPIOG->BSRRH = LED1_PIN | LED2_PIN;
//		GPIO_ResetBits(GPIOG, LED0_PIN);	//直接给指定的引脚 低电平（引脚为低电平：亮灯）
//		GPIO_ResetBits(GPIOG, LED1_PIN);
//		GPIO_ResetBits(GPIOG, LED2_PIN);
//		GPIO_ResetBits(GPIOG, LED3_PIN);	//亮起3灯
//		LED3_ON;	//亮起3灯
//		LED3(1);	//亮起3灯
