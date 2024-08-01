#include "key.h"
#include "stm32f4xx.h"

//配置按键的引脚和相应的中断线,按键的引脚号：PG2、PG3、PG4、PG5
void key_init(void)
{
	//定义结构体变量
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* Enable GPIOG clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = KEY0_PIN | KEY1_PIN | KEY2_PIN | KEY3_PIN;	//指定要配置的引脚号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//配置为输入模式
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//输出模式的具体方式为推挽输出（指引脚能够直接输出高低电平）
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//高低电平切换的速度
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//设置上下拉电阻为：不需要
	
	//void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
	GPIO_Init(GPIOG, &GPIO_InitStructure);	//利用GPIO_Init()，根据参数2结构体变量里的成员信息，配置参数1端口组，
	
	//普通引脚 与 中断线 建立映射关系
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

//(紧急处理函数)中断请求服务函数的名称是不可改（在官方的启动文件里限定了），函数不得声明，函数不得传参，函数不得返回，
void EXTI2_IRQHandler(void)	//连接按键 PG2 的 中断线
{
	//判断 中断2 号线（与按键PG2连接的线）  是否发生中断变化
  //if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	if(EXTI_GetITStatus(EXTI_Line2) == SET)
  {
		
		//适当的延时
		delay_ms(20);
		
    /* 做相关事情 */
		times -= 200;
    if(times == 0)
					times = 1000;
		
    /* Clear the EXTI line 2 pending bit */
		//清除中断2号线的中断状态，以便下次触发中断了，能够再次检测到变化。
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

