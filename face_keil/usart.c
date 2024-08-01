#include "usart.h"
#include "stm32f4xx.h"
#include "beep.h"
#include "led.h"

#include <stdio.h>

//更改printf函数内部的代码信息，以使得printf函数能够把数据通过串口发送出去
int fputc(int ch, FILE *fp)
{
	/* 发送单个字节数据到串口 */
	USART_SendData(USART1, (uint8_t)ch);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;

	/* 反馈输出的数据 */
	return ch;
}

//粗略延时函数
void delay(int times)
{
	int i, j;
	
	for(i=0; i<times; i++)
	{
		for(j=0; j<0x1FFF; j++);
	}
}

//串口配置函数: TXD-PA9 和 RXD-PA10
void usart_init(void)
{
	//1、定义变量
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	
	//2、开启时钟
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
		/* Enable USART clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//把普通引脚 映射为 串口通信接口引脚
		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置普通引脚
		/* Configure USART Tx and Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//发送引脚为PA9
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//接收引脚为PA10
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the USART OverSampling by 8 */
		//USART_OverSampling8Cmd(USART1, ENABLE);  
	
	//配置串口通信方式
		USART_InitStructure.USART_BaudRate = 115200;	//波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长
		USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;	//无检验
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//模式为：收发
		USART_Init(USART1, &USART_InitStructure);

		/* NVIC configuration */
		/* Configure the Priority Group to 2 bits */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//选择中断分组为 2组
	
	//配置串口1 的中断
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Enable USART */
	//开启串口通信
		USART_Cmd(USART1, ENABLE);
		
	//开启串口中断	
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void usart2_init(void)
{
		//1、定义变量
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	
	//2、开启时钟
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
		/* Enable USART2 clock */
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//把普通引脚 映射为 串口2 通信接口引脚:  TXD2-PA2     RXD-PA3 
		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置普通引脚
		/* Configure USART Tx and Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//发送引脚为PA2
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//接收引脚为PA3
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the USART OverSampling by 8 */
		//USART_OverSampling8Cmd(USART1, ENABLE);  
	
	//配置串口通信方式
	  USART_InitStructure.USART_BaudRate = 115200;	//波特率:通信速度/速率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长
		USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;	//无检验
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//模式为：收发
		USART_Init(USART2, &USART_InitStructure);

		/* NVIC configuration */
		/* Configure the Priority Group to 2 bits */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//选择中断分组为 2组
	
	//配置串口2 的中断
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Enable USART */
	//开启串口通信
		USART_Cmd(USART2, ENABLE);
		
	//开启串口中断	
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}



//定义变量用以接收数据
char rx_data = 0;


/**				中断的请求处理函数
* @brief  This function handles USRAT interrupt request.
* @param  None
* @retval None
*/
void USART1_IRQHandler(void)
{
	//希望芯片能够及时接收来自电脑/外部设备的数据，所以需要接收中断
  // 判断 串口的 接收中断 是否发送变化 
  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    //清除中断状态以便下次可重新判断
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
    //接收数据（一次接收8位数据）
    rx_data = USART_ReceiveData(USART1);

  }
  
	//判断接收到数据 是否 为 指定的数据，如果是则执行相关动作
	if(rx_data == 'A')
	{
			LED(1, LED0_PIN);	//亮起0灯
	}
	if(rx_data == 'B')
	{
			LED(0, LED0_PIN);	//熄灭0灯
	}
	
	
	
   //把接收到的数据 重新发给  发送者，即 回显 
   USART_SendData(USART1, rx_data );
   
}



//定义变量用以接收数据
char rx2_data = 0;

/**				中断的请求处理函数
* @brief  This function handles USRAT interrupt request.
* @param  None
* @retval None
*/
void USART2_IRQHandler(void)
{
	//希望芯片能够及时接收来自电脑/外部设备的数据，所以需要接收中断
  // 判断 串口的 接收中断 是否发送变化 
  if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
  {
    //清除中断状态以便下次可重新判断
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		
    //接收数据（一次接收8位数据）
    rx2_data = USART_ReceiveData(USART2);

  }
  
	//判断接收到数据 是否 为 指定的数据，如果是则执行相关动作
	if(rx2_data == 'A')
	{
			LED(1, LED0_PIN);	//亮起0灯
		  delay(1000);
			LED(0, LED0_PIN); //熄灭0灯		
	}
	if(rx2_data == 'B')
	{
			LED(1, LED0_PIN);	//熄灭0灯
	}
	//判断接收到数据 是否 为 指定的数据，如果是则执行相关动作
	if(rx2_data == '1')
	{
			GPIO_SetBits(GPIOB,GPIO_Pin_5);//蜂鸣器响
	}
	if(rx2_data == '0')
	{
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//蜂鸣器不响
	}
	
   //把接收到的数据 重新发给  发送者，即 回显 
   USART_SendData(USART1, rx2_data );
   
}
