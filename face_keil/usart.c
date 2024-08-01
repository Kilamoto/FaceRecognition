#include "usart.h"
#include "stm32f4xx.h"
#include "beep.h"
#include "led.h"

#include <stdio.h>

//����printf�����ڲ��Ĵ�����Ϣ����ʹ��printf�����ܹ�������ͨ�����ڷ��ͳ�ȥ
int fputc(int ch, FILE *fp)
{
	/* ���͵����ֽ����ݵ����� */
	USART_SendData(USART1, (uint8_t)ch);

	/* �ȴ�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;

	/* ������������� */
	return ch;
}

//������ʱ����
void delay(int times)
{
	int i, j;
	
	for(i=0; i<times; i++)
	{
		for(j=0; j<0x1FFF; j++);
	}
}

//�������ú���: TXD-PA9 �� RXD-PA10
void usart_init(void)
{
	//1���������
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	
	//2������ʱ��
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
		/* Enable USART clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//����ͨ���� ӳ��Ϊ ����ͨ�Žӿ�����
		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//������ͨ����
		/* Configure USART Tx and Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//��������ΪPA9
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//��������ΪPA10
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the USART OverSampling by 8 */
		//USART_OverSampling8Cmd(USART1, ENABLE);  
	
	//���ô���ͨ�ŷ�ʽ
		USART_InitStructure.USART_BaudRate = 115200;	//������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�
		USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;	//�޼���
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ģʽΪ���շ�
		USART_Init(USART1, &USART_InitStructure);

		/* NVIC configuration */
		/* Configure the Priority Group to 2 bits */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//ѡ���жϷ���Ϊ 2��
	
	//���ô���1 ���ж�
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Enable USART */
	//��������ͨ��
		USART_Cmd(USART1, ENABLE);
		
	//���������ж�	
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void usart2_init(void)
{
		//1���������
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	
	//2������ʱ��
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
		/* Enable USART2 clock */
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//����ͨ���� ӳ��Ϊ ����2 ͨ�Žӿ�����:  TXD2-PA2     RXD-PA3 
		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//������ͨ����
		/* Configure USART Tx and Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//��������ΪPA2
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//��������ΪPA3
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the USART OverSampling by 8 */
		//USART_OverSampling8Cmd(USART1, ENABLE);  
	
	//���ô���ͨ�ŷ�ʽ
	  USART_InitStructure.USART_BaudRate = 115200;	//������:ͨ���ٶ�/����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�
		USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;	//�޼���
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ģʽΪ���շ�
		USART_Init(USART2, &USART_InitStructure);

		/* NVIC configuration */
		/* Configure the Priority Group to 2 bits */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//ѡ���жϷ���Ϊ 2��
	
	//���ô���2 ���ж�
		/* Enable the USARTx Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Enable USART */
	//��������ͨ��
		USART_Cmd(USART2, ENABLE);
		
	//���������ж�	
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}



//����������Խ�������
char rx_data = 0;


/**				�жϵ���������
* @brief  This function handles USRAT interrupt request.
* @param  None
* @retval None
*/
void USART1_IRQHandler(void)
{
	//ϣ��оƬ�ܹ���ʱ�������Ե���/�ⲿ�豸�����ݣ�������Ҫ�����ж�
  // �ж� ���ڵ� �����ж� �Ƿ��ͱ仯 
  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    //����ж�״̬�Ա��´ο������ж�
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
    //�������ݣ�һ�ν���8λ���ݣ�
    rx_data = USART_ReceiveData(USART1);

  }
  
	//�жϽ��յ����� �Ƿ� Ϊ ָ�������ݣ��������ִ����ض���
	if(rx_data == 'A')
	{
			LED(1, LED0_PIN);	//����0��
	}
	if(rx_data == 'B')
	{
			LED(0, LED0_PIN);	//Ϩ��0��
	}
	
	
	
   //�ѽ��յ������� ���·���  �����ߣ��� ���� 
   USART_SendData(USART1, rx_data );
   
}



//����������Խ�������
char rx2_data = 0;

/**				�жϵ���������
* @brief  This function handles USRAT interrupt request.
* @param  None
* @retval None
*/
void USART2_IRQHandler(void)
{
	//ϣ��оƬ�ܹ���ʱ�������Ե���/�ⲿ�豸�����ݣ�������Ҫ�����ж�
  // �ж� ���ڵ� �����ж� �Ƿ��ͱ仯 
  if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
  {
    //����ж�״̬�Ա��´ο������ж�
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		
    //�������ݣ�һ�ν���8λ���ݣ�
    rx2_data = USART_ReceiveData(USART2);

  }
  
	//�жϽ��յ����� �Ƿ� Ϊ ָ�������ݣ��������ִ����ض���
	if(rx2_data == 'A')
	{
			LED(1, LED0_PIN);	//����0��
		  delay(1000);
			LED(0, LED0_PIN); //Ϩ��0��		
	}
	if(rx2_data == 'B')
	{
			LED(1, LED0_PIN);	//Ϩ��0��
	}
	//�жϽ��յ����� �Ƿ� Ϊ ָ�������ݣ��������ִ����ض���
	if(rx2_data == '1')
	{
			GPIO_SetBits(GPIOB,GPIO_Pin_5);//��������
	}
	if(rx2_data == '0')
	{
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);	//����������
	}
	
   //�ѽ��յ������� ���·���  �����ߣ��� ���� 
   USART_SendData(USART1, rx2_data );
   
}
