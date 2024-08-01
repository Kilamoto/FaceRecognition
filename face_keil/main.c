#include <stdio.h>
#include "stm32f4xx.h"	//�����ٷ�ͷ�ļ����Ա�ʹ�ùٷ��ṩ����ش��루ͷ�ļ����˺�����ȫ�ֱ������궨��������� 
#include "led.h"
#include "key.h"
#include "usart.h"
#include "beep.h"
#include <stdio.h>

//���� ʱ�� ȫ�ֱ���������.c����/���ã�����������ļ�����extern�ⲿ����������������ļ����ã�
int times = 1000;

//������ʱ����
void delay_ms(int times)
{
	int i, j;
	
	for(i=0; i<times; i++)
	{
		for(j=0; j<0x1FFF; j++);
	}
}

//������
int main(void)
{
	//���õƵ�����
	led_init();
	
	//���ð��������ź���Ӧ���ж���
	key_init();
	
	//������
	beep_init();
	
	//�������ú���
	usart_init();	//���ô���1�ģ���������Խ���ͨ�ŵ�
	usart2_init();	//���ô���2�ģ��������ⲿ�豸����ͨ�ŵ�
	
	printf("���й���ģ�������ú��ˡ�\r\n");
	LED(0, LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN);	//������е�
	
	//��Ƭ�����������������Ա�оƬһֱ����Ƿ�����ض�����
	while(1)
	{
//			LED(1, LED0_PIN);	//����0��
//			delay_ms(times);
//		
//			LED(1, LED1_PIN);	//����1��
//			delay_ms(times);
//		
//			LED(1, LED2_PIN);	//����2��
//			delay_ms(times);
//		
//			LED(1, LED3_PIN);	//����3��
//			delay_ms(times);
//		
//			LED(0, LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN);	//������е�
//			delay_ms(times);
	}
}

//		/* Set PG6 and PG8 ���øߵ�ƽ */
//		//GPIOG->BSRRL = LED1_PIN | LED2_PIN;
//		//void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//		//GPIO_SetBits(GPIOG, LED1_PIN);	//ֱ�Ӹ�ָ�������� �ߵ�ƽ������Ϊ�ߵ�ƽ����ƣ�
//				
//		
//		
//		/* Reset PG6 and PG8 ���õ͵�ƽ */
//		//GPIOG->BSRRH = LED1_PIN | LED2_PIN;
//		GPIO_ResetBits(GPIOG, LED0_PIN);	//ֱ�Ӹ�ָ�������� �͵�ƽ������Ϊ�͵�ƽ�����ƣ�
//		GPIO_ResetBits(GPIOG, LED1_PIN);
//		GPIO_ResetBits(GPIOG, LED2_PIN);
//		GPIO_ResetBits(GPIOG, LED3_PIN);	//����3��
//		LED3_ON;	//����3��
//		LED3(1);	//����3��
