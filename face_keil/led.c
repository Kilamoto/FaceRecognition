#include "stm32f4xx.h"
#include "led.h"


//���õƵ�����
void led_init(void)
{
	//1������ṹ�����
	//int             a;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//2����������˿������ŵ�ʱ�ӣ�����Э���������ŵĹ������ߵ͵�ƽ�仯��
	//���ô���� ���ú� ���ţ��������������ָ����ƽ
	/* GPIOG Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	//3������ ָ�������źţ�����Ϊ���ģʽ
	/* Configure PG6 and PG8 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN;	//ָ��Ҫ���õ����ź�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//���ģʽ�ľ��巽ʽΪ���������ָ�����ܹ�ֱ������ߵ͵�ƽ��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ߵ͵�ƽ�л����ٶ�
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//��������������Ϊ������Ҫ
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

