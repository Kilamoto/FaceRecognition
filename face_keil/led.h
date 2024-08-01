//ͷ�ļ����˺�����ȫ�ֱ������궨�������
#ifndef __LED_H__
#define __LED_H__

//								���źŸ���Ϊ ��Ӧ������� �Ƶ����ż���
//LED0->PG14��LED1->PG13��LED2->PG6��LED3->PG11��
#define LED0_PIN   GPIO_Pin_14
#define LED1_PIN   GPIO_Pin_13
#define LED2_PIN   GPIO_Pin_6
#define LED3_PIN   GPIO_Pin_11

#define LED3_ON GPIO_ResetBits(GPIOG, LED3_PIN)
#define LED3_OFF GPIO_SetBits(GPIOG, LED3_PIN)

//(�����Ƿ����)? ��������ִ�����1��:����������ִ�����2����
#define LED3(X)	(X)?(GPIO_ResetBits(GPIOG, LED3_PIN)):(GPIO_SetBits(GPIOG, LED3_PIN))

#define LED(X,GPIO_Pin)	(X)?(GPIO_ResetBits(GPIOG, GPIO_Pin)):(GPIO_SetBits(GPIOG, GPIO_Pin))


//��������
void led_init(void);


#endif
