#ifndef __KEY_H__
#define __KEY_H__

//�궨�������
#define KEY0_PIN   GPIO_Pin_2
#define KEY1_PIN   GPIO_Pin_3
#define KEY2_PIN   GPIO_Pin_4
#define KEY3_PIN   GPIO_Pin_5

//ȫ�ֱ�����������ʹ���ⲿ������
extern int times;	//���ùؼ���extern������������������ʱ���ø�ֵ��

//����������
void key_init(void);

void delay_ms(int times);//���������󣬾Ϳ�ֱ��ʹ���ⲿ�ļ��ṩ�ĺ�����

#endif
