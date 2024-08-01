#ifndef __KEY_H__
#define __KEY_H__

//宏定义的声明
#define KEY0_PIN   GPIO_Pin_2
#define KEY1_PIN   GPIO_Pin_3
#define KEY2_PIN   GPIO_Pin_4
#define KEY3_PIN   GPIO_Pin_5

//全局变量的声明（使用外部变量）
extern int times;	//利用关键字extern作变量声明，且声明时不得赋值；

//函数的声明
void key_init(void);

void delay_ms(int times);//声明函数后，就可直接使用外部文件提供的函数了

#endif
