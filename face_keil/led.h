//头文件作了函数、全局变量、宏定义的声明
#ifndef __LED_H__
#define __LED_H__

//								引脚号更改为 对应开发板的 灯的引脚即可
//LED0->PG14、LED1->PG13、LED2->PG6、LED3->PG11、
#define LED0_PIN   GPIO_Pin_14
#define LED1_PIN   GPIO_Pin_13
#define LED2_PIN   GPIO_Pin_6
#define LED3_PIN   GPIO_Pin_11

#define LED3_ON GPIO_ResetBits(GPIOG, LED3_PIN)
#define LED3_OFF GPIO_SetBits(GPIOG, LED3_PIN)

//(条件是否成立)? （成立则执行语句1）:（不成立则执行语句2）；
#define LED3(X)	(X)?(GPIO_ResetBits(GPIOG, LED3_PIN)):(GPIO_SetBits(GPIOG, LED3_PIN))

#define LED(X,GPIO_Pin)	(X)?(GPIO_ResetBits(GPIOG, GPIO_Pin)):(GPIO_SetBits(GPIOG, GPIO_Pin))


//声明函数
void led_init(void);


#endif
