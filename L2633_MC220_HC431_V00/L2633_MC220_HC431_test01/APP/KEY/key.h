#ifndef __key_H__
#define __key_H__

#include "stm32f0xx.h"
#include "system.h"

//L2158控制M1/M2, HC225控制M3,M4由MOC控制
#define 	KEY1				 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)
#define 	KEY2				 GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_0)
#define 	KEY3				 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)
#define 	KEY4				 GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)
#define 	KEY5				 GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1)
#define 	KEY6				 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)
#define 	KEY7				 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)


#define 	M1Out			{GPIO_SetBits(GPIOB, GPIO_Pin_12);   GPIO_ResetBits(GPIOA, GPIO_Pin_3);}
#define 	M1In			{GPIO_ResetBits(GPIOB, GPIO_Pin_12); GPIO_SetBits(GPIOA, GPIO_Pin_3);}
#define 	M1Close			{GPIO_ResetBits(GPIOB, GPIO_Pin_12); GPIO_ResetBits(GPIOA, GPIO_Pin_3);}

#define 	M2Out			{GPIO_SetBits(GPIOB, GPIO_Pin_10);   GPIO_ResetBits(GPIOA, GPIO_Pin_6);}
#define 	M2In			{GPIO_ResetBits(GPIOB, GPIO_Pin_10); GPIO_SetBits(GPIOA, GPIO_Pin_6);}
#define 	M2Close			{GPIO_ResetBits(GPIOB, GPIO_Pin_10); GPIO_ResetBits(GPIOA, GPIO_Pin_6);}

#define		OpenAirbag		{GPIO_ResetBits(GPIOB, GPIO_Pin_1); GPIO_SetBits(GPIOB, GPIO_Pin_0);}	
#define 	CloseAirbag		{GPIO_SetBits(GPIOB, GPIO_Pin_1);   GPIO_ResetBits(GPIOB, GPIO_Pin_0);}
#define 	StopAirbag		{GPIO_ResetBits(GPIOB, GPIO_Pin_1); GPIO_ResetBits(GPIOB, GPIO_Pin_0);}

extern unsigned int press_time;
extern unsigned int press_HoldTime;
extern unsigned char SwitchFastFlag;

extern unsigned char motor1_topFlag;
extern unsigned char motor1_bottomFlag;

extern unsigned char motor2_topFlag;
extern unsigned char motor2_bottomFlag;


void Motor_GPIO_Init(void);
void Airbag_GPIO_Init(void);
void HandSet_Init(void);
void HandSet_Handle(void);
extern void IWDG_Feeding(void);
extern void IWDG_Configuration(void);
extern unsigned char M1M2_AD_Detect(void);


#endif
