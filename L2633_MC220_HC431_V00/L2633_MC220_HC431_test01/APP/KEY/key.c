#include "key.h"
#include "STM32F0XX_GPIO.H"
#include "STM32F0XX_RCC.H"


unsigned char motor1_firstFlag=0;
unsigned char motor2_firstFlag=0;

unsigned char motor1_topFlag=0;
unsigned char motor1_bottomFlag=0;

unsigned char motor2_topFlag=0;
unsigned char motor2_bottomFlag=0;


unsigned int press_time=0;
unsigned int press_HoldTime=0;
unsigned char SwitchFastFlag=0;

void Motor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_12 | GPIO_Pin_10);  //m2 out, m1 out
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_3);  	//m2 in, m1 in
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	M1Close;M2Close;
}

void Airbag_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	StopAirbag;
}

void HandSet_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF,ENABLE);
	
	//L1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_PuPd=	GPIO_PuPd_NOPULL;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//L2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	//L3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//L4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//L5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	//L6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//L7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void HandSet_Handle(void)
{
	unsigned char status=0;
	
	//key 1 motor1 out
	if(KEY1 && !KEY2 && !KEY3 && !KEY4 && !KEY5 && !KEY6 && SwitchFastFlag)
	{
		motor2_firstFlag=0;
		
		if(motor1_topFlag==0){
			if(motor1_firstFlag==0){
				motor1_firstFlag=1;
				M1In;Delay_ms(40);
                motor1_bottomFlag=0;
			}else{
                status = M1M2_AD_Detect();
                if((status&0x01)==0x01){
					M1Close;M2Close;
                    motor1_topFlag=1;
                 }
            }
        }else{
			M1Close;M2Close;
			motor1_bottomFlag=0;
        }
	}
	//key 2 motor1 in
	else if(!KEY1 && KEY2 && !KEY3 && !KEY4 && !KEY5 && !KEY6 && SwitchFastFlag)
	{
		motor2_firstFlag=0;
		
		if(motor1_bottomFlag==0){
			if(motor1_firstFlag==0){
				motor1_firstFlag=1;
				M1Out;Delay_ms(40);
				motor1_topFlag=0;
			}else{
				status = M1M2_AD_Detect();
				if((status&0x01)==0x01){
					M1Close;M2Close;
					motor1_bottomFlag=1;
				}
			}
		}else{
			M1Close;M2Close;
			motor1_topFlag=0;
		}
	}
	//key 3 motor2 out
	else if(!KEY1 && !KEY2 && KEY3 && !KEY4 && !KEY5 && !KEY6 && SwitchFastFlag)
	{
		motor1_firstFlag=0;
		
		if(motor2_topFlag==0){
			if(motor2_firstFlag==0){
				motor2_firstFlag=1;
				M2Out;Delay_ms(40);
				motor2_bottomFlag=0;
			}else{
				status = M1M2_AD_Detect();
				if((status&0x02)==0x02){
					M1Close;M2Close;
					motor2_topFlag=1;
				}
			}
		}else{
			M1Close;M2Close;
			motor2_bottomFlag=0;
		}
	}
	//key 4 motor2 in
	else if(!KEY1 && !KEY2 && !KEY3 && KEY4 && !KEY5 && !KEY6 && SwitchFastFlag)
	{
		motor1_firstFlag=0;
		
		if(motor2_bottomFlag==0){
			if(motor2_firstFlag==0){
				motor2_firstFlag=1;
				M2In;Delay_ms(40);
				motor2_topFlag=0;
			}else{
				status = M1M2_AD_Detect();
				if((status&0x02)==0x02){
					M1Close;M2Close;
					motor2_bottomFlag=1;
				}
			}
		}else{
			M1Close;M2Close;
			motor2_topFlag=0;
		}
	}
	//key5 Airbag out
	else if(!KEY1 && !KEY2 && !KEY3 && !KEY4 && KEY5 && KEY6 && SwitchFastFlag)
	{
		motor1_firstFlag=0;
		motor2_firstFlag=0;
				
		OpenAirbag;
	}			
	//key6 Airbag in
	else if(!KEY1 && !KEY2 && !KEY3 && !KEY4 && !KEY5 && KEY6 && SwitchFastFlag)
	{
		motor1_firstFlag=0;
		motor2_firstFlag=0;
		
		CloseAirbag;
	}
	else 
	{
		M1Close;
		M2Close;
		StopAirbag;
		motor1_firstFlag=0;
		motor2_firstFlag=0;
	}
}


















