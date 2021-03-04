#include "system.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_exti.h"
#include "STM32F0XX_MISC.H"
#include "STM32F0XX_RCC.H"
#include "STM32F0XX_GPIO.H"
#include "STM32F0XX_USART.H"
#include "stm32f0xx_flash.h"
#include "STM32F0XX_DMA.H"
#include "STM32F0XX_ADC.H"
#include "codetable.h"
#include "key.h"


#define  MaxCurrent_ADC			220000
#define  MinCurrent_ADC			2000

#define   	ADC1_DR_Address  	0x40012440
#define 	ADC_CHANNEL_NUM		2

uint32_t M1_AD=0;
uint32_t M2_AD=0;
uint32_t tmpM1_ad=0;
uint32_t tmpM2_ad=0;
uint8_t M1NeedToStopFlag=0;
uint8_t M2NeedToStopFlag=0;
uint16_t ReciveceDatecount=0;
uint32_t ADC_ConvertedValueLocal1=0;    
uint32_t ADC_ConvertedValueLocal2=0; 
__IO uint16_t RegularConvData_Tab[ADC_CHANNEL_NUM]={0}; 

//timer 10ms
void TIM16_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TIM_DeInit(TIM16);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 10000;	//10ms
	TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM16,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void TIM16_IRQHandler(void) //10ms
{    		
	TIM_ClearITPendingBit(TIM16 , TIM_FLAG_Update);
	
	if( KEY1 && !KEY2 && !KEY3 && !KEY4 && !KEY5 && !KEY6){
		press_time++;
		press_HoldTime++;
	}else if( !KEY1 && KEY2 && !KEY3 && !KEY4 && !KEY5 && !KEY6){
		press_time++;
		press_HoldTime++;
	}else if( !KEY1 && !KEY2 && KEY3 && !KEY4 && !KEY5 && !KEY6){
		press_time++;
		press_HoldTime++;
	}else if( !KEY1 && !KEY2 && !KEY3 && KEY4 && !KEY5 && !KEY6){
		press_time++;
		press_HoldTime++;
	}else if( !KEY1 && !KEY2 && !KEY3 && !KEY4 && KEY5 && !KEY6){
		press_time++;
		press_HoldTime++;
	}else if( !KEY1 && !KEY2 && !KEY3 && !KEY4 && !KEY5 && KEY6){
		press_time++;
		press_HoldTime++;
	}
	else{
		press_time=0;
		SwitchFastFlag=0;
		press_HoldTime=0;
	}
	
	if(press_time>=35){
		press_time=0;
		SwitchFastFlag=1;
	}
}


 void ADC1_DMA_Init(void)
{
    ADC_InitTypeDef     ADC_InitStruct;
	DMA_InitTypeDef     DMA_InitStruct;
	GPIO_InitTypeDef    GPIO_InitStruct;
	NVIC_InitTypeDef    NVIC_InitStruct;

	/* ADC1 DeInit */
	ADC_DeInit(ADC1);        

	/* Enable  GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE); 

	/* Configure gpio  as analog input */
	GPIO_InitStruct.GPIO_Pin =  (GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;	//GPIO_PuPd_NOPULL  GPIO_PuPd_UP  GPIO_PuPd_DOWN
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&RegularConvData_Tab;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                                                           
	DMA_InitStruct.DMA_BufferSize = ADC_CHANNEL_NUM;                                                                                                                 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                                 
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                                                                     
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;                                                       
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE); 
	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);                                                        
	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStruct);

	 /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
	ADC_InitStruct.ADC_Resolution  = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ContinuousConvMode  = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign  = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ScanDirection  = ADC_ScanDirection_Upward;//ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStruct);  

	/* ADC Channel Config */
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_55_5Cycles);

	/* ADC Calibrate */
	ADC_GetCalibrationFactor(ADC1); 
	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);     
	/* Wait the ADCEN flag */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

	/*Enables the specified DMAy Channelx interrupts */
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);       
	//DMA NVIC
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/*ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(ADC1);
}

void DMA1_Channel1_IRQHandler(void)
{				
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1) != RESET)
	{   
		DMA_ClearFlag(DMA1_FLAG_TC1);	
		if(ReciveceDatecount<200){	
			M1_AD=M1_AD + (u32)(RegularConvData_Tab[0]);
			M2_AD=M2_AD + (u32)(RegularConvData_Tab[1]);
			ReciveceDatecount++;
		}else{
			ReciveceDatecount=0;
			ADC_ConvertedValueLocal1=M1_AD;
			ADC_ConvertedValueLocal2=M2_AD;
			M1_AD=0;
			M2_AD=0;
		}
	
	}												
}


/********************************************************************
**		ADC Detect Handler
**		
**		
*******************************************************************/
unsigned char M1M2_AD_Detect(void)
{			
	unsigned char res=0;
	
	tmpM1_ad=ADC_ConvertedValueLocal1;
	tmpM2_ad=ADC_ConvertedValueLocal2;
	
	//motor1
	if(tmpM1_ad < MinCurrent_ADC){
		M1NeedToStopFlag=1;
	}	
	else{
		M1NeedToStopFlag=0;	
	}
	
	//motor2
	if(tmpM2_ad < MinCurrent_ADC){
		M2NeedToStopFlag=2;
	}	
	else{
		M2NeedToStopFlag=0;	
	}
										
	res = (M1NeedToStopFlag | M2NeedToStopFlag);	
	
	return res;
}

int main(void)
{
    SystemInit();	// cpu clk=24mhz£¬pll clk
	Delay_ms(10);
	Motor_GPIO_Init();
	Airbag_GPIO_Init();
	HandSet_Init();
	ADC1_DMA_Init();
	TIM16_Configuration();		
	IWDG_Configuration();

	while(1)
	{
		IWDG_Feeding();
		HandSet_Handle();
		IWDG_Feeding();
	}
}
