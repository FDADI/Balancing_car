#include "delay.h"
	    								   
void delay_us(u32 nus)
{		
	uint32_t us_tick=SystemCoreClock / 1000000UL;
	uint32_t start,now,JianGe,reload;
	start=SysTick->VAL;              //���ʼ��õļ���ֵ��Ϊ��׼ֵ
	reload = SysTick->LOAD;
	do{
	 now=SysTick->VAL;				//��ȡ��ǰֵ
		JianGe = start > now ? start - now : reload + start - now;
	}while(JianGe<nus*us_tick);			 //�ж��Ƿ񵽴���

}

void delay_ms(u16 nms)
{	 		  	  
	HAL_Delay(nms);	
} 

