#include "delay.h"
	    								   
void delay_us(u32 nus)
{		
	uint32_t us_tick=SystemCoreClock / 1000000UL;
	uint32_t start,now,JianGe,reload;
	start=SysTick->VAL;              //把最开始获得的计数值作为基准值
	reload = SysTick->LOAD;
	do{
	 now=SysTick->VAL;				//获取当前值
		JianGe = start > now ? start - now : reload + start - now;
	}while(JianGe<nus*us_tick);			 //判断是否到达间隔

}

void delay_ms(u16 nms)
{	 		  	  
	HAL_Delay(nms);	
} 

