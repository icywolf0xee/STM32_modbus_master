
#include "stm32f10x.h"
#include "delay.h"
#include "mb_host.h"	  
 
  
 int main(void)
 {
	  
  delay_init();                                 
	mbh_init(19200,0);     
  delay_ms(1000);      //上单后有乱码，延时一段时间去除

  while(1)
	{ 	
		mbh_send(0x01,0x03,0x0000,0x000B); //访问01号从机
		delay_ms(50);                      
		mbh_poll();
			
	  mbh_send(0x02,0x03,0x0000,0x000B); //访问02号从机
		delay_ms(50);
		mbh_poll();	                                                  
	}
 }

