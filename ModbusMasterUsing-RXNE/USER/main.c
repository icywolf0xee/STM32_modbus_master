
#include "stm32f10x.h"
#include "delay.h"
#include "mb_host.h"	  
 
  
 int main(void)
 {
	  
  delay_init();                                 
	mbh_init(19200,0);     
  delay_ms(1000);      //�ϵ��������룬��ʱһ��ʱ��ȥ��

  while(1)
	{ 	
		mbh_send(0x01,0x03,0x0000,0x000B); //����01�Ŵӻ�
		delay_ms(50);                      
		mbh_poll();
			
	  mbh_send(0x02,0x03,0x0000,0x000B); //����02�Ŵӻ�
		delay_ms(50);
		mbh_poll();	                                                  
	}
 }

