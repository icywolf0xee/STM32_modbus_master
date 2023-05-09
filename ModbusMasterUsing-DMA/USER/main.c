
#include "stm32f10x.h"
#include "delay.h"
#include "mb_host.h"	  
#include "mb_port.h" 
#include "string.h" 


extern uint8_t  RxBuffer[BufferSize];   
extern uint8_t  RxCounter ;
extern uint8_t  RxStatus ;
  
 int main(void)
 {
	uint8_t   i = 0;
  delay_init();                                 
	mbh_init(19200,0);     
  delay_ms(1000);      //�ϵ��������룬��ʱһ��ʱ��ȥ��

  while(1)
	{ 	
		mbh_send(0x01,0x03,0x0000,0x000B); //����01�Ŵӻ�
		delay_ms(50); 
		if(RxStatus == 1)
		{
			mbh_poll();
			RxStatus = 0;
			memset(RxBuffer, 0, i);    //�������
      RxCounter = 0;
      DMA_SetCurrDataCounter(DMA1_Channel6, BufferSize);
      DMA_Cmd(DMA1_Channel6, ENABLE); 
		}	
		                     
		
	  mbh_send(0x02,0x03,0x0000,0x000B); //����02�Ŵӻ�
		delay_ms(50);
		if(RxStatus == 1)
		{
			mbh_poll();
			RxStatus = 0;
			memset(RxBuffer, 0, i);    //�������
      RxCounter = 0;
      DMA_SetCurrDataCounter(DMA1_Channel6, BufferSize);
      DMA_Cmd(DMA1_Channel6, ENABLE); 
		}		                                                  
	}
 }

