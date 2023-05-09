
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
  delay_ms(1000);      //上单后有乱码，延时一段时间去除

  while(1)
	{ 	
		mbh_send(0x01,0x03,0x0000,0x000B); //访问01号从机
		delay_ms(50); 
		if(RxStatus == 1)
		{
			mbh_poll();
			RxStatus = 0;
			memset(RxBuffer, 0, i);    //清楚缓存
      RxCounter = 0;
      DMA_SetCurrDataCounter(DMA1_Channel6, BufferSize);
      DMA_Cmd(DMA1_Channel6, ENABLE); 
		}	
		                     
		
	  mbh_send(0x02,0x03,0x0000,0x000B); //访问02号从机
		delay_ms(50);
		if(RxStatus == 1)
		{
			mbh_poll();
			RxStatus = 0;
			memset(RxBuffer, 0, i);    //清楚缓存
      RxCounter = 0;
      DMA_SetCurrDataCounter(DMA1_Channel6, BufferSize);
      DMA_Cmd(DMA1_Channel6, ENABLE); 
		}		                                                  
	}
 }

