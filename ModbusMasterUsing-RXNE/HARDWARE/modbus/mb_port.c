/**
  ******************************************************************************
  * @file    mb_port.c
  * @author  simon 
  * @brief   modebus移植接口
  ******************************************************************************
  */

#include "mb_port.h"
#include "mb_host.h"



//char *usart2RcvBuffer;
//char USART2_RX_BUF[USART2_REC_LEN]; 

/**
 * 	@brief  MODBUS串口初始化接口
 * 	@param	baud:串口波特率
 * 	@param 	parity:奇偶校验位设置	
 * 	@return	NONE
 */
void mb_port_uartInit(uint32_t baud,uint8_t parity)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	       //PB.01是使能脚
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                     // USART2_Tx (PA.02)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               
	GPIO_Init(GPIOA, &GPIO_InitStructure);                        	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                      // USART2_Rx (PA.03)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				               //  (PB.01)使能脚
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		           // 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_1);     //主控默认发                         

	
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);	                

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);           
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);                        
	
 
	USART_Cmd(USART2, ENABLE);                                
	
}
/**
 * 	@brief  串口TX\RX使能接口
 * 	@param	txen:0-关闭tx中断	1-打开tx中断
 * 	@param 	rxen:0-关闭rx中断	1-打开rx中断	
 * 	@return	NONE
 */
void mb_port_uartEnable(uint8_t txen,uint8_t rxen)
{
	if(rxen)
	{
    USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);      //接收中断使能，对CR的 RXNEIE 位操作
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);                   //485使能
  }
	else
	{
    USART_ITConfig(USART2,USART_IT_RXNE, DISABLE);            
  }

  if(txen)
  {
		USART_ITConfig(USART2,USART_IT_TC, ENABLE);        //发送中断使能，对CR的 TCIE 位操作
		GPIO_SetBits(GPIOB,GPIO_Pin_1);                     // 
  }
	else
	{
		USART_ITConfig(USART2,USART_IT_TC, DISABLE);             
  }
}


/**
 * 	@brief  串口发送一个字节byte
 * 	@param	ch:要发送的byte	
 * 	@return	NONE
 * 	@note	 调用串口发送函数
 */
void mb_port_putchar(uint8_t ch)
{
	 USART_SendData(USART2, ch);
}


/**
 * 	@brief  串口读取一个byte
 * 	@param	ch:存放读取一个byte的指针	
 * 	@return	NONE
 * 	@note	调用串口接收函数
 */
void mb_port_getchar(uint8_t *ch)
{
	*ch = USART_ReceiveData(USART2);
}



/**
 * 	@brief  定时器初始化接口
 * 	@param	baud:串口波特率,根据波特率生成3.5T的定时
 * 	@return	NONE
 * 	 
 */
void mb_port_timerInit(uint32_t ulBaudRate)
{
	 uint32_t    usTimerT35_50us;
	 uint16_t    PrescalerValue = 0;
	
	 if( ulBaudRate > 19200 )
   {
		 usTimerT35_50us = 35;       /* 1800us. */
   }
   else
   {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
		usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
  }
	
  //定时器初始化，并装载3.5T所需要的（数值)	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  //时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	PrescalerValue = (uint16_t) (SystemCoreClock / 20000) - 1;  //  如果要得到50us的周期，即20kHz的时钟；则分频系数为3599（72M/20K -1)
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)usTimerT35_50us;   //3.5T中断，需要计数的数值
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure );
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);    
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);          
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //优先级，要低于串口1（给短信猫发信息） 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
		  
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);    //清楚溢出中断标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);   //定时器4溢出中断位关闭
  TIM_Cmd(TIM4,  DISABLE);                      //关闭定时器4
	 
}
/**
 * 	@brief  定时器使能
 * 	@return	NONE
 * 	@note	定时器要清0重新计数
 */
void mb_port_timerEnable()
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清楚中断溢出位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);   //使能中断位
  TIM_SetCounter(TIM4,0x0000);                 //清零
  TIM_Cmd(TIM4, ENABLE);                       //使能
}

/**
 * 	@brief  定时器关闭
 * 	@return	NONE
 * 	@note	定时器要清0重新计数
 */
void mb_port_timerDisable()
{
  TIM_Cmd(TIM4, DISABLE);                     //关闭定时器4
	TIM_SetCounter(TIM4,0x0000); 
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);   
}

/****
***@brief串口中断服务函数 
***/
void USART2_IRQHandler()
{
	 
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)       //接收完成中断
  {
    mbh_uartRxIsr( );
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);   		
  }
  if(USART_GetITStatus(USART2, USART_IT_TC) == SET)         //每发送完一个字节，中断一次，发送下一个
  {
    mbh_uartTxIsr( );   
    USART_ClearITPendingBit(USART2, USART_IT_TC);            
  }
}

//定时器中断服务函数
void TIM4_IRQHandler()
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);   //清楚中断溢出标志（待处理位)
	mbh_timer3T5Isr();
}

