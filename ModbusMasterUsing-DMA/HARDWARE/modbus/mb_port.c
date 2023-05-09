/**
  ******************************************************************************
  * @file    mb_port.c
  * @author  simon 
  * @brief   modebus移植接口
  ******************************************************************************
  */

#include "mb_port.h"
#include "mb_host.h"


uint8_t  RxBuffer[BufferSize];  //usart2_rx缓存区
uint8_t  RxCounter ;
uint8_t  RxStatus ;
 
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
	DMA_InitTypeDef DMA_InitStruct;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);      // 使能DMA
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	        
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

	
	USART_Cmd(USART2, ENABLE);    //使能USART中断                            
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  //开启 USART1 总线空闲中断
	
	//中断设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);           
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);                        

	  //DMA配置
	  DMA_DeInit(DMA1_Channel6);    //通道6
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);    //外设--->内存
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)RxBuffer;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                  //内存地址自增1
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         //外设地址不变，一直为 &USART2->DR
    DMA_InitStruct.DMA_BufferSize = BufferSize;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //字节，8位总线访问
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(DMA1_Channel6, &DMA_InitStruct);

    DMA_Cmd(DMA1_Channel6, ENABLE);                              //DM1的通道6对应USART2的RX
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);              // 使能 USART1接收DMA
 
	
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



/****
***@brief串口中断服务函数 
***/
void USART2_IRQHandler()
{
	
	  uint8_t clear = clear;  // 用来消除编译器的“没有用到”的提醒

    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        clear = USART2->SR;   //这两个用来清楚IDLE标志位
        clear = USART2->DR;

        RxCounter  = BufferSize - DMA_GetCurrDataCounter(DMA1_Channel6);//缓存中的字节数
        RxStatus = 1;   //标记接收到一帧
			
        USART_ClearITPendingBit(USART2, USART_IT_IDLE); // 清除空闲中断
        DMA_Cmd(DMA1_Channel6, DISABLE);                // 停止DMA，清除DMA缓存
    }
  if(USART_GetITStatus(USART2, USART_IT_TC) == SET)         //每发送完一个字节，中断一次，发送下一个
  {
    mbh_uartTxIsr( );   
    USART_ClearITPendingBit(USART2, USART_IT_TC);            
  }
}

