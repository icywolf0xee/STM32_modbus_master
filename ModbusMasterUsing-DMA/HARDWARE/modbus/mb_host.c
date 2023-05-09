/**
  ******************************************************************************
  * @file    mb_host.c
  * @author  simon
  * @brief   modebus主机实现代码
  ******************************************************************************
  * @note
  * 该文件无需修改
  ******************************************************************************
  */
#include "string.h"
#include "mb_host.h"
#include "mb_port.h"
#include "mb_crc.h"
#include "mb_hook.h"


extern uint8_t  RxBuffer[BufferSize];   
extern uint8_t  RxCounter ;
extern uint8_t  RxStatus ;

struct
{
	uint8_t state;						  //modbus host状态
	uint8_t errTimes;  					//失败次数计数
	uint8_t txLen;     					//需要发送的帧长度
	uint8_t txCounter;					//已发送bytes计数
	uint8_t txBuf[MBH_RTU_MAX_SIZE];	//发送缓冲区
	uint16_t rxCounter;					//接收计数
	uint8_t rxBuf[MBH_RTU_RC_MAX_SIZE];	//接收缓冲区
	uint8_t rxTimeOut;					//接收时的超时计数
	
}mbHost;



//modbus初始化
void mbh_init(uint32_t baud,uint8_t parity)
{
	mb_port_uartEnable(0,0);
	mb_port_uartInit(baud,parity);
}
uint8_t mbh_getState()
{
	return mbHost.state;
}

//发送一帧命令
int8_t mbh_send(uint8_t add,uint8_t cmd,uint16_t startAddr,uint16_t read_data_len)
{
	uint16_t crc;
	 
  switch(cmd)
	{
		case 01:
		break;
		
		case 03:
				mbHost.txCounter=0;
				mbHost.rxCounter=0;	
				mbHost.txBuf[0]=add;
				mbHost.txBuf[1]=cmd;
				mbHost.txBuf[2]=(uint8_t)(startAddr>>0x08);  //高8位在前
				mbHost.txBuf[3]=(uint8_t)(startAddr&0xff);
				mbHost.txBuf[4]=(uint8_t)(read_data_len>>0x08);  //高8位在前
				mbHost.txBuf[5]=(uint8_t)(read_data_len&0xff);
				crc = mb_crc16(mbHost.txBuf,6);
				mbHost.txBuf[6]=(uint8_t)(crc&0xff);
				mbHost.txBuf[7]=(uint8_t)(crc>>8);
				mbHost.txLen = 8;
		
			 	mb_port_putchar(mbHost.txBuf[0]);    //将第一个字节放进USART->DR中
		    mbHost.txCounter++;
		
				mbHost.state = MBH_STATE_TX;
				mb_port_uartEnable(1,0);  //enable tx,disable rx
		break;
		
		case 04:
		
		break;
	}
	return 0;
}
//接收正确,进行解析处理
void mbh_exec(uint8_t *pframe,uint8_t len)
{
	
	
	switch(pframe[1])//cmd
	{
		case 1:
		//	mbh_hook_rec01(pframe[0],(pframe+2),len);
			break;
		case 2:
		//	mbh_hook_rec02(pframe[0],(pframe+2),len);
			break;
		
		case 3: 
			
			mbh_hook_rec03(pframe[0],(pframe+3),len);  //地址和温度数据进行处理	
		
			break;
		
		
		case 4:
		//	mbh_hook_rec04(pframe[0],(pframe+2),len);
			break;
		case 5:
			//mbh_hook_rec05(pframe[0],(pframe+2),len);
			break;
		case 6:
		//	mbh_hook_rec06(pframe[0],(pframe+2),len);
			break;
		case 15:
		//	mbh_hook_rec15(pframe[0],(pframe+2),len);
			break;
		case 16:
		//	mbh_hook_rec16(pframe[0],(pframe+2),len);
			break;
	}
}

//对接收的数据进行校验和判断
void mbh_poll()
{
	  uint16_t  crc;
   
    crc = mb_crc16(RxBuffer,RxCounter);
		if(crc ==0) 	//接收的一帧数据正确
		{
			if((RxBuffer[0]==mbHost.txBuf[0])&&(RxBuffer[1]==mbHost.txBuf[1]))			//发送帧数据和接收到的帧数据地址和功能码一样
			{
					mbh_exec(RxBuffer,RxCounter);	
			
			}	
		}	
}



void mbh_uartTxIsr()
{
	switch (mbHost.state)
	{
		case MBH_STATE_TX:
			if(mbHost.txCounter==mbHost.txLen) //全部发送完
			{
				mbHost.state=MBH_STATE_TX_END;
				mb_port_uartEnable(0,1);  //disable tx,enable rx
				mbHost.rxTimeOut=0;		  //清除接收超时计数
			}
			else
			{
				mb_port_putchar(mbHost.txBuf[mbHost.txCounter++]);
			}
			break;
		case MBH_STATE_TX_END:
			mb_port_uartEnable(0,1);  	  //disable tx,enable rx
			break;
	}	
}



