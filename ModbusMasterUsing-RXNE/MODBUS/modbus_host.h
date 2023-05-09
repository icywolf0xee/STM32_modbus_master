/*
****************************************************************************
			ModBus主机协议
****************************************************************************
*/
#ifndef __MOSBUS_HOST_H
#define __MOSBUS_HOST_H
#include "stm32f10x.h"

#define HBAUD485		19200

#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128

#define REG_T01		0x0201
typedef struct
{
	uint16_t slaveaddr;
	uint8_t RxBuf[H_RX_BUF_SIZE];
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;

	uint8_t RspCode;

	uint8_t TxBuf[H_TX_BUF_SIZE];
	uint8_t TxCount;
	
	uint16_t Reg01H;		/* 保存主机发送的寄存器首地址 */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* 寄存器个数 */

	uint8_t fAck01H;		/* 应答命令标志 0 表示执行失败 1表示执行成功 */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;




typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H 读写离散输入寄存器 */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	/* 04H 读取模拟量寄存器 */
	uint16_t A01;
	
	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_T;





uint8_t MODH_ReadParam_01H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//读线圈寄存器
uint8_t MODH_ReadParam_02H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//读离散输入寄存器
uint8_t MODH_ReadParam_03H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//读保存寄存器
uint8_t MODH_ReadParam_04H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//读输入寄存器
uint8_t MODH_WriteParam_05H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value);	//写单个线圈寄存器
uint8_t MODH_WriteParam_06H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value);	//写单个保存寄存器	
uint8_t MODH_WriteParam_10H(uint16_t slaveAddr,uint16_t _reg, uint8_t _num, const uint16_t *_buf);	//写多个保存寄存器


void MODBUS_Init(void);
void MODH_Poll(void);
int CHECKVALUE(uint16_t value);


#endif


