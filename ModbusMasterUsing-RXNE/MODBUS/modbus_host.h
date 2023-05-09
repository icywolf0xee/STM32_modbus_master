/*
****************************************************************************
			ModBus����Э��
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
	
	uint16_t Reg01H;		/* �����������͵ļĴ����׵�ַ */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* �Ĵ������� */

	uint8_t fAck01H;		/* Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ� */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;




typedef struct
{
	/* 03H 06H ��д���ּĴ��� */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H ��д��ɢ����Ĵ��� */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	/* 04H ��ȡģ�����Ĵ��� */
	uint16_t A01;
	
	/* 01H 05H ��д����ǿ����Ȧ */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_T;





uint8_t MODH_ReadParam_01H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//����Ȧ�Ĵ���
uint8_t MODH_ReadParam_02H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//����ɢ����Ĵ���
uint8_t MODH_ReadParam_03H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//������Ĵ���
uint8_t MODH_ReadParam_04H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num);	//������Ĵ���
uint8_t MODH_WriteParam_05H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value);	//д������Ȧ�Ĵ���
uint8_t MODH_WriteParam_06H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value);	//д��������Ĵ���	
uint8_t MODH_WriteParam_10H(uint16_t slaveAddr,uint16_t _reg, uint8_t _num, const uint16_t *_buf);	//д�������Ĵ���


void MODBUS_Init(void);
void MODH_Poll(void);
int CHECKVALUE(uint16_t value);


#endif


