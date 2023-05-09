#ifndef	_BSP_USER_LIB_H
#define	_BSP_USER_LIB_H
#include "stm32f10x.h"



uint16_t BEBufToUint16(uint8_t *_pBuf);

uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen);



#endif


