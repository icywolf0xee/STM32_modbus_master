/**
  ******************************************************************************
  * @file    mb_hook.c
  * @author  Derrick Wang
  * @brief   modebus回调函数接口
  ******************************************************************************
  * @note	
  * 针对modbus的回调处理 	
  ******************************************************************************
  */
	


#include "mb_hook.h"
#include "mb_port.h"

uint16_t mlx90640data[11];   //采集了11个数据
uint8_t  slaveaddr;

void mbh_hook_rec01(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
void mbh_hook_rec02(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
//将mlx90640的数据整合并发送给 短信猫
//addr从机地址；03功能码； 0x16数据长度; 第4个开始才是真正的数据；
void mbh_hook_rec03(uint8_t add, uint8_t *data,uint8_t datalen)
{
	int index = 0;
	uint8_t  mlx_len = datalen - 2;   //减掉2个crc
	slaveaddr = add;                  //第一个从机地址
	
	while( mlx_len > 0)
	{
		mlx90640data[index] = *data++ << 8;
		mlx90640data[index] |= *data++; 
		mlx_len-- ;
		index++ ;
	}
}

void mbh_hook_rec04(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
void mbh_hook_rec05(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
void mbh_hook_rec06(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
void mbh_hook_rec15(uint8_t add,uint8_t *data,uint8_t datalen)
{

}
void mbh_hook_rec16(uint8_t add,uint8_t *data,uint8_t datalen)
{

}


void mbh_hook_timesErr(uint8_t add,uint8_t cmd)
{
	
}

