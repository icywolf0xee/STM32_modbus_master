#ifndef __MB_PORT_H
#define __MB_PORT_H

#include "stdint.h"
#include "sys.h"
/**
  ******************************************************************************
  * @file    mb_port.h
  * @author  simon
  * @brief   modebus移植接口头文件
  ******************************************************************************
  * @note
  * 该文件为modbus移植接口的实现 
  ******************************************************************************
  */
#define ENTER_CRITICAL_SECTION( )   __set_PRIMASK(1)   //¹Ø±ÕÖÐ¶Ï£¨×ÜµÄ£©
#define EXIT_CRITICAL_SECTION( )    __set_PRIMASK(0)   //¿ªÆôÖÐ¶Ï £¨×ÜµÄ£© 
//#define USART2_REC_LEN  256

typedef enum
{
	MB_PARITY_NONE=0X00,	//无奇偶校验，两个停止位
	MB_PARITY_ODD, 			//奇校验
	MB_PARITY_EVEN			//偶校验
}mbParity;



/**
 * 	@brief  MODBUS串口初始化接口
 * 	@param	baud:串口波特率
 * 	@param 	parity:奇偶校验位设置	
 * 	@return	NONE
 * 	 
 */
void mb_port_uartInit(uint32_t baud,uint8_t parity);
/**
 * 	@brief  串口TX\RX使能接口
 * 	@param	txen:0-关闭tx中断	1-打开tx中断
 * 	@param 	rxen:0-关闭rx中断	1-打开rx中断	
 * 	@return	NONE
 * 	 
 */
void mb_port_uartEnable(uint8_t txen,uint8_t rxen);
/**
 * 	@brief  串口发送一个byte
 * 	@param	ch:要发送的byte	
 * 	@return	NONE
 *  
 */
void mb_port_putchar(uint8_t ch);
/**
 * 	@brief  串口读取一个byte
 * 	@param	ch:存放读取一个byte的指针	
 * 	@return	NONE
 * 	 
 */
void mb_port_getchar(uint8_t *ch);
/**
 * 	@brief  定时器初始化接口
 * 	@param	baud:串口波特率,根据波特率生成3.5T的定时
 * 	@return	NONE
 *  
 */
void mb_port_timerInit(uint32_t baud);
/**
 * 	@brief  定时器使能
 * 	@return	NONE
 * 	@note	定时器要清0重新计数
 */
void mb_port_timerEnable(void);
/**
 * 	@brief  定时器关闭
 * 	@return	NONE
 * 	@note	定时器要清0重新计数
 */
void mb_port_timerDisable(void);

#endif

