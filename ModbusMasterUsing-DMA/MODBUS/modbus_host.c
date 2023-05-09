/*
****************************************************************************
			ModBus主机协议
****************************************************************************
*/

#include "delay.h"

#include "modbus_host.h"

#include "bsp_user_lib.h" 

#include "modbus_timer.h"

#include "modbus_usart.h"

#define TIMEOUT		500		/* 接收命令超时时间, 单位ms */
#define NUM			1		/* 循环发送次数 */

extern MODH_T g_tModH;

uint8_t g_modh_timeout = 0;

static void MODH_RxTimeOut(void);
static void MODH_AnalyzeApp(void);

static void MODH_Read_01H(void);
static void MODH_Read_02H(void);
static void MODH_Read_03H(void);
static void MODH_Read_04H(void);
static void MODH_Read_05H(void);
static void MODH_Read_06H(void);
static void MODH_Read_10H(void);
static void MODH_ReciveNew(uint8_t _data);

VAR_T g_tVar;
//#define g_tVar g_tVar;

//需要连接的外部接口函数
extern void (*RS485_ReceiveData)(uint8_t);	//接受数据,在串口中断接受到的数传递到此函数即可
extern void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);	//发送数据

extern void bsp_StartHardTimer(uint32_t timeout, void * _pCallBack);	//开始计时3.5个字节的时间,单位为us
//以下两个函数用来测量间隔时间
extern uint32_t bsp_GetRunTime(void);	//返回当前时间,单位ms
extern uint32_t bsp_CheckRunTime(int32_t _LastTime);		//返回与之前间隔的时间

/*
*********************************************************************************************************
*	函 数 名: MODH_SendAckWithCRC
*	功能说明: 发送应答,自动加CRC.  
*	形    参: 无。发送数据在 g_tModH.TxBuf[], [g_tModH.TxCount
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_SendAckWithCRC(void)
{
	uint16_t crc;
	PDout(7) = 1;
	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
	g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxCount++] = crc;	
	RS485_SendBuf(g_tModH.TxBuf, g_tModH.TxCount);
	PDout(7) = 0;
	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_AnalyzeApp
*	功能说明: 分析应用层协议。处理应答。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_AnalyzeApp(void)
{	
	switch (g_tModH.RxBuf[1])			/* 第2个字节 功能码 */
	{
		case 0x01:	/* 读取线圈状态 */
			MODH_Read_01H();
			break;

		case 0x02:	/* 读取输入状态 */
			MODH_Read_02H();
			break;

		case 0x03:	/* 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值 */
			MODH_Read_03H();
			break;

		case 0x04:	/* 读取输入寄存器 */
			MODH_Read_04H();
			break;

		case 0x05:	/* 强制单线圈 */
			MODH_Read_05H();
			break;

		case 0x06:	/* 写单个寄存器 */
			MODH_Read_06H();
			break;		

		case 0x10:	/* 写多个寄存器 */
			MODH_Read_10H();
			break;
		
		default:
			break;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send01H
*	功能说明: 发送01H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */ //寄存器地址
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck01H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg01H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send02H
*	功能说明: 发送02H指令，读离散输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck02H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg02H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck03H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send04H
*	功能说明: 发送04H指令，读输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	g_tModH.fAck04H = 0;		/* 清接收标志 */
	g_tModH.RegNum = _num;		/* 寄存器个数 */
	g_tModH.Reg04H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send05H
*	功能说明: 发送05H指令，写强置单线圈
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;			/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */

	g_tModH.fAck05H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写1个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* 功能码 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */
	
	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
	
	g_tModH.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
*			  _buf : n个寄存器的数据。长度 = 2 * n
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, const uint16_t *_buf)
{
	uint16_t i;
	
	g_tModH.fAck10H = 0;	//将应答标志位置0
	g_tModH.TxCount = 0;
	g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* 从站地址 */	
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
	g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* 数据字节数 */
	
	for (i = 0; i < _num; i++)
	{
		if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
		{
			return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
		}
		g_tModH.TxBuf[g_tModH.TxCount++] = (uint8_t)((*_buf)>>8);		
		g_tModH.TxBuf[g_tModH.TxCount++] = (uint8_t)(*_buf);
		_buf++;
	}
	
	MODH_SendAckWithCRC();	/* 发送数据，自动加CRC */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 
*	返 回 值: 1 表示有数据
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _data)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
	uint32_t timeout;

	g_modh_timeout = 0;
	
	timeout = 35000000 / HBAUD485;		/* 计算超时时间，单位us*/
	
	/* 硬件定时中断，定时精度us 硬件定时器2用于MODBUS从机, 定时器3用于MODBUS从机主机*/
	bsp_StartHardTimer(timeout, (void *)MODH_RxTimeOut);

	if (g_tModH.RxCount < H_RX_BUF_SIZE)
	{
		g_tModH.RxBuf[g_tModH.RxCount++] = _data;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_rtu_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_RxTimeOut(void)
{
	g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
void MODH_Poll(void)
{	
	uint16_t crc1;
	//没有超过3.5个字节的时间，继续接受
	if (g_modh_timeout == 0)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1 */
	{
		/* 没有超时，继续接收。不要清零 g_tModH.RxCount */
		return ;
	}

	//接受时间超过3.5个字节，一帧数据接受结束，进行处理
	g_modh_timeout = 0;
	
	//接受字节数小于4,接受错误
	if (g_tModH.RxCount < 4)
		return;
	
	if(g_tModH.RxBuf[0] != g_tModH.slaveaddr)
		return;
	
	/* 计算CRC校验和 */
	crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount-2);
    if (((uint8_t)crc1 != g_tModH.RxBuf[g_tModH.RxCount-1])||(((uint8_t)(crc1>>8) != g_tModH.RxBuf[g_tModH.RxCount-2])))
		return;

	/* 分析应用层协议 */
	//对数据进行处理
	MODH_AnalyzeApp();

//err_ret:
	g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Read_01H
*	功能说明: 分析01H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_01H(void)
{
  uint8_t *p;  				
  if(g_tModH.RxBuf[0] == g_tModH.slaveaddr)
  {
    p = &g_tModH.RxBuf[3];					
    g_tVar.D01 = BEBufToUint16(p); p += 2;	/* 寄存器 */
    if(g_tModH.RxBuf[2]>2)
    {g_tVar.D02 = BEBufToUint16(p); p += 2;}	/* 寄存器 */	
    if(g_tModH.RxBuf[2]>4)
    {g_tVar.D03 = BEBufToUint16(p); p += 2;}	/* 寄存器 */	
    if(g_tModH.RxBuf[2]>6)
    {g_tVar.D04 = BEBufToUint16(p); p += 2;}	/* 寄存器 */                                     
    g_tModH.fAck01H = 1;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_02H
*	功能说明: 分析02H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_02H(void)
{
  uint8_t *p;
  p = &g_tModH.RxBuf[3];	
  g_tVar.T01 = BEBufToUint16(p); p += 2;	        /* 寄存器 */	
  if(g_tModH.RxBuf[2]>2)
  {g_tVar.T02 = BEBufToUint16(p); p += 2;}	/* 寄存器 */	
  if(g_tModH.RxBuf[2]>4)
  {g_tVar.T03 = BEBufToUint16(p); p += 2;}	/* 寄存器 */		
  g_tModH.fAck02H = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_04H
*	功能说明: 分析04H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_04H(void)
{
 uint8_t bytes;
	uint8_t *p;
	
	//if (g_tModH.RxCount > 0)
	
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */				
	
				if (bytes == 2)
				{
					p = &g_tModH.RxBuf[3];	
					
					g_tVar.A01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
					
					g_tModH.fAck04H = 1;
				}
		
	
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_05H
*	功能说明: 分析05H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_05H(void)
{
  g_tModH.fAck05H = 1;		/* 接收到应答 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_06H
*	功能说明: 分析06H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_06H(void)
{
  g_tModH.fAck06H = 1;		/* 接收到应答 */
}


/*
*********************************************************************************************************
*	函 数 名: MODH_Read_03H
*	功能说明: 分析03H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_03H(void)
{
  uint8_t *p;
  p = &g_tModH.RxBuf[3];	
  g_tVar.P01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
  if(g_tModH.RxBuf[2]>2)
  {g_tVar.P02 = BEBufToUint16(p); p += 2;}	/* 寄存器 */	
  g_tModH.fAck03H = 1;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Read_10H
*	功能说明: 分析10H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Read_10H(void)
{
  g_tModH.fAck10H = 1;		/* 接收到应答 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_01H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{
		MODH_Send01H (slaveAddr, _reg, _num);		  /* 发送命令 */
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)				/* 等待应答,超时或接收到应答则break  */
		{
		   	MODH_Poll();

			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck01H > 0)
			{
				break;		/* 接收到应答 */
			}
		}
		
		if (g_tModH.fAck01H > 0)
		{
			break;			/* 循环NUM次，如果接收到命令则break循环 */
		}
	}
	
	if (g_tModH.fAck01H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 01H 读成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_02H
*	功能说明: 单个参数. 通过发送02H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_02H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{
		MODH_Send02H (slaveAddr, _reg, _num);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			MODH_Poll();

			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck02H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck02H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck02H == 0)
	{
		return 0;
	}
	else 
	{
		return 1;	/* 02H 读成功 */
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_03H
*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_03H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{
		MODH_Send03H (slaveAddr, _reg, _num);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
	
		while (1)
		{
			MODH_Poll();

			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck03H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck03H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck03H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 写入03H参数成功 */
	}
}


/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_04H
*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_04H(uint16_t slaveAddr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{
		MODH_Send04H (slaveAddr, _reg, _num);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			MODH_Poll();

			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;		/* 通信超时了 */
			}
			
			if (g_tModH.fAck04H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck04H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck04H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else 
	{
		return 1;	/* 04H 读成功 */
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_05H
*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_05H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value)
{
	int32_t time1;
	uint8_t i;

	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{
		MODH_Send05H (slaveAddr, _reg, _value);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		
		while (1)
		{
			MODH_Poll();
			
			/* 超时大于 TIMEOUT，则认为异常 */
			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;	/* 通信超时了 */
			}
			
			if (g_tModH.fAck05H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck05H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck05H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 05H 写成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_06H
*	功能说明: 单个参数. 通过发送06H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_06H(uint16_t slaveAddr,uint16_t _reg, uint16_t _value)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{	
		MODH_Send06H (slaveAddr, _reg, _value);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
				
		while (1)	//若等待100ms没有回应,则退出,通讯超时
		{
			
			MODH_Poll();
		
			if (bsp_CheckRunTime(time1) > TIMEOUT)		//等待时间
			{
				break;
			}
			
			if (g_tModH.fAck06H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck06H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck06H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入06H参数成功 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_WriteParam_10H
*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_WriteParam_10H(uint16_t slaveAddr,uint16_t _reg, uint8_t _num, const uint16_t *_buf)
{
	int32_t time1;
	uint8_t i;
	
	g_tModH.slaveaddr = slaveAddr;
	for (i = 0; i < NUM; i++)
	{	
		MODH_Send10H(slaveAddr, _reg, _num, _buf);
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
				
		while (1)
		{
			MODH_Poll();
		
			if (bsp_CheckRunTime(time1) > TIMEOUT)		
			{
				break;
			}
			
			if (g_tModH.fAck10H > 0)
			{
				break;
			}
		}
		
		if (g_tModH.fAck10H > 0)
		{
			break;
		}
	}
	
	if (g_tModH.fAck10H == 0)
	{
		return 0;	/* 通信超时了 */
	}
	else
	{
		return 1;	/* 写入10H参数成功 */
	}
}

void MODBUS_Init(void)
{
	RS485_ReceiveData = MODH_ReciveNew;
	modbus_timer2_Init();
	modbus_timer3_Init();
	modbus_usart3_Init(9600);
}

/*
*********************************************************************************************************
*	函 数 名: CHECKVALUE()
*	功能说明: 检查从机返回的数据值与指定值是否相等
*	形    参: unsign value
*	返 回 值: 相等返回1   不相等 返回0     
*********************************************************************************************************
*/
int CHECKVALUE(uint16_t value)
{
	uint16_t temp = g_tModH.RxBuf[3]<<8|g_tModH.RxBuf[4];
	if (temp == value ){
		return 1;
	}
	else 
	{
		return 0;
	}

}







