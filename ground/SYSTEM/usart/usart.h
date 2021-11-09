#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "string.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
enum EN_Status
{
	RX_FREE,
	RX_Length,
	RX_ID,
	RX_Data,
	RX_Tail
};

struct USART_RX_TypeDef
{
	USART_TypeDef* USARTx;             //串口号
	DMA_Stream_TypeDef* DMAy_Streamx;  //DMA数据流
	u8* pMailbox;                      //接收邮箱地址
	__IO u8* pDMAbuf;                  //DMA内存基地址
	u16 MbLen;                         //邮箱大小
	u16 DMALen;                        //DMA缓存区大小
	u16 rxConter;                      //当前接收帧结束地址+1
	u16 rxBufferPtr;                   //当前帧起始地址
	u16 rxsize;                        //当前帧大小
};

/*串口1通信缓冲长度*/
#define USART1_RXMB_LEN 250
#define USART1_TXMB_LEN 20

#define USART2_RXMB_LEN 10000
#define USART2_TXMB_LEN 128
#define UART4_RXMB_LEN 250
#define UART4_TXMB_LEN 20
#define UA1RxDMAbuf_LEN 18
#define UA2RxDMAbuf_LEN 10000
#define UA2TxDMAbuf_LEN 128
#define UA3RxDMAbuf_LEN 52
#define UA3TxDMAbuf_LEN 20
#define UA4RxDMAbuf_LEN 8
#define UA4TxDMAbuf_LEN 36
#define UA5RxDMAbuf_LEN 8
#define UA5TxDMAbuf_LEN 36
#define UA6RxDMAbuf_LEN 18
#define UA6TxDMAbuf_LEN 6

#define USART1_RX_STREAM  DMA2_Stream2  //224
#define USART1_TX_STREAM  DMA2_Stream7  //274
#define USART2_RX_STREAM  DMA1_Stream5  //154
#define USART2_TX_STREAM  DMA1_Stream6  //164
#define USART3_RX_STREAM  DMA1_Stream1  //154
#define USART3_TX_STREAM  DMA1_Stream3  //164
#define UART4_RX_STREAM   DMA1_Stream2
#define UART4_TX_STREAM   DMA1_Stream4
#define UART5_RX_STREAM   DMA1_Stream0  //104 
#define UART5_TX_STREAM   DMA1_Stream7  //174
#define USART6_RX_STREAM  DMA2_Stream1  //215
#define USART6_TX_STREAM  DMA2_Stream6  //265


void USART1_Init(u32 bound);
void USART2_Init(u32 bound);
void USART3_Init(u32 bound);
//void UART5_Init(void);
void UART4_Init(u32 bound);
void UART5_Init(u32 bound);
void USART6_Init(u32 bound);
unsigned short USART_Receive(struct USART_RX_TypeDef* USARTx);
void DataProtocol(struct USART_RX_TypeDef* USART_RX_Struct);
void USART1_DMA_printf(u8* data, u8 length);
void USART_printf(USART_TypeDef* USARTx, u8* data, u8 length);
void UART4_Configuration(void);
#endif


