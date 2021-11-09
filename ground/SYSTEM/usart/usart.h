#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "string.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
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
	USART_TypeDef* USARTx;             //���ں�
	DMA_Stream_TypeDef* DMAy_Streamx;  //DMA������
	u8* pMailbox;                      //���������ַ
	__IO u8* pDMAbuf;                  //DMA�ڴ����ַ
	u16 MbLen;                         //�����С
	u16 DMALen;                        //DMA��������С
	u16 rxConter;                      //��ǰ����֡������ַ+1
	u16 rxBufferPtr;                   //��ǰ֡��ʼ��ַ
	u16 rxsize;                        //��ǰ֡��С
};

/*����1ͨ�Ż��峤��*/
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


