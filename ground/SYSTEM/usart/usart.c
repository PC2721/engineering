#include "sys.h"
#include "usart.h"	

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{
	int handle; 
};

u8 UA1RxDMAbuf[USART1_RXMB_LEN] = {0};    //串口1接收变量
u8 UA1RxMailbox[USART1_RXMB_LEN] = {0};    
u8 UA2RxDMAbuf[USART2_RXMB_LEN] = {0};    //串口2接收变量
u8 UA2RxMailbox[USART2_RXMB_LEN] = {0};    
u8 UA3RxDMAbuf[UA3RxDMAbuf_LEN] = {0};    //串口3接收变量
u8 UA3TxDMAbuf[UA3TxDMAbuf_LEN] = {0}; 	 
u8 UA4RxDMAbuf[UA4RxDMAbuf_LEN] = {0};    //串口4接收变量
u8 UA4TxDMAbuf[UA4TxDMAbuf_LEN] = {0}; 	 
u8 UA5RxDMAbuf[UA5RxDMAbuf_LEN] = {0};    //串口4接收变量
u8 UA5TxDMAbuf[UA5TxDMAbuf_LEN] = {0}; 	 
u8 UA6RxDMAbuf[UA6RxDMAbuf_LEN] = {0};    //串口6接收变量
u8 UA6TxDMAbuf[UA6TxDMAbuf_LEN] = {0};
//u8 UA6RxDMAbuf[UA3RxDMAbuf_LEN] = {0};    //串口6接收变量
//u8 UA6TxDMAbuf[UA3TxDMAbuf_LEN] = {0};
struct USART_RX_TypeDef USART1_Rcr = {USART1, USART1_RX_STREAM, UA1RxMailbox, UA1RxDMAbuf, USART1_RXMB_LEN, UA1RxDMAbuf_LEN, 0, 0, 0};
struct USART_RX_TypeDef USART2_Rcr = {USART2, USART2_RX_STREAM, UA2RxMailbox, UA2RxDMAbuf, USART2_RXMB_LEN, UA2RxDMAbuf_LEN, 0, 0, 0};
/*串口1初始化*/
/*接收遥控器数据*/
void USART1_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
	USART_InitTypeDef USART_InitStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0}; 
	DMA_InitTypeDef DMA_InitStructure={0};
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
  
	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&(USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)UA1RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStructure.DMA_BufferSize = UA1RxDMAbuf_LEN  ;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	DMA_Cmd(DMA2_Stream2,ENABLE);
   
	DMA_DeInit(DMA2_Stream7);
     DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&(USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = NULL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7,&DMA_InitStructure);
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	DMA_Cmd(DMA2_Stream7,DISABLE);
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能DMA接收
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//使能DMA发送
}

/*串口2初始化*/
/*与裁判系统通讯*/
void USART2_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef 	DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1,ENABLE);//使能PD端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;//开漏输出
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;//IO口速度为100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;//浮空输入
	GPIO_Init(GPIOD,&GPIO_InitStructure);//根据设定参数初始化GPIOA
	
	NVIC_InitStructure.NVIC_IRQChannel 						= USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;//IRQ通道使能 
	NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化VIC寄存器

	USART_InitStructure.USART_BaudRate 				= bound;//波特率
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity 				= USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_Mode 					= USART_Mode_Tx|USART_Mode_Rx;//收发模式
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART2,&USART_InitStructure);//初始化串口
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//使能串口空闲中断
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART2,ENABLE);//使能串口
	
	DMA_DeInit(USART2_RX_STREAM);
	DMA_InitStructure.DMA_Channel				= DMA_Channel_4;//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)UA2RxDMAbuf;//将串口2接收到的数据存在ucRxData[]里
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;//DMA传输为单向
	DMA_InitStructure.DMA_BufferSize 			= UA2RxDMAbuf_LEN;//设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART2_RX_STREAM,&DMA_InitStructure);
	DMA_Cmd(USART2_RX_STREAM,ENABLE);
	
	
	//USART2_Tx
	DMA_DeInit(USART2_TX_STREAM);
	while( DMA_GetCmdStatus(USART2_TX_STREAM) == ENABLE);			//等待DMA可配置
	
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= NULL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;	//内存到外设
	DMA_InitStructure.DMA_BufferSize 			= NULL;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;					//普通模式发送
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART2_TX_STREAM, &DMA_InitStructure);
	DMA_ITConfig(USART2_TX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART2_TX_STREAM, DISABLE);	//失能DMA
	
	
}

void USART3_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 	//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);		//使能DMA1时钟
 
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8复用为USART3 TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD9复用为USART3 RX
	
	//USART3端口配置
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_9; //GPIOD8 GPIOD9
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PD8 PD9

   //USART3 初始化设置
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate 				= bound;//波特率设置
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity 				= USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode 					= USART_Mode_Tx | USART_Mode_Rx;	//接收模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_Cmd(USART3, ENABLE);  //使能串口3
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启空闲中断	
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel 						= USART3_IRQn;	//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;			//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;			//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器

	//配置USART3_RX DMA
	DMA_DeInit(USART3_RX_STREAM);
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (u32)UA3RxDMAbuf;	
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 			= UA3RxDMAbuf_LEN;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART3_RX_STREAM, &DMA_InitStructure);
	
	DMA_Cmd(USART3_RX_STREAM, ENABLE);
	
	//配置USART3_TX DMA
	DMA_DeInit(USART3_TX_STREAM);
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= NULL; 
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize 			= 0;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART3_TX_STREAM, &DMA_InitStructure);
	
	DMA_Cmd(USART3_TX_STREAM, DISABLE);
}

////串口4初始化：PC10 PC11
//void UART5_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
//	
//	//GPIO_C12
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	
//	//GPIO_D2
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOD,&GPIO_InitStructure);
//}

void UART4_Init(u32 bound)
{
		GPIO_InitTypeDef	gpio = {0};
	USART_InitTypeDef	usart = {0};
	NVIC_InitTypeDef 	nvic = {0};
	DMA_InitTypeDef		dma = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						//使能UART4时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1,ENABLE);	//使能PC端口时钟

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//Tx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);//Rx
	
    gpio.GPIO_OType = GPIO_OType_OD;//开漏
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOC, &gpio);

	nvic.NVIC_IRQChannel 					= UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority	= 0;		//抢占优先级
	nvic.NVIC_IRQChannelSubPriority			= 6;		//子优先级
	nvic.NVIC_IRQChannelCmd					= ENABLE;	//IRQ通道使能 
	NVIC_Init(&nvic);//根据指定的参数初始化NVIC寄存器

	usart.USART_BaudRate			= bound;				//波特率
	usart.USART_WordLength			= USART_WordLength_8b;	//字长为8位数据格式
	usart.USART_StopBits			= USART_StopBits_1;		//一个停止位
	usart.USART_Parity				= USART_Parity_No;		//无奇偶校验位
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	usart.USART_Mode				= USART_Mode_Tx|USART_Mode_Rx;		//收发模式
    USART_Init(UART4, &usart);//初始化串口

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	//开启空闲中断
    USART_Cmd(UART4, ENABLE);	//使能串口
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    //UART4_Tx
	DMA_DeInit(DMA1_Stream4);
	while( DMA_GetCmdStatus(DMA1_Stream4) == ENABLE );			//等待DMA可配置

	dma.DMA_Channel            =    DMA_Channel_4;                 //外设地址
	dma.DMA_PeripheralBaseAddr =    (uint32_t)&(UART4->DR);
	dma.DMA_Memory0BaseAddr    =    (uint32_t)UA4TxDMAbuf;
	dma.DMA_DIR                =    DMA_DIR_MemoryToPeripheral;    //DMA传输为单向
	dma.DMA_BufferSize         =    UA4TxDMAbuf_LEN;               //设置DMA在传输区的长度
	dma.DMA_PeripheralInc      =    DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc          =    DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize =    DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize     =    DMA_MemoryDataSize_Byte;
	dma.DMA_Mode               =    DMA_Mode_Normal;
	dma.DMA_Priority           =    DMA_Priority_High;
	dma.DMA_FIFOMode           =    DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold      =    DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst        =    DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst    =    DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &dma);
	DMA_Cmd(DMA1_Stream4, DISABLE);
    
    //UART4_Rx
	DMA_DeInit(DMA1_Stream2);
	while( DMA_GetCmdStatus(DMA1_Stream2) == ENABLE );			//等待DMA可配置

	dma.DMA_Channel            =    DMA_Channel_4;				//即DMA_Channel_5
	dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART4->DR);
	dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA4RxDMAbuf;
	dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
	dma.DMA_BufferSize         = 	UA4RxDMAbuf_LEN;
	dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
	dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
	DMA_Init(DMA1_Stream2, &dma);
	DMA_Cmd(DMA1_Stream2, ENABLE);	//使能DMA
}
//函数功能：串口5初始化
void UART5_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); 	//TX
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5); 	//RX
	
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12; 	//TX
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 	    //RX
    
	USART_DeInit(UART5);
	USART_InitStructure.USART_BaudRate				= bound;
	USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits				= USART_StopBits_1;
	USART_InitStructure.USART_Parity				= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;//发送中断
	USART_Init(UART5, &USART_InitStructure);
    
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);   //开启空闲中断
	USART_Cmd(UART5, ENABLE);                       //使能串口
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //使能串口发送DMA
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);   //使能串口接收DMA
    
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(UART5_RX_STREAM);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;//通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);//外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)UA5RxDMAbuf;//将串口4接收到的数据ucRxData_DMA1_Stream2[]里，内存基地址
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize         = UA5RxDMAbuf_LEN;//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;	//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//DMA_Mode_Normal;////设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst        = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(UART5_RX_STREAM,&DMA_InitStructure);	
    DMA_Cmd(UART5_RX_STREAM,ENABLE);	
      
    //DMA TX
    DMA_DeInit(UART5_TX_STREAM);

    DMA_InitStructure.DMA_Channel				=   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr	=	(uint32_t)&(UART5->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr		=	NULL;//暂无
    DMA_InitStructure.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	//内存到外设
    DMA_InitStructure.DMA_BufferSize			=	NULL;//暂无
    DMA_InitStructure.DMA_PeripheralInc			=	DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc				=	DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode					=	DMA_Mode_Normal;			//正常发送
    DMA_InitStructure.DMA_Priority				=	DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode				=	DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold			=	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;
    DMA_Init(UART5_TX_STREAM, &DMA_InitStructure);
    DMA_Cmd(UART5_TX_STREAM, DISABLE);	//失能DMA    
}

void USART6_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef   	DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //TX	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //RX
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   //OD
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel 						= USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate 				= bound;					//最好效果的波特率
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;
	USART_InitStructure.USART_Parity 				= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 					= USART_Mode_Rx | USART_Mode_Tx;	 
	USART_Init(USART6, &USART_InitStructure);

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);//使能串口空闲中断
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART6, ENABLE);
	
	DMA_DeInit(USART6_RX_STREAM);
	DMA_InitStructure.DMA_Channel				= DMA_Channel_5;//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)&(USART6->DR);//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)UA6RxDMAbuf;//将串口6接收到的数据UA6RxDMAbuf[]里
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;//设置数据传输方向
	DMA_InitStructure.DMA_BufferSize 			= UA6RxDMAbuf_LEN;//设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;//设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;	//设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;//DMA_Mode_Normal;//设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;//DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART6_RX_STREAM,&DMA_InitStructure);
	DMA_Cmd(USART6_RX_STREAM,ENABLE);
	
	//USART6_Tx
	DMA_DeInit(USART6_TX_STREAM);
	while( DMA_GetCmdStatus(USART6_TX_STREAM) == ENABLE);			//等待DMA可配置
	
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&(USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= NULL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;	//内存到外设
	DMA_InitStructure.DMA_BufferSize 			= NULL;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;					//普通模式发送
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART6_TX_STREAM, &DMA_InitStructure);
	
	DMA_Cmd(USART6_TX_STREAM, DISABLE);	//失能DMA
}

/*函数名：USART_Receive*/
/*输入参数：指向串口数据结构体*/
/*作用：原始数据内容，数据长度获取*/
unsigned short USART_Receive(struct USART_RX_TypeDef* USARTx)
{
	USARTx->rxConter = USARTx->DMALen - DMA_GetCurrDataCounter(USARTx->DMAy_Streamx);   //本次DMA缓冲区填充到的位置
	
	USARTx->rxBufferPtr += USARTx->rxsize;    //上次DMA缓冲区填充到的位置
	
	if(USARTx->rxBufferPtr >= USARTx->DMALen) //说明DMA缓冲区已经满了一次
	{
		USARTx->rxBufferPtr %= USARTx->DMALen;  //DMA缓冲区满了以后又填充了的位数
	}
	
	if(USARTx->rxBufferPtr < USARTx->DMALen)
	{
		USARTx->rxsize = USARTx->rxConter - USARTx->rxBufferPtr;  //计算本次接收数据的长度
		
		if(USARTx->rxsize <= USARTx->MbLen)
		{
			for(u16 i = 0; i < USARTx->rxsize; i++)
			{
				*(USARTx->pMailbox + i)= *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
			}
		}
	}
	else  //DMA缓冲区已经满了一次
	{
		USARTx->rxsize = USARTx->rxConter + USARTx->DMALen - USARTx->rxBufferPtr;   //计算本次接收数据长度，本次位置+缓存区大小-上次位置
		
		if(USARTx->rxsize <= USARTx->MbLen)   //如果数据长度小于邮箱大小
		{
			for(u16 i = 0; i < USARTx->rxsize - USARTx->rxConter; i++)
			{
				*(USARTx->pMailbox + i)= *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);  //从上次填充到的位置开始发送到邮箱
			}
			for(u16 i = 0; i < USARTx->rxConter; i++)
			{	
				*(USARTx->pMailbox + USARTx->rxsize - USARTx->rxConter + i)= *(USARTx->pDMAbuf + i);
			}
		}
	}
	return USARTx->rxsize;
}

enum EN_Status RX_Status = RX_FREE;  //状态机状态
u8 RX_Buf[USART1_RXMB_LEN] = {0}; //接收数据数组
u8 data = 0;  //状态机正在处理的数据
u16 dataLength = 0;  //数据帧长度
u8 dataID = 0;  //数据ID
u8 bufNum = 0;  //数据的位数

/*函数名：DataProtocol*/
/*输入参数：指向串口数据结构体*/
/*作用：不定长数据解析*/
void DataProtocol(struct USART_RX_TypeDef* USART_RX_Struct)
{
	for(u32 i = 0; i < USART_RX_Struct->rxsize; i++)  //循环次数和数据长度一致
	{
		data = USART_RX_Struct->pMailbox[i];
		switch(RX_Status)
		{
			case RX_FREE:
				if(data == 0x55)  //如果帧头1为0x55认为开始
				{
					bufNum = 0;
					RX_Status = RX_Length;
				}
				else
					RX_Status = RX_FREE;
					//帧头不为0x55，等待下一组数据
				break;
				
			case RX_Length:
				dataLength = data;  //读取数据长度
				RX_Status = RX_ID;  //进入ID状态
				break;
			
			case RX_ID:
				dataID = data;     //读取数据ID
				RX_Status = RX_Data;  //进入数据状态
				break;
			
			case RX_Data:
				if(dataLength > 0)
				{
					dataLength --;
					RX_Buf[bufNum++] = data;
					break;
				}
				else
				{
					RX_Status = RX_Tail;
				}
	
			case RX_Tail:
				if(data == 0xAA)
				{
					/*接收数据成功，开始处理*/
//					GPIO_ToggleBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);
				}
				else
				{
					/*数据接收失败*/
					dataLength = 0;
					dataID = 0;
					memset(RX_Buf, 0 , USART1_RXMB_LEN);
				}
				RX_Status = RX_FREE;  //准备下一帧数据的接收
				break;
				
			default:
				break;
		}
	}
}
void USART_printf(USART_TypeDef* USARTx, u8* data, u8 length)
{
	while(length--)
	{
		while((USARTx->SR & 0x40) == 0);
		USARTx->DR =(u8) *data;
		data++;
	}
}


void USART1_DMA_printf(u8* data, u8 length)
{
	while(DMA_GetCurrDataCounter(USART1_TX_STREAM)); //等待发送完毕
	
	DMA_ClearITPendingBit(USART1_TX_STREAM, DMA_IT_TCIF7);  //清除发送完成中断标志位
	DMA_Cmd(USART1_TX_STREAM,DISABLE);  //设置当前计数值先禁用DMA
	USART1_TX_STREAM->M0AR = (uint32_t)data;  //设置当前待发数据基地址：Memory0 tARget
	USART1_TX_STREAM->NDTR = (uint32_t)length;  //设置当前待发数据的数量
	DMA_Cmd(USART1_TX_STREAM, ENABLE);  //启用串口DMA发送
	
}
void USART2_DMA_printf(u8* data, u8 length)
{
	while(DMA_GetCurrDataCounter(USART2_TX_STREAM)); //等待发送完毕
	
	DMA_ClearITPendingBit(USART2_TX_STREAM, DMA_IT_TCIF7);  //清除发送完成中断标志位
	DMA_Cmd(USART2_TX_STREAM,DISABLE);  //设置当前计数值先禁用DMA
	USART2_TX_STREAM->M0AR = (uint32_t)data;  //设置当前待发数据基地址：Memory0 tARget
	USART2_TX_STREAM->NDTR = (uint32_t)length;  //设置当前待发数据的数量
	DMA_Cmd(USART2_TX_STREAM, ENABLE);  //启用串口DMA发送
}


