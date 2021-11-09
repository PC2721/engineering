#include "sys.h"
#include "usart.h"	

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{
	int handle; 
};

u8 UA1RxDMAbuf[USART1_RXMB_LEN] = {0};    //����1���ձ���
u8 UA1RxMailbox[USART1_RXMB_LEN] = {0};    
u8 UA2RxDMAbuf[USART2_RXMB_LEN] = {0};    //����2���ձ���
u8 UA2RxMailbox[USART2_RXMB_LEN] = {0};    
u8 UA3RxDMAbuf[UA3RxDMAbuf_LEN] = {0};    //����3���ձ���
u8 UA3TxDMAbuf[UA3TxDMAbuf_LEN] = {0}; 	 
u8 UA4RxDMAbuf[UA4RxDMAbuf_LEN] = {0};    //����4���ձ���
u8 UA4TxDMAbuf[UA4TxDMAbuf_LEN] = {0}; 	 
u8 UA5RxDMAbuf[UA5RxDMAbuf_LEN] = {0};    //����4���ձ���
u8 UA5TxDMAbuf[UA5TxDMAbuf_LEN] = {0}; 	 
u8 UA6RxDMAbuf[UA6RxDMAbuf_LEN] = {0};    //����6���ձ���
u8 UA6TxDMAbuf[UA6TxDMAbuf_LEN] = {0};
//u8 UA6RxDMAbuf[UA3RxDMAbuf_LEN] = {0};    //����6���ձ���
//u8 UA6TxDMAbuf[UA3TxDMAbuf_LEN] = {0};
struct USART_RX_TypeDef USART1_Rcr = {USART1, USART1_RX_STREAM, UA1RxMailbox, UA1RxDMAbuf, USART1_RXMB_LEN, UA1RxDMAbuf_LEN, 0, 0, 0};
struct USART_RX_TypeDef USART2_Rcr = {USART2, USART2_RX_STREAM, UA2RxMailbox, UA2RxDMAbuf, USART2_RXMB_LEN, UA2RxDMAbuf_LEN, 0, 0, 0};
/*����1��ʼ��*/
/*����ң��������*/
void USART1_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
	USART_InitTypeDef USART_InitStructure={0};
	NVIC_InitTypeDef NVIC_InitStructure={0}; 
	DMA_InitTypeDef DMA_InitStructure={0};
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
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
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ��DMA����
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//ʹ��DMA����
}

/*����2��ʼ��*/
/*�����ϵͳͨѶ*/
void USART2_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef 	DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1,ENABLE);//ʹ��PD�˿�ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;//����ģʽ
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;//��©���
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;//IO���ٶ�Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;//��������
	GPIO_Init(GPIOD,&GPIO_InitStructure);//�����趨������ʼ��GPIOA
	
	NVIC_InitStructure.NVIC_IRQChannel 						= USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;//IRQͨ��ʹ�� 
	NVIC_Init(&NVIC_InitStructure);//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	USART_InitStructure.USART_BaudRate 				= bound;//������
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity 				= USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_Mode 					= USART_Mode_Tx|USART_Mode_Rx;//�շ�ģʽ
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART2,&USART_InitStructure);//��ʼ������
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//ʹ�ܴ��ڿ����ж�
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART2,ENABLE);//ʹ�ܴ���
	
	DMA_DeInit(USART2_RX_STREAM);
	DMA_InitStructure.DMA_Channel				= DMA_Channel_4;//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);//�ڴ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)UA2RxDMAbuf;//������2���յ������ݴ���ucRxData[]��
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;//DMA����Ϊ����
	DMA_InitStructure.DMA_BufferSize 			= UA2RxDMAbuf_LEN;//����DMA�ڴ������ĳ���
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
	while( DMA_GetCmdStatus(USART2_TX_STREAM) == ENABLE);			//�ȴ�DMA������
	
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= NULL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
	DMA_InitStructure.DMA_BufferSize 			= NULL;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;					//��ͨģʽ����
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART2_TX_STREAM, &DMA_InitStructure);
	DMA_ITConfig(USART2_TX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART2_TX_STREAM, DISABLE);	//ʧ��DMA
	
	
}

void USART3_Init(u32 bound)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	DMA_InitTypeDef		DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 	//ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ��USART3ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);		//ʹ��DMA1ʱ��
 
	
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8����ΪUSART3 TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD9����ΪUSART3 RX
	
	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_8 | GPIO_Pin_9; //GPIOD8 GPIOD9
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PD8 PD9

   //USART3 ��ʼ������
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate 				= bound;//����������
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity 				= USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode 					= USART_Mode_Tx | USART_Mode_Rx;	//����ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//���������ж�	
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

	//Usart3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel 						= USART3_IRQn;	//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;			//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;			//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	//����USART3_RX DMA
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
	
	//����USART3_TX DMA
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

////����4��ʼ����PC10 PC11
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						//ʹ��UART4ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1,ENABLE);	//ʹ��PC�˿�ʱ��

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//Tx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);//Rx
	
    gpio.GPIO_OType = GPIO_OType_OD;//��©
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOC, &gpio);

	nvic.NVIC_IRQChannel 					= UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority	= 0;		//��ռ���ȼ�
	nvic.NVIC_IRQChannelSubPriority			= 6;		//�����ȼ�
	nvic.NVIC_IRQChannelCmd					= ENABLE;	//IRQͨ��ʹ�� 
	NVIC_Init(&nvic);//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	usart.USART_BaudRate			= bound;				//������
	usart.USART_WordLength			= USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
	usart.USART_StopBits			= USART_StopBits_1;		//һ��ֹͣλ
	usart.USART_Parity				= USART_Parity_No;		//����żУ��λ
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
	usart.USART_Mode				= USART_Mode_Tx|USART_Mode_Rx;		//�շ�ģʽ
    USART_Init(UART4, &usart);//��ʼ������

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	//���������ж�
    USART_Cmd(UART4, ENABLE);	//ʹ�ܴ���
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    //UART4_Tx
	DMA_DeInit(DMA1_Stream4);
	while( DMA_GetCmdStatus(DMA1_Stream4) == ENABLE );			//�ȴ�DMA������

	dma.DMA_Channel            =    DMA_Channel_4;                 //�����ַ
	dma.DMA_PeripheralBaseAddr =    (uint32_t)&(UART4->DR);
	dma.DMA_Memory0BaseAddr    =    (uint32_t)UA4TxDMAbuf;
	dma.DMA_DIR                =    DMA_DIR_MemoryToPeripheral;    //DMA����Ϊ����
	dma.DMA_BufferSize         =    UA4TxDMAbuf_LEN;               //����DMA�ڴ������ĳ���
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
	while( DMA_GetCmdStatus(DMA1_Stream2) == ENABLE );			//�ȴ�DMA������

	dma.DMA_Channel            =    DMA_Channel_4;				//��DMA_Channel_5
	dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART4->DR);
	dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA4RxDMAbuf;
	dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//���赽�ڴ�
	dma.DMA_BufferSize         = 	UA4RxDMAbuf_LEN;
	dma.DMA_Mode               = 	DMA_Mode_Circular;			//ѭ������
	dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
	DMA_Init(DMA1_Stream2, &dma);
	DMA_Cmd(DMA1_Stream2, ENABLE);	//ʹ��DMA
}
//�������ܣ�����5��ʼ��
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
	USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;//�����ж�
	USART_Init(UART5, &USART_InitStructure);
    
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);   //���������ж�
	USART_Cmd(UART5, ENABLE);                       //ʹ�ܴ���
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //ʹ�ܴ��ڷ���DMA
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);   //ʹ�ܴ��ڽ���DMA
    
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(UART5_RX_STREAM);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;//ͨ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);//�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)UA5RxDMAbuf;//������4���յ�������ucRxData_DMA1_Stream2[]��ڴ����ַ
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;//�������ݴ��䷽��
	DMA_InitStructure.DMA_BufferSize         = UA5RxDMAbuf_LEN;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;//DMA_Mode_Normal;////����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
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
    DMA_InitStructure.DMA_Memory0BaseAddr		=	NULL;//����
    DMA_InitStructure.DMA_DIR					=	DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
    DMA_InitStructure.DMA_BufferSize			=	NULL;//����
    DMA_InitStructure.DMA_PeripheralInc			=	DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc				=	DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode					=	DMA_Mode_Normal;			//��������
    DMA_InitStructure.DMA_Priority				=	DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode				=	DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold			=	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst			=	DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst		=	DMA_PeripheralBurst_Single;
    DMA_Init(UART5_TX_STREAM, &DMA_InitStructure);
    DMA_Cmd(UART5_TX_STREAM, DISABLE);	//ʧ��DMA    
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
	USART_InitStructure.USART_BaudRate 				= bound;					//���Ч���Ĳ�����
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;
	USART_InitStructure.USART_Parity 				= USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 					= USART_Mode_Rx | USART_Mode_Tx;	 
	USART_Init(USART6, &USART_InitStructure);

	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);//ʹ�ܴ��ڿ����ж�
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART6, ENABLE);
	
	DMA_DeInit(USART6_RX_STREAM);
	DMA_InitStructure.DMA_Channel				= DMA_Channel_5;//�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)&(USART6->DR);//�ڴ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)UA6RxDMAbuf;//������6���յ�������UA6RxDMAbuf[]��
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;//�������ݴ��䷽��
	DMA_InitStructure.DMA_BufferSize 			= UA6RxDMAbuf_LEN;//����DMAһ�δ����������Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;//���������ַ����
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;//������������ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;//�����ڴ�����ݳ���Ϊ�ֽڣ�8bits��
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;//DMA_Mode_Normal;//����DMAģʽΪѭ��ģʽ
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;//DMA_Priority_Medium;//����DMAͨ�������ȼ�Ϊ������ȼ�
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART6_RX_STREAM,&DMA_InitStructure);
	DMA_Cmd(USART6_RX_STREAM,ENABLE);
	
	//USART6_Tx
	DMA_DeInit(USART6_TX_STREAM);
	while( DMA_GetCmdStatus(USART6_TX_STREAM) == ENABLE);			//�ȴ�DMA������
	
	DMA_InitStructure.DMA_Channel 				= DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)&(USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr 		= NULL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;	//�ڴ浽����
	DMA_InitStructure.DMA_BufferSize 			= NULL;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;					//��ͨģʽ����
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(USART6_TX_STREAM, &DMA_InitStructure);
	
	DMA_Cmd(USART6_TX_STREAM, DISABLE);	//ʧ��DMA
}

/*��������USART_Receive*/
/*���������ָ�򴮿����ݽṹ��*/
/*���ã�ԭʼ�������ݣ����ݳ��Ȼ�ȡ*/
unsigned short USART_Receive(struct USART_RX_TypeDef* USARTx)
{
	USARTx->rxConter = USARTx->DMALen - DMA_GetCurrDataCounter(USARTx->DMAy_Streamx);   //����DMA��������䵽��λ��
	
	USARTx->rxBufferPtr += USARTx->rxsize;    //�ϴ�DMA��������䵽��λ��
	
	if(USARTx->rxBufferPtr >= USARTx->DMALen) //˵��DMA�������Ѿ�����һ��
	{
		USARTx->rxBufferPtr %= USARTx->DMALen;  //DMA�����������Ժ�������˵�λ��
	}
	
	if(USARTx->rxBufferPtr < USARTx->DMALen)
	{
		USARTx->rxsize = USARTx->rxConter - USARTx->rxBufferPtr;  //���㱾�ν������ݵĳ���
		
		if(USARTx->rxsize <= USARTx->MbLen)
		{
			for(u16 i = 0; i < USARTx->rxsize; i++)
			{
				*(USARTx->pMailbox + i)= *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
			}
		}
	}
	else  //DMA�������Ѿ�����һ��
	{
		USARTx->rxsize = USARTx->rxConter + USARTx->DMALen - USARTx->rxBufferPtr;   //���㱾�ν������ݳ��ȣ�����λ��+��������С-�ϴ�λ��
		
		if(USARTx->rxsize <= USARTx->MbLen)   //������ݳ���С�������С
		{
			for(u16 i = 0; i < USARTx->rxsize - USARTx->rxConter; i++)
			{
				*(USARTx->pMailbox + i)= *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);  //���ϴ���䵽��λ�ÿ�ʼ���͵�����
			}
			for(u16 i = 0; i < USARTx->rxConter; i++)
			{	
				*(USARTx->pMailbox + USARTx->rxsize - USARTx->rxConter + i)= *(USARTx->pDMAbuf + i);
			}
		}
	}
	return USARTx->rxsize;
}

enum EN_Status RX_Status = RX_FREE;  //״̬��״̬
u8 RX_Buf[USART1_RXMB_LEN] = {0}; //������������
u8 data = 0;  //״̬�����ڴ��������
u16 dataLength = 0;  //����֡����
u8 dataID = 0;  //����ID
u8 bufNum = 0;  //���ݵ�λ��

/*��������DataProtocol*/
/*���������ָ�򴮿����ݽṹ��*/
/*���ã����������ݽ���*/
void DataProtocol(struct USART_RX_TypeDef* USART_RX_Struct)
{
	for(u32 i = 0; i < USART_RX_Struct->rxsize; i++)  //ѭ�����������ݳ���һ��
	{
		data = USART_RX_Struct->pMailbox[i];
		switch(RX_Status)
		{
			case RX_FREE:
				if(data == 0x55)  //���֡ͷ1Ϊ0x55��Ϊ��ʼ
				{
					bufNum = 0;
					RX_Status = RX_Length;
				}
				else
					RX_Status = RX_FREE;
					//֡ͷ��Ϊ0x55���ȴ���һ������
				break;
				
			case RX_Length:
				dataLength = data;  //��ȡ���ݳ���
				RX_Status = RX_ID;  //����ID״̬
				break;
			
			case RX_ID:
				dataID = data;     //��ȡ����ID
				RX_Status = RX_Data;  //��������״̬
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
					/*�������ݳɹ�����ʼ����*/
//					GPIO_ToggleBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);
				}
				else
				{
					/*���ݽ���ʧ��*/
					dataLength = 0;
					dataID = 0;
					memset(RX_Buf, 0 , USART1_RXMB_LEN);
				}
				RX_Status = RX_FREE;  //׼����һ֡���ݵĽ���
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
	while(DMA_GetCurrDataCounter(USART1_TX_STREAM)); //�ȴ��������
	
	DMA_ClearITPendingBit(USART1_TX_STREAM, DMA_IT_TCIF7);  //�����������жϱ�־λ
	DMA_Cmd(USART1_TX_STREAM,DISABLE);  //���õ�ǰ����ֵ�Ƚ���DMA
	USART1_TX_STREAM->M0AR = (uint32_t)data;  //���õ�ǰ�������ݻ���ַ��Memory0 tARget
	USART1_TX_STREAM->NDTR = (uint32_t)length;  //���õ�ǰ�������ݵ�����
	DMA_Cmd(USART1_TX_STREAM, ENABLE);  //���ô���DMA����
	
}
void USART2_DMA_printf(u8* data, u8 length)
{
	while(DMA_GetCurrDataCounter(USART2_TX_STREAM)); //�ȴ��������
	
	DMA_ClearITPendingBit(USART2_TX_STREAM, DMA_IT_TCIF7);  //�����������жϱ�־λ
	DMA_Cmd(USART2_TX_STREAM,DISABLE);  //���õ�ǰ����ֵ�Ƚ���DMA
	USART2_TX_STREAM->M0AR = (uint32_t)data;  //���õ�ǰ�������ݻ���ַ��Memory0 tARget
	USART2_TX_STREAM->NDTR = (uint32_t)length;  //���õ�ǰ�������ݵ�����
	DMA_Cmd(USART2_TX_STREAM, ENABLE);  //���ô���DMA����
}


