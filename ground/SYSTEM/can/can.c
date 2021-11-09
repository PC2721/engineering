#include "can.h"
#include "global_declare.h"



/*-------------------------------------------------------
CAN1			|	Rx              |	Tx			    |
---------------------------------------------------------
引脚			|	PD0			    |	PD1		    	|
中断优先级		|	抢0子3		    |	抢1子1		    |
波特率			|	42M			    |	42M			    |
---------------------------------------------------------
功能			|   底盘电机、裁判系统、超级电容	    |
---------------------------------------------------------

---------------------------------------------------------
CAN2			|	Rx              |	Tx			    |
---------------------------------------------------------
引脚			|	PB12            | PB13		        |
中断优先级		|	抢0子4		    |	抢1子2		    |
波特率			|	42M			    |	42M			    |
---------------------------------------------------------
功能			|	拨弹电机、pitch电机、Yaw电机	    |
-------------------------------------------------------*/


/*--------------------------------------------------------------------------
函数功能：初始化CAN1配置为1M波特率
备    注：PA11(CAN1_RX);PA12(CAN1_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN1_Configuration(void)
{
    GPIO_InitTypeDef       gpio = {0};
    NVIC_InitTypeDef       nvic = {0};
    CAN_InitTypeDef        can1 = {0};
    CAN_FilterInitTypeDef  can1_fliter = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   //使能PD端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使能CAN1时钟,42MHz

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOD, &gpio);
    
    nvic.NVIC_IRQChannel                    = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;        //抢占优先级
    nvic.NVIC_IRQChannelSubPriority         = 3;        //子优先级
    nvic.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器
    
    nvic.NVIC_IRQChannel                    = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;        //抢占优先级
    nvic.NVIC_IRQChannelSubPriority         = 1;        //子优先级
    nvic.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器
 
    /****************************************CAN单元初始化*****************************************/
    CAN_DeInit(CAN1);
    CAN_StructInit(&can1);   
	/*CAN主控制寄存器器（CAN_MCR）*/
    can1.CAN_TTCM = DISABLE;        //时间触发通信模式
    can1.CAN_ABOM = DISABLE;        //自动的总线关闭管理
    can1.CAN_AWUM = DISABLE;        //自动唤醒模式
    can1.CAN_NART = DISABLE;        //禁止自动重发送
    can1.CAN_RFLM = DISABLE;        //接收FIFO锁定模式
    can1.CAN_TXFP = ENABLE;         //发送FIFIO优先级
	/*CAN位时序寄存器（CAN_BTR）*/
	/*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    can1.CAN_Mode = CAN_Mode_Normal;//普通模式
    can1.CAN_SJW  = CAN_SJW_1tq;    //重新同步跳跃宽度
    can1.CAN_BS1  = CAN_BS1_9tq;    //时间段1
    can1.CAN_BS2  = CAN_BS2_4tq;    //时间段2
    can1.CAN_Prescaler = 3;         //分频系数
    CAN_Init(CAN1, &can1);          //根据指定的参数初始化CAN寄存器
	/****************************************CAN过滤器初始化****************************************/
    can1_fliter.CAN_FilterNumber         = 0;                       //过滤器0
    can1_fliter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为掩码模式
    can1_fliter.CAN_FilterScale          = CAN_FilterScale_32bit;   //一个32位标识符过滤器
    can1_fliter.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    can1_fliter.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    can1_fliter.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    can1_fliter.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    can1_fliter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    can1_fliter.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&can1_fliter);             //根据指定的参数初始化CAN_Filter寄存器
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //接收中断:FIFO 0消息挂起中断
    CAN_ITConfig(CAN1, CAN_IT_TME,  ENABLE);  //发送中断：发送邮箱空中断 
}


/*--------------------------------------------------------------------------
函数功能：将CAN2配置为1M波特率
备    注：PB5(CAN2_RX);PB6(CAN2_TX)
          BaudRate = 42MHz/Prescaler*(1+BS1+BS2)
--------------------------------------------------------------------------*/
void CAN2_Configuration(void)
{
    GPIO_InitTypeDef       gpio = {0};
    NVIC_InitTypeDef       nvic = {0};
    CAN_InitTypeDef        can2 = {0};
    CAN_FilterInitTypeDef  can2_filter = {0};

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能PB端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使能CAN1时钟*注意*若单独使用can2,需先使能can1的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,  ENABLE);   //使能CAN2时钟,42MHz

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode  = GPIO_Mode_AF; //复用模式
    GPIO_Init(GPIOB, &gpio);        //根据设定参数初始化GPIOA

    nvic.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;         //抢占优先级   1101
    nvic.NVIC_IRQChannelSubPriority        = 3;         //子优先级
    nvic.NVIC_IRQChannelCmd                = ENABLE;    //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器
    
    nvic.NVIC_IRQChannel                   = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;         //抢占优先级
    nvic.NVIC_IRQChannelSubPriority        = 1;         //子优先级
    nvic.NVIC_IRQChannelCmd                = ENABLE;    //IRQ通道使能
    NVIC_Init(&nvic);   //根据指定的参数初始化NVIC寄存器
    
    /****************************************CAN单元初始化*****************************************/
    CAN_DeInit(CAN2);
    CAN_StructInit(&can2);
	/*CAN主控制寄存器器（CAN_MCR）*/
    can2.CAN_TTCM = DISABLE;        //时间触发通信模式
    can2.CAN_ABOM = DISABLE;        //自动的总线关闭管理
    can2.CAN_AWUM = DISABLE;        //自动唤醒模式
    can2.CAN_NART = DISABLE;        //禁止自动重发送
    can2.CAN_RFLM = DISABLE;        //接收FIFO锁定模式
    can2.CAN_TXFP = ENABLE;         //发送FIFIO优先级
	/*CAN位时序寄存器（CAN_BTR）*/
	/*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    can2.CAN_Mode = CAN_Mode_Normal;//普通模式
    can2.CAN_SJW  = CAN_SJW_1tq;    //重新同步跳跃宽度
    can2.CAN_BS1  = CAN_BS1_9tq;    //时间段1
    can2.CAN_BS2  = CAN_BS2_4tq;    //时间段2
    can2.CAN_Prescaler = 3;         //分频系数
    CAN_Init(CAN2, &can2);          //根据指定的参数初始化CAN寄存器
	/*CAN过滤器寄存器*/   
    can2_filter.CAN_FilterNumber         = 14;//过滤器14
    can2_filter.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为掩码模式
    can2_filter.CAN_FilterScale          = CAN_FilterScale_32bit;   //一个32位标识符过滤器
    can2_filter.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    can2_filter.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    can2_filter.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    can2_filter.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    can2_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    can2_filter.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&can2_filter);//根据指定的参数初始化CAN_Filter寄存器
    
	/*CAN中断使能寄存器（CAN_IER）*/
    CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE); //接收中断:FIFO 0消息挂起中断
    CAN_ITConfig(CAN2,CAN_IT_TME,  ENABLE); //发送中断：发送邮箱空中断
}

u8 CAN_Send(CAN_TypeDef *CANx, u32 id, s16 data1, s16 data2, s16 data3, s16 data4)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = id;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(data1>>8);
    tx_message.Data[1] = (u8)data1;
    tx_message.Data[2] = (u8)(data2>>8);
    tx_message.Data[3] = (u8)data2;
    tx_message.Data[4] = (u8)(data3>>8);
    tx_message.Data[5] = (u8)data3;
    tx_message.Data[6] = (u8)(data4>>8);
    tx_message.Data[7] = (u8)data4;
    
    return CAN_Transmit(CANx,&tx_message);
}

void Abs_Encoder_Process(ST_ENCODER* pEncoder, s32 value)
{
	pEncoder->uiPreRawValue = pEncoder->uiRawValue;
	pEncoder->uiRawValue = value;
	pEncoder->siDiff = pEncoder->uiRawValue - pEncoder->uiPreRawValue;
	
	if(pEncoder->siDiff < (-pEncoder->siNumber/2) ) //反转过0，圈数变化
	{
		pEncoder->siDiff += pEncoder->siNumber;
	}
	else if(pEncoder->siDiff > (pEncoder->siNumber/2)) //正传过0，圈数变化
	{
		pEncoder->siDiff -= pEncoder->siNumber;
	}
	
	else 
		pEncoder->fpSumValue += pEncoder->siDiff; //记录编码器的总线数	

}

/*-------------------------------------------------------------------------------------------------
函 数 名：GetEncoderNumber
函数功能：接收6025/6623/RM3510/RM3508电调板返回的机械角度值（绝对式编码器值）
备    注：机械角度值范围：0~8191（0x1FFF）
-------------------------------------------------------------------------------------------------*/
s32 Get_Encoder_Number(CanRxMsg* rx_message)
{
    s32 encoder_temp;
	encoder_temp = rx_message->Data[0]<<8 | rx_message->Data[1];
	return encoder_temp;
}


/*-------------------------------------------------------------------------------------------------
函 数 名：Get_Speed
函数功能：接收RM3510/RM3508电调板返回的转速，单位：r/min
备    注：RM3508电机减速比为1:19；RM3510电机减速比有1:19和1:27
-------------------------------------------------------------------------------------------------*/
s32 Get_Speed(CanRxMsg* rx_message)
{
    s32 speed_temp;
	s32 base_value=0xFFFF;
	if(rx_message->Data[2] & 0x01<<7)
	{	
		speed_temp = (base_value<<16 | rx_message->Data[2]<<8 | rx_message->Data[3]);
	}
	else
	{
		speed_temp = (rx_message->Data[2]<<8 | rx_message->Data[3]);//rpm
	}
	return speed_temp;
}


void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
		CAN_Receive(CAN2,CAN_FIFO0, &CAN2_RxMsg);
		
		switch(CAN2_RxMsg.StdId)
		{
//			case 0x201:
//				Abs_Encoder_Process(&g_stLF_Encoder, Get_Encoder_Number(&CAN2_RxMsg));
//				g_stGM_LF_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/(FP32)g_stLF_Encoder.uiGearRatio;
//				g_stGM_LF_PosPID.m_fpFB = g_stLF_Encoder.fpSumValue/(FP32)g_stLF_Encoder.siNumber/(FP32)g_stLF_Encoder.uiGearRatio*360.0f;
//				system_monitor.g_stLF_cnt++;
//				break;
//			case 0x202:
//				Abs_Encoder_Process(&g_stLB_Encoder, Get_Encoder_Number(&CAN2_RxMsg));
//				g_stGM_LB_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/(FP32)g_stLB_Encoder.uiGearRatio;
//				g_stGM_LB_PosPID.m_fpFB = g_stLB_Encoder.fpSumValue/(FP32)g_stLB_Encoder.siNumber/(FP32)g_stLB_Encoder.uiGearRatio*360.0f;
//				system_monitor.g_stLB_cnt++;
//				break;
			case 0x201:
				Abs_Encoder_Process(&g_stGround_Encoder, Get_Encoder_Number(&CAN2_RxMsg));
				g_stGM_Ground_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/(FP32)g_stLF_Encoder.uiGearRatio;
				g_stGM_Ground_PosPID.m_fpFB = g_stGround_Encoder.fpSumValue/(FP32)g_stGround_Encoder.siNumber/(FP32)g_stGround_Encoder.uiGearRatio*360.0f;
				system_monitor.g_stLF_cnt++;
				break;
//			case 0x202:
//				Abs_Encoder_Process(&g_stGround_Encoder, Get_Encoder_Number(&CAN2_RxMsg));
//				g_stGM_Ground_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/(FP32)g_stLF_Encoder.uiGearRatio;
//				g_stGM_Ground_PosPID.m_fpFB = g_stGround_Encoder.fpSumValue/(FP32)g_stGround_Encoder.siNumber/(FP32)g_stGround_Encoder.uiGearRatio*360.0f;
//				system_monitor.g_stLF_cnt++;
//				break;
			case 0x203:
				Abs_Encoder_Process(&g_stLift_L_Encoder,Get_Encoder_Number(&CAN2_RxMsg));
				g_stGM_Lift_L_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/g_stLift_L_Encoder.uiGearRatio;
				if(Lift_L_first)
				{
					ErrLift_L = g_stLift_L_Encoder.fpSumValue/g_stLift_L_Encoder.siNumber/g_stLift_L_Encoder.uiGearRatio*360.0f;
					Lift_L_first = false;
				}
				if(Lift_Switch_Flag)
				{
					g_stGM_Lift_L_PosPID.m_fpFB = g_stLift_L_Encoder.fpSumValue/g_stLift_L_Encoder.siNumber/g_stLift_L_Encoder.uiGearRatio*360.0f;
					g_stGM_Lift_L_PosPID.m_fpFB -= ErrLift_L;
				}
				system_monitor.g_stGM_Lift_L_cnt++;
				break;
			case 0x204:
				Abs_Encoder_Process(&g_stYaw1_Encoder,Get_Encoder_Number(&CAN2_RxMsg));
				g_stGM_Yaw1_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/g_stYaw1_Encoder.uiGearRatio;
				g_stGM_Yaw1_PosPID.m_fpFB = g_stYaw1_Encoder.fpSumValue/g_stYaw1_Encoder.siNumber/g_stYaw1_Encoder.uiGearRatio*360.0f;
				if(Yaw1_first)
				{
					ErrYaw1 = g_stGM_Yaw1_PosPID.m_fpFB;
					Yaw1_first = false;
				}
				g_stGM_Yaw1_PosPID.m_fpFB -= ErrYaw1;
				system_monitor.g_stGM_Yaw1_cnt++;
				break;
				case 0x209:
				Abs_Encoder_Process(&g_stNip1_Encoder,Get_Encoder_Number(&CAN2_RxMsg));
				g_stGM_Nip1_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/g_stNip1_Encoder.uiGearRatio;
				g_stGM_Nip1_PosPID.m_fpFB = g_stNip1_Encoder.fpSumValue/g_stNip1_Encoder.siNumber/g_stNip1_Encoder.uiGearRatio*360.0f;
				if(Nip1_first)
				{
					ErrNip1 = g_stGM_Nip1_PosPID.m_fpFB;
					Nip1_first = false;
				}
				g_stGM_Nip1_PosPID.m_fpFB -= ErrNip1;
				system_monitor.g_stGM_Nip1_cnt++;
				break;
			case 0x20B:
				Abs_Encoder_Process(&g_stNip2_Encoder,Get_Encoder_Number(&CAN2_RxMsg));
				g_stGM_Nip2_SpeedPID.m_fpFB = Get_Speed(&CAN2_RxMsg)/g_stNip2_Encoder.uiGearRatio;
				g_stGM_Nip2_PosPID.m_fpFB = g_stNip2_Encoder.fpSumValue/g_stNip2_Encoder.siNumber/g_stNip2_Encoder.uiGearRatio*360.0f;
				if(Nip2_first)
				{
					ErrNip2 = g_stGM_Nip2_PosPID.m_fpFB;
					Nip2_first = false;
				}
				g_stGM_Nip2_PosPID.m_fpFB -= ErrNip2;
				system_monitor.g_stGM_Nip2_cnt++;
				break;
			default:
				break;
		}
//		if(CAN2_RxMsg.StdId == TEST_ID)
//		{
//							g_usSwitch=(CAN2_RxMsg.Data[0])|(CAN2_RxMsg.Data[1]<<8);
//				if((g_usSwitch&0x64)!=0)
//					Testswitch = true;
//				else
//					Testswitch = false;
//		}
//		system_monitor.CAN2_cnt++;
    }
}


void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
    }
}


void CAN1_RX0_IRQHandler(void)
{
   if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		CAN_Receive(CAN1,CAN_FIFO0, &CAN1_RxMsg);
		
		switch(CAN1_RxMsg.StdId)
		{
			case 0x207:
				Abs_Encoder_Process(&g_stRF_Encoder, Get_Encoder_Number(&CAN1_RxMsg));
				g_stGM_RF_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/(FP32)g_stRF_Encoder.uiGearRatio;
				g_stGM_RF_PosPID.m_fpFB = g_stRF_Encoder.fpSumValue/(FP32)g_stRF_Encoder.siNumber/(FP32)g_stRF_Encoder.uiGearRatio*360.0f;
				system_monitor.g_stRF_cnt++;
				break;
			case 0x208:
				Abs_Encoder_Process(&g_stRB_Encoder, Get_Encoder_Number(&CAN1_RxMsg));
				g_stGM_RB_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/(FP32)g_stRB_Encoder.uiGearRatio;
				g_stGM_RB_PosPID.m_fpFB = g_stRB_Encoder.fpSumValue/(FP32)g_stRB_Encoder.siNumber/(FP32)g_stRB_Encoder.uiGearRatio*360.0f;
				system_monitor.g_stRB_cnt++;
				break;
			case 0x206:
				Abs_Encoder_Process(&g_stLift_R_Encoder,Get_Encoder_Number(&CAN1_RxMsg));
				g_stGM_Lift_R_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/g_stLift_R_Encoder.uiGearRatio;
				if(Lift_R_first)
				{
					ErrLift_R = g_stLift_R_Encoder.fpSumValue/g_stLift_R_Encoder.siNumber/g_stLift_R_Encoder.uiGearRatio*360.0f;
					Lift_R_first = false;
				}
				if(Lift_Switch_Flag)
				{
					g_stGM_Lift_R_PosPID.m_fpFB = g_stLift_R_Encoder.fpSumValue/g_stLift_R_Encoder.siNumber/g_stLift_R_Encoder.uiGearRatio*360.0f;
					g_stGM_Lift_R_PosPID.m_fpFB -= ErrLift_R;
				}
				system_monitor.g_stGM_Lift_R_cnt++;
				break;
//			case 0x204:
//				Abs_Encoder_Process(&g_stYaw2_Encoder,Get_Encoder_Number(&CAN1_RxMsg));
//				g_stGM_Yaw2_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/g_stYaw2_Encoder.uiGearRatio;
//				g_stGM_Yaw2_PosPID.m_fpFB = g_stYaw2_Encoder.fpSumValue/g_stYaw2_Encoder.siNumber/g_stYaw2_Encoder.uiGearRatio*360.0f;
//				if(Yaw2_first)
//				{
//					ErrYaw2 = g_stGM_Yaw2_PosPID.m_fpFB;
//					Yaw2_first = false;
//				}
//				g_stGM_Yaw2_PosPID.m_fpFB -= ErrYaw2;
//				system_monitor.g_stGM_Yaw2_cnt++;
//				break;
			case 0x205:
				Abs_Encoder_Process(&g_stNip1_R_Encoder,Get_Encoder_Number(&CAN1_RxMsg));
				g_stGM_Nip1_R_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/g_stNip1_R_Encoder.uiGearRatio;
				g_stGM_Nip1_R_PosPID.m_fpFB = g_stNip1_R_Encoder.fpSumValue/g_stNip1_R_Encoder.siNumber/g_stNip1_R_Encoder.uiGearRatio*360.0f;
				if(Nip1_R_first)
				{
					ErrNip1_R = g_stGM_Nip1_R_PosPID.m_fpFB;
					Nip1_R_first = false;
				}
				g_stGM_Nip1_R_PosPID.m_fpFB -= ErrNip1_R;
				system_monitor.g_stGM_Nip1_R_cnt++;
				break;
			case 0x20A:
				Abs_Encoder_Process(&g_stNip2_R_Encoder,Get_Encoder_Number(&CAN1_RxMsg));
				g_stGM_Nip2_R_SpeedPID.m_fpFB = Get_Speed(&CAN1_RxMsg)/g_stNip2_R_Encoder.uiGearRatio;
				g_stGM_Nip2_R_PosPID.m_fpFB = g_stNip2_R_Encoder.fpSumValue/g_stNip2_R_Encoder.siNumber/g_stNip2_R_Encoder.uiGearRatio*360.0f;
				if(Nip2_R_first)
				{
					ErrNip2_R = g_stGM_Nip2_R_PosPID.m_fpFB;
					Nip2_R_first = false;
				}
				g_stGM_Nip2_R_PosPID.m_fpFB -= ErrNip2_R;
				system_monitor.g_stGM_Nip2_R_cnt++;
				break;
				case CANTXKEYID:
				g_usSwitch=(CAN1_RxMsg.Data[0])|(CAN1_RxMsg.Data[1]<<8);
				break;
			default:
				break;
		}
//		system_monitor.CAN1_cnt++;
    }
}

void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}



UINT32 Preg_uiAirValve=0;
int fps_adc=0;
int number_adc=0;

//void SysTick_Handler(void)
//{
//	if(g_uiAirValve!=Preg_uiAirValve)
//	{
//		SendAirMsgByCan1(&g_uiAirValve); 
//	}
//  if((g_usSwitch&0x80)!=0)
//	{	
//		PhotoGateGBL = true;		//冲上看
//		PhotoGateGBR = false;
//		PhotoGateGBR_num--;
//		system_monitor.PhotogateGBR_cnt++;
//	}
//	if((g_usSwitch&0x40)!=0)
//	{	
//		PhotoGateGBR = true;			//往下看
//		PhotoGateGBL = false;
//		PhotoGateGBR_num++;
//		system_monitor.PhotogateGBL_cnt++;
//	}
//	if(g_usSwitch==0x60)
//	{	
//		PhotoGateGBR = true;
//		PhotoGateGBL = true;
//		PhotoGateGBR_num++;
//		system_monitor.PhotogateGBR_cnt++;
//		system_monitor.PhotogateGBL_cnt++;
//	}
//	if(g_usSwitch == 0x00)
//	{
//		PhotoGateGBR_num--;
//		PhotoGateGBR = false;
//		PhotoGateGBL = false;
//	}
//	Preg_uiAirValve = g_uiAirValve;
//	if(PhotoGateGBR_num>=1000)
//		PhotoGateGBR_num=1000;
//	else if(PhotoGateGBR_num<=0)
//		PhotoGateGBR_num=0;
////	if(Get_Adc(ADC_Channel_5))
////		 {
////			 number_adc++;
////		 }
////		 if(DelayCNT==1000)
////		 {
////			 DelayCNT=0;
////			 fps_adc = number_adc;
////			 number_adc=0;
////		 }
//}



