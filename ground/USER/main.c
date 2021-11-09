#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "global_declare.h"
//抬升电机相关参数
float Lift_Des_Max, Lift_Des_Reset, Lift_Des_Min;
float Lift_Des = 0, Lift_FB = 0;
int lift=1;

//旋转夹爪电机
float ground_FB=0;
float ground_Des = -140;


//四边形电机
float Nip1_FB = 0.0f;
float Nip1_Des = 15.0f;
float Nip_Des_Max = 130.0f;
FP32 step_Nip1=0.01;
int Nip1=1;

int t=0;

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	USART1_Init(100000);  //遥控器数据接收
	USART2_Init(115200);  //裁判系统，波特率115200
	USART3_Init(691200);  //双主控通讯
	USART6_Init(460800);  //视觉通讯
	UART5_Init(115200);
	UART4_Init(115200);
	CAN1_Configuration();
//	delay_ms(2000);
	CAN2_Configuration();
//	delay_ms(2000);

//	TIM2pwm_Configuration();   //寒道测试
	SysTick_Config(168000000/1000);                          
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

//	WWDG_Configuration();
//	delay_ms(1000);
	
//	while(1){
//		g_stGM_Lift_L_PosPID.m_fpFB = g_stLift_L_Encoder.fpSumValue;
//		g_stGM_Lift_R_PosPID.m_fpFB = g_stLift_R_Encoder.fpSumValue;
//		RampSignal(&g_stGM_Lift_L_PosPID.m_fpDes, Lift_Des, 1);
//		RampSignal(&g_stGM_Lift_R_PosPID.m_fpDes, -Lift_Des, 1);
//	
////		g_stGM_Lift_R_PosPID.m_fpFB = FB_True_Lift * 716.0f / 220.0f;
////		g_stGM_Lift_L_PosPID.m_fpFB = FB_True_Lift_L * 705.0f / 214.0f;
//	
//		CalIWeakenPID(&g_stGM_Lift_L_PosPID);
//		CalIWeakenPID(&g_stGM_Lift_R_PosPID);
//	 
//		DiffCB12PosPID.m_fpDes = 0.0f;
//		DiffCB12PosPID.m_fpFB = g_stGM_Lift_L_PosPID.m_fpFB + g_stGM_Lift_R_PosPID.m_fpFB;
//		
//		CalIWeakenPID(&DiffCB12PosPID);
//		
//		g_stGM_Lift_L_SpeedPID.m_fpDes = g_stGM_Lift_L_PosPID.m_fpU;
//		g_stGM_Lift_R_SpeedPID.m_fpDes = g_stGM_Lift_R_PosPID.m_fpU;
//	
//		CalIWeakenPID(&g_stGM_Lift_L_SpeedPID);
//		CalIWeakenPID(&g_stGM_Lift_R_SpeedPID);
//	
//		g_stGM_Lift_L_SpeedPID.m_fpU += 2500.0;
//		g_stGM_Lift_R_SpeedPID.m_fpU += -2500.0;  
//	
//		Lift_L_Current = Clip(g_stGM_Lift_L_SpeedPID.m_fpU, -C620_CURRENT_MAX, C620_CURRENT_MAX);
//		Lift_R_Current = Clip(g_stGM_Lift_R_SpeedPID.m_fpU, -C620_CURRENT_MAX, C620_CURRENT_MAX);
//		
////		CAN_Send(CAN2, 0x200, LF_Current, LB_Current, Lift_L_Current, Yaw1_Current); 
////		CAN_Send(CAN1, 0x1FF, Nip1_Current, Lift_R_Current ,RF_Current , RB_Current);
////		if((g_stGM_Lift_L_PosPID.m_fpFB-g_stGM_Lift_L_PosPID.m_fpDes<=2)&&(g_stGM_Lift_L_PosPID.m_fpFB-g_stGM_Lift_L_PosPID.m_fpDes>=-2))//结束循环条件？？要不要改angle的值？
////		{
////			lift=0;
////		}
//	}
	Nip1Open;//爪子打开CLOSE_CYLINDER(13)
	Nip1In;//爪子不往前伸OPEN_CYLINDER(14)
	while(1)
	{		t++;
//			ground_FB = g_stGM_Ground_PosPID.m_fpFB;
//			Nip1_Des = Clip(Nip1_Des,-30.0f,Nip_Des_Max);  //限位
		ground_Des=55;
		
//			g_stGM_Nip1_R_PosPID.m_fpDes = Nip1_Des;
//			RampSignal(&g_stGM_Nip1_R_PosPID.m_fpDes, Nip1_Des, step_Nip1);//斜坡
			g_stGM_Ground_PosPID.m_fpDes=ground_Des;
			CalIWeakenPID(&g_stGM_Ground_PosPID);
			g_stGM_Ground_SpeedPID.m_fpDes = g_stGM_Ground_PosPID.m_fpU;
			CalIWeakenPID(&g_stGM_Ground_SpeedPID);
			Ground_Current  = (s16)Clip(g_stGM_Ground_SpeedPID.m_fpU, -GM6020_CURRENT_MAX, GM6020_CURRENT_MAX);
//			CAN_Send(CAN1, 0x1FF, Nip1_Current, Lift_R_Current ,RF_Current , RB_Current);
//			CAN_Send(CAN1, 0x2FF, 0, Nip1_Current, 0, 0);
//			CAN_Send(CAN2, 0x200, Ground_Current,  -Ground_Current, Lift_L_Current, Yaw1_Current);
////	CAN2_2 = CAN_Send(CAN2, 0x1FF, 0, 0, 0, 0);   
//			CAN_Send(CAN2, 0x2FF, -Nip1_Current, 0, -Nip1_Current, 0);  

//	
 
	}
 
	while(1)
	{
				Nip1_FB = g_stGM_Nip1_R_PosPID.m_fpFB;
//			Nip1_Des = Clip(Nip1_Des,-30.0f,Nip_Des_Max);  //限位
				Nip1_Des=15; 
		
//			g_stGM_Nip1_R_PosPID.m_fpDes = Nip1_Des;
			RampSignal(&g_stGM_Nip1_R_PosPID.m_fpDes, Nip1_Des, step_Nip1);//斜坡
			CalIWeakenPID(&g_stGM_Nip1_R_PosPID);
			g_stGM_Nip1_R_SpeedPID.m_fpDes = g_stGM_Nip1_R_PosPID.m_fpU;
			CalIWeakenPID(&g_stGM_Nip1_R_SpeedPID);
			Nip1_Current  = (s16)Clip(g_stGM_Nip1_R_SpeedPID.m_fpU, -GM6020_CURRENT_MAX, GM6020_CURRENT_MAX);
//			CAN_Send(CAN1, 0x1FF, Nip1_Current, Lift_R_Current ,RF_Current , RB_Current);
//			CAN_Send(CAN1, 0x2FF, 0, Nip1_Current, 0, 0);
//			CAN_Send(CAN2, 0x200, LF_Current, LB_Current, Lift_L_Current, Yaw1_Current);
////	CAN2_2 = CAN_Send(CAN2, 0x1FF, 0, 0, 0, 0);   
//			CAN_Send(CAN2, 0x2FF, -Nip1_Current, 0, -Nip1_Current, 0);  

	}
 
}

