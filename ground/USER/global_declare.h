#ifndef __GOLBAL_DECLARE_H__
#define __GOLBAL_DECLARE_H__

#include "stm32f4xx.h"
#include "general_math.h"
#include "aircan_protocol.h"
#include "stdbool.h"
#include "rm_data_types.h"
extern int number;
extern bool OpenOrClose;
#define C610_CURRENT_MAX  10000
#define C620_CURRENT_MAX  16384
#define LIFT_C620_CURRENT_MAX 11468
#define GM6020_CURRENT_MAX 30000
/*--------------���̿����й�------------------------------*/
#define ActionStay     111      //��ʼΪ��
#define ActionStart    0		//Ϊ����ʼִ��
#define ActionFinished 88		//Ϊ�����ʾ���������
#define ActionFirst    100		//��ʾ�����Ѿ���ɣ�����δ�ɿ�����
#define Ready_Third_Out 1
#define Ready_Third_Back 10
#define Ex_One_State1  12
#define Ex_Two_State1  9

/*--------------------�㷨�ṹ������--------------------------------*/
typedef struct
{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
}Filter_t;				//�б�ѩ��II��ͨ�˲�

typedef struct
{
	float preout;
	float out;
	float in;
	float off_freq;
	float samp_tim;
	bool clear;
}ST_LPF;

typedef struct
{
	float x1;
	float x2;
	float x;
	float r;
	float h;
	float T;
	float aim;
}TD;							//����΢����

#define DECLARE_MONITOR(name) USHORT16 name##_cnt; USHORT16 name##_fps;
typedef struct 
{
	DECLARE_MONITOR(System)
	DECLARE_MONITOR(CAN1)
	DECLARE_MONITOR(CAN2)
	DECLARE_MONITOR(USART1) 
	DECLARE_MONITOR(USART2)
	DECLARE_MONITOR(USART3)
	DECLARE_MONITOR(UART4)
	DECLARE_MONITOR(UART5)
	DECLARE_MONITOR(USART6)
	DECLARE_MONITOR(CAN1_1)
	DECLARE_MONITOR(CAN1_2)
	DECLARE_MONITOR(CAN1_3)
	DECLARE_MONITOR(CAN2_1)
	DECLARE_MONITOR(CAN2_2)
	DECLARE_MONITOR(CAN2_3)
	
	DECLARE_MONITOR(StrategyTask)
	DECLARE_MONITOR(SendDataTask)
	DECLARE_MONITOR(ChassisTask)
	DECLARE_MONITOR(GBTask)	
	DECLARE_MONITOR(LiftUpTask)
	DECLARE_MONITOR(LedTask)
	DECLARE_MONITOR(TranPicTask)
	DECLARE_MONITOR(ObstacleTask)
	DECLARE_MONITOR(g_stLF)
	DECLARE_MONITOR(g_stLB)
	DECLARE_MONITOR(g_stRF)
	DECLARE_MONITOR(g_stRB)
	DECLARE_MONITOR(g_stOB1)
	DECLARE_MONITOR(g_stOB2)
	DECLARE_MONITOR(g_stGM_Lift_L)
	DECLARE_MONITOR(g_stGM_Lift_R)
	DECLARE_MONITOR(g_stGM_Nip1) 
	DECLARE_MONITOR(g_stGM_Nip1_R)
	DECLARE_MONITOR(g_stGM_Nip2) 
	DECLARE_MONITOR(g_stGM_Nip2_R)
	DECLARE_MONITOR(g_stGM_Stage)
	DECLARE_MONITOR(g_stGM_Limit)
	DECLARE_MONITOR(g_stGM_Yaw1)
	DECLARE_MONITOR(g_stGM_Yaw2)
	DECLARE_MONITOR(test_air)
	DECLARE_MONITOR(PhotogateGBL)
	DECLARE_MONITOR(PhotogateGBR)
	UINT32 TaskTotalTime;
	UINT32 TaskTotalTimeMost;
}SYSTEM_MONITOR;
extern SYSTEM_MONITOR system_monitor;
/*------------------------------�����˿���ģʽ�ṹ��---------------------*/
typedef enum
{
	RC_Mode,//ң����ģʽ������ģʽ��  0
	Mouse_Mode,
	Safe_Mode,
	AlwaysSafe_Mode
}EM_OPERATION_MODE;
extern EM_OPERATION_MODE g_emOperation_Mode;
extern UCHAR8 Flag_Safety_Mode;
/*---------------------------�������ṹ��--------------------------------*/
typedef struct
{
	float uiRawValue;		//���α�������ԭʼֵ
	float uiPreRawValue;	//��һ�α�������ԭʼֵ
	float siDiff;			//����������ԭʼֵ�Ĳ�ֵ
	float fpSpeed;      //�����������ת�� r/min
	float uiGearRatio;    //���ٱ�
	float siNumber;			//����������
	float fpSumValue;		//�������ۼ�ֵ
}ST_ENCODER;
/*-----------------------�����˵����˶��ٶȽṹ��-----------------------*/
typedef struct
{
	FP32 fpVx; //�ط����ٶȣ���λmm/s��
	FP32 fpVy; //Y�����ٶȣ���λ��mm/s��
	FP32 fpW;  //���ٶȣ���λrad/s��
}ST_VELT;

/*------------------------�����˵����˶��ٶȽṹ��----------------------*/
typedef struct
{
	FP32 fpXx; //�ط���λ��
	FP32 fpXy; //Y����λ��
	FP32 fpR;  //�Ƕ�
}ST_POS;

/*------------------------������ṹ��----------------------*/
typedef struct
{
	FP32 UsefulDist;
	USHORT16 RealDist;
	USHORT16 Status;
	UCHAR8 Time;
	UCHAR8 Mode;
	USHORT16 CheckSum;
	bool Gy53OK;
}Gy53_Rx_TypeDef;
/*---------------------------����̧�����--------------------------------*/
extern ST_ENCODER g_stLift_L_Encoder;
extern ST_ENCODER g_stLift_R_Encoder;
extern ST_PID g_stGM_Lift_L_PosPID;
extern ST_PID g_stGM_Lift_L_SpeedPID;
extern ST_PID g_stGM_Lift_R_PosPID;
extern ST_PID g_stGM_Lift_R_SpeedPID;
extern ST_PID DiffCB12PosPID;
extern s16 Lift_L_Current;
extern s16 Lift_R_Current;
extern float ErrLift_L, ErrLift_R;
extern bool Lift_L_first, Lift_R_first;
extern UCHAR8 GetInState;
extern UCHAR8 GetOBState;		//��¼ʱ״̬���λ
extern UINT32 DelayGetIn;
extern UINT32 DelayGetOB;
extern bool ResetMode;
extern bool Lift_L_First;
extern bool Lift_R_First;
extern bool True_Lift_L_first;
extern bool True_Lift_R_first;

extern float adcx;
extern float FB_True_Lift_L;
extern float FB_True_Lift;
extern bool Lift_Switch_Flag;
extern float Err_Lift;
extern bool Lift_first;
/*---------------------------���¼�צ���--------------------------------*/
extern ST_ENCODER g_stNip1_Encoder;
extern ST_ENCODER g_stNip1_R_Encoder;
extern ST_ENCODER g_stNip2_Encoder;
extern ST_ENCODER g_stNip2_R_Encoder;
extern ST_ENCODER g_stLimit_Encoder;
extern ST_ENCODER g_stFri1_Encoder;
extern ST_ENCODER g_stFri2_Encoder;
extern ST_ENCODER g_stYaw1_Encoder;
extern ST_ENCODER g_stYaw2_Encoder;
extern ST_ENCODER g_stGround_Encoder;
extern ST_PID g_stGM_Nip1_PosPID;
extern ST_PID g_stGM_Nip1_SpeedPID;
extern ST_PID g_stGM_Nip1_R_PosPID;
extern ST_PID g_stGM_Nip1_R_SpeedPID;
extern ST_PID g_stGM_Nip2_PosPID;
extern ST_PID g_stGM_Nip2_SpeedPID;
extern ST_PID g_stGM_Nip2_R_PosPID;
extern ST_PID g_stGM_Nip2_R_SpeedPID;
extern ST_PID g_stGM_Limit_PosPID;
extern ST_PID g_stGM_Limit_SpeedPID;
extern ST_PID g_stGM_Fri1_PosPID;
extern ST_PID g_stGM_Fri1_SpeedPID;
extern ST_PID g_stGM_Fri2_PosPID;
extern ST_PID g_stGM_Fri2_SpeedPID;
extern ST_PID g_stGM_Yaw1_PosPID;
extern ST_PID g_stGM_Yaw1_SpeedPID;
extern ST_PID g_stGM_Yaw2_PosPID;
extern ST_PID g_stGM_Yaw2_SpeedPID;
extern ST_PID g_stGM_Ground_PosPID;
extern ST_PID g_stGM_Ground_SpeedPID;
extern s16 Limit_Current;
extern s16 Nip1_Current;
extern s16 Fri1_Current;
extern s16 Fri2_Current;
extern s16 Yaw1_Current;
extern s16 Yaw2_Current;
extern s16 Ground_Current;
extern float ErrNip1;
extern float ErrNip1_R;
extern float ErrNip2;
extern float ErrNip2_R;
extern float ErrLimit;
extern float ErrFri1;
extern float ErrFri2;
extern float ErrYaw1;
extern float ErrYaw2;
extern float ErrGround;
extern bool Nip1_first;
extern bool Nip1_R_first;
extern bool Nip2_first;
extern bool Nip2_R_first;
extern bool Fri1_first;
extern bool Fri2_first;
extern bool Limit_first;
extern bool Yaw1_first;
extern bool Yaw2_first;

extern u8 QR_One;
extern u8 QR_Two;

extern bool Third_Out_Flag;
extern u8 Get_One_State;
extern u8 Get_Auto_State;
extern u8 Get_Auto2_State;
extern u8 Ready_Third_State;
extern u8 Get_Third_State;
extern u8 Search_QR_State;
extern u8 Ex_One_State;
extern u8 Ex_Two_State;
extern u8 Ex_Three_State;
extern u8 Reset_Nip_State;
extern u8 Nip1_Reset_State;
extern u8 Fri_180_State;
extern u8 Return_Ex_State;
extern u8 Return_Third_State;
extern u8 Get_Ground_State;
extern u8 Adjust_State	;
extern bool Get_One_Action;
extern bool Get_Third_Action;
extern bool Get_Auto_Action;
extern bool No_Current_Flag;
extern bool No_Lift_Current_Flag;
extern bool No_Nip_Current_Flag;
extern bool No_Current_Reset_Flag;
extern bool No_Fri_Current_Flag;
extern bool ReadyToThird;

extern float Nip1_Des_Close;
extern float Nip1_Des_Reset;
extern u32 Nip1_Time;
extern float Nip1_Des_Error;

extern int Limit_flag; 
extern bool Nip1out_flag;
extern int PhotoGateGBR_num;
/*---------------------------���̵������ǰ����ǰ��----------------------*/
extern ST_PID g_stGM_LF_PosPID;
extern ST_PID g_stGM_LB_PosPID;
extern ST_PID g_stGM_RF_PosPID;
extern ST_PID g_stGM_RB_PosPID;
extern ST_PID g_stGM_LF_SpeedPID;
extern ST_PID g_stGM_LB_SpeedPID;
extern ST_PID g_stGM_RF_SpeedPID;
extern ST_PID g_stGM_RB_SpeedPID;
extern ST_ENCODER g_stLF_Encoder;
extern ST_ENCODER g_stLB_Encoder;
extern ST_ENCODER g_stRF_Encoder;
extern ST_ENCODER g_stRB_Encoder;
extern float Chassis_Des;
extern s16 LF_Current;
extern s16 LB_Current;
extern s16 RF_Current;
extern s16 RB_Current;
extern u8 Direction;				//�����л�����
extern u8 Direction_U;
extern u8 view_angle;
extern float View_Distance_led;
extern float View_Distance_close;
extern u8 View_Run;
extern u8 View_Run_angle;
extern float Com_Distance_led;
extern u8 view_direction;
extern u8 view_state;
extern ST_PID g_stGM_View_PosPID;
extern ST_PID st_ChassisTheta;
extern ST_PID st_ChassisYPosPID;
extern Gy53_Rx_TypeDef Gy53_DataL;
extern Gy53_Rx_TypeDef Gy53_DataR;

//������
#define GY53_DATA_Len 8
#define GY53_DATA_HEAD 0x5A
/*---------------------------�ϰ����������ң�----------------------*/
extern ST_PID g_stOB1_PosPID;
extern ST_PID g_stOB2_PosPID;
extern ST_PID g_stOB1_SpeedPID;
extern ST_PID g_stOB2_SpeedPID;
extern ST_ENCODER g_stOB1_Encoder;
extern ST_ENCODER g_stOB2_Encoder;
extern s16 Obstacle_Current1;
extern s16 Obstacle_Current2;

extern u8 Obstacle_Run_State;
extern u8 Obstacle_Reset_State;

extern bool OB1_first;
extern bool OB2_first;
extern float ErrOB1;
extern float ErrOB2;

extern ST_VELT st_velt;
extern ST_POS  st_deltaX;

extern bool ReadyToMove;
extern bool LockTheChassis;
/*��翪�ؼ�ⶨ��*/
extern bool PhotoGateGBL;
extern bool PhotoGateGBR;
extern u32 time3;

extern u8 searchflag;

extern u8 DuctState;
/*---------------ͼ�����--------------------*/
extern u8 transferState;
/*---------------����ͷ���------------------*/
extern u8 transferState_Up;
/*---------------���߳��--------------------*/
extern bool wirelessState;
/*---------------------------���׿������---------------------------------*/
extern unsigned int test_air_open;      //��������Կ���ͨ��
extern unsigned int test_air_close;     //��������Թر�ͨ��
/*�ϼ�צ*/
#define Nip1Open		CLOSE_CYLINDER(13) //���ϼ�צ
#define Nip1Close		OPEN_CYLINDER(13)
#define Nip1Out     	CLOSE_CYLINDER(14);Nip1out_flag = true//�ϼ�צ����
#define Nip1In     		OPEN_CYLINDER(14);Nip1out_flag = false
#define PushOut    		OPEN_CYLINDER(15)//�һ���ʯ
#define PushIn			CLOSE_CYLINDER(15)
#define Nip1Up			OPEN_CYLINDER(2)
#define Nip1Down		CLOSE_CYLINDER(2)

//�̶����� ������̬����
#define TightOne		OPEN_CYLINDER(8)//��7
#define LooseOne		CLOSE_CYLINDER(8)
#define PushTwo         OPEN_CYLINDER(5)//��
#define PullTwo     	CLOSE_CYLINDER(5)//��
#define StageUp         OPEN_CYLINDER(3)//����̨����
#define StageDown       CLOSE_CYLINDER(3)//����̨�½�
/*��Ԯ*/
#define ClawOut			OPEN_CYLINDER(7)     //�����Ԯ
#define ClawIn			CLOSE_CYLINDER(7)    //�ջؾ�Ԯ
#define ClawDown		OPEN_CYLINDER(4)    //�ſ���Ԯ
#define ClawUp		    CLOSE_CYLINDER(4)     //�պϾ�Ԯ
/*RFID��Ԯ*/
#define RFIDOut			OPEN_CYLINDER(6)     //�����Ԯ
#define RFIDIn			CLOSE_CYLINDER(6)    //�ջؾ�Ԯ
/*---------------------------USART------------------------------*/
typedef struct
{
	u8 head[4];			  //4
	float Distance_close; //4
	float Distance_led;   //4
	u8 view_direction;	  //1
	u8 Get_Num;			  //1
	u8 QR_One;			  //1	
	u8 QR_Two;			  //1
	u8 tail[2];			  //2
}View_Receive;		//18


typedef struct
{
	u8 head[2];			//2
	u8 start;			//1
	u8 Mode;				//1 
	u8 tail[2];			//2
}View_Send;		//6


__packed typedef struct
{
	u8 head[2];						//2
	float OB1_PosFB;			//4
	s16 OB1_SpeedFB;			//2
	float OB2_PosFB;			//4
	s16 OB2_SpeedFB;			//2
	float Fri1_PosFB;			//4
	s16 Fri1_SpeedFB;			//2
	float Fri2_PosFB;			//4
	s16 Fri2_SpeedFB;			//2
	float Ground_PosFB;		    //4
	s16 Ground_SpeedFB;		    //2
	float Limit_PosFB;			//4
	s16 Limit_SpeedFB;			//2
	float Yaw2_PosFB;			//4
	s16 Yaw2_SpeedFB;			//2
	float FB_True_Lift_L;       //4
	u8 QR_One;                  //1
	u8 QR_Two;                  //1
	u8 tail[2];					//2
}DMA_Receive;						//52

__packed typedef struct
{
	u8 head[2];						//2
	u8 Restart;						//1
	s16 OB1Current;				//2
	s16 OB2Current;				//2
	s16 Fri1Current;			//2
	s16 Fri2Current;			//2
	s16 GroundCurrent;		//2
	s16 LimitCurrent;			//2
	s16 Yaw2Current;			//2
	u8 transferState;           //1
	u8 tail[2];						//2
}DMA_Send;							//20

extern DMA_Receive USART3_rx;
extern DMA_Send USART3_tx;
extern View_Receive USART6_rx_now;
extern View_Send USART6_tx;
extern bool test_Flag;

extern u8 start_flag;
extern u8 restart;
/*---------------------------CAN--------------------------------*/
extern CanRxMsg CAN1_RxMsg;
extern CanRxMsg CAN2_RxMsg;
extern ST_ENCODER Encoder;
extern unsigned int g_uiAirValve;	     //�����й�ȫ�ֱ���
extern USHORT16 g_usSwitch;             //�����й�ȫ�ֱ���
extern USHORT16 g_usSwitchPre;          //������һʱ�̿���ֵ

/*-------------------------��λ����------------------------------------*/
extern bool PRESSED_W;
extern bool PRESSED_S;
extern bool PRESSED_A;
extern bool PRESSED_D;
extern bool PRESSED_SHIFT;
extern bool PRESSED_CTRL;
extern bool PRESSED_Q;
extern bool PRESSED_E;
extern bool PRESSED_R;
extern bool PRESSED_F;
extern bool PRESSED_G;
extern bool PRESSED_Z;
extern bool PRESSED_X;
extern bool PRESSED_C;
extern bool PRESSED_V;
extern bool PRESSED_B;
extern bool PRESSED_RIGHT;
extern bool PRESSED_LEFT;
extern bool PRESSED_CH0;
extern bool PRESSED_CH1;
extern bool PRESSED_CH2;
extern bool PRESSED_CH3;

extern bool 		 Custom_Get_Data;
extern float         test_engine_des;
extern u8 Action_Flag;

/*-------------------------------�Ӿ�ʶ�����-------------------------*/

#endif






