#include "global_declare.h"

/*---------------------------SYSTEM_MONITOR---------------------------------------*/
SYSTEM_MONITOR system_monitor = {0};

/*--------------------------机器人控制模式----------------------------------*/
EM_OPERATION_MODE g_emOperation_Mode;						//机器人控制方式
UCHAR8 Flag_Safety_Mode = 0;

/*---------------------------抬升电机（左右）3508--------------------------------*/
ST_ENCODER g_stLift_L_Encoder = {0,0,0,0,19,8192,0};
ST_ENCODER g_stLift_R_Encoder = {0,0,0,0,19,8192,0};
ST_PID g_stGM_Lift_L_PosPID = {0,0,3,0.1,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};//3
ST_PID g_stGM_Lift_L_SpeedPID = {0,0,140,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};//180
ST_PID g_stGM_Lift_R_PosPID = {0,0,3,0.1,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_PID g_stGM_Lift_R_SpeedPID = {0,0,140,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_PID DiffCB12PosPID = {0,0,10.0f,0,0,0,0,0,0,0,0,0,100,100000,100000,100000,2.5};
s16 Lift_L_Current = 0;
s16 Lift_R_Current = 0;
float ErrLift_L = 0.0f, ErrLift_R = 0.0f;
bool Lift_L_first = true, Lift_R_first = true;
bool True_Lift_L_first = true, True_Lift_R_first = true;
UCHAR8 GetInState=8,GetOBState = 0;		//标定状态检测位
UINT32 DelayGetIn=0,DelayGetOB = 0;
bool ResetMode	= false;
bool Lift_L_First = true,Lift_R_First = true;
	int number = 0;
	bool OpenOrClose = false;

float adcx=0;
float FB_True_Lift_L = 0;
float FB_True_Lift=0;
bool Lift_Switch_Flag = true;
float Err_Lift = 0.0f;
bool Lift_first = true;
/*---------------------------夹爪电机（上下）2006--------------------------------*/
ST_ENCODER g_stNip1_Encoder = {0,0,0,0,3.4,8192,0};
ST_ENCODER g_stNip1_R_Encoder = {0,0,0,0,3.4,8192,0};
ST_ENCODER g_stNip2_Encoder = {0,0,0,0,3.4,8192,0};
ST_ENCODER g_stNip2_R_Encoder = {0,0,0,0,3.4,8192,0};
ST_ENCODER g_stStage_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stStage2_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stLimit_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stFri1_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stFri2_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stYaw1_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stYaw2_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stGround_Encoder = {0,0,0,0,19,8192,0};
ST_PID g_stGM_Nip1_PosPID = {0,0,6,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};//左一  //6
ST_PID g_stGM_Nip1_SpeedPID = {0,0,450,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};  //500
ST_PID g_stGM_Nip1_R_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};//右一  //6
ST_PID g_stGM_Nip1_R_SpeedPID = {0,0,200,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};  //500
ST_PID g_stGM_Nip2_PosPID = {0,0,6,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};//左二  //6
ST_PID g_stGM_Nip2_SpeedPID = {0,0,450,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};  //500
ST_PID g_stGM_Nip2_R_PosPID = {0,0,6,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};//右二  //6
ST_PID g_stGM_Nip2_R_SpeedPID = {0,0,450,0,0,0,0,0,0,0,0,0,GM6020_CURRENT_MAX,GM6020_CURRENT_MAX,0,0,0};  //500
ST_PID g_stGM_Limit_PosPID = {0,0,4,1,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//限位  //6
ST_PID g_stGM_Limit_SpeedPID = {0,0,400,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};  //500
ST_PID g_stGM_Fri1_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//夹爪摩擦片电机
ST_PID g_stGM_Fri1_SpeedPID = {0,0,300,0,0.01,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
ST_PID g_stGM_Fri2_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//夹爪摩擦片电机
ST_PID g_stGM_Fri2_SpeedPID = {0,0,300,0,0.01,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
ST_PID g_stGM_Yaw1_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//下Yaw电机
ST_PID g_stGM_Yaw1_SpeedPID = {0,0,300,0,0.01,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
ST_PID g_stGM_Yaw2_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//上Yaw电机
ST_PID g_stGM_Yaw2_SpeedPID = {0,0,300,0,0.01,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
ST_PID g_stGM_Ground_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//用于取地面矿的电机
ST_PID g_stGM_Ground_SpeedPID = {0,0,200,0,0.01,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
s16 Nip1_Current = 0;
s16 Limit_Current = 0;
s16 Fri1_Current  = 0;
s16 Fri2_Current  = 0;
s16 Yaw1_Current = 0;
s16 Yaw2_Current = 0;
s16 Ground_Current = 0;
float ErrNip1 = 0.0f;
float ErrNip1_R = 0.0f;
float ErrNip2 = 0.0f;
float ErrNip2_R = 0.0f;
float ErrFri1 = 0.0f;
float ErrFri2 = 0.0f;
float ErrLimit = 0.0f;
float ErrYaw1 = 0.0f;
float ErrYaw2 = 0.0f;
float ErrGound = 0.0f;
bool Nip1_first = true;
bool Nip1_R_first = true;
bool Nip2_first = true;
bool Nip2_R_first = true;
bool Fri1_first = true;
bool Fri2_first = true;
bool Limit_first = true;
bool Yaw1_first = true;
bool Yaw2_first = true;

u8 QR_One = 0;
u8 QR_Two = 0;

u8 Get_One_State   = ActionStay;
u8 Get_Auto_State  = ActionStay;
u8 Get_Auto2_State = ActionStay;
u8 Ready_Third_State=ActionStay;
u8 Get_Third_State = ActionStay;
u8 Search_QR_State = ActionStay;
u8 Ex_One_State    = ActionStay;
u8 Ex_Two_State    = ActionStay;
u8 Ex_Three_State  = ActionStay;
u8 Reset_Nip_State = ActionStay;
u8 Nip1_Reset_State= ActionStay;
u8 Get_Ground_State = ActionStay;
u8 Fri_180_State   = ActionStay;
u8 Return_Ex_State = ActionStay;
u8 Return_Third_State = ActionStay;
u8 Adjust_State		= ActionStay;
bool Get_One_Action    = false;
bool Get_Third_Action  = false;
bool Get_Auto_Action	 = false;
bool No_Current_Flag   = true;
bool No_Lift_Current_Flag = false;
bool No_Nip_Current_Flag = false;
bool No_Fri_Current_Flag = false;
bool No_Current_Reset_Flag = false;
bool ReadyToThird      = false;

int Limit_flag = 0;
bool Nip1out_flag = false;
int PhotoGateGBR_num = 0;
/*---------------------------底盘电机3508（左前后，右前后）----------------------*/
ST_PID g_stGM_LF_PosPID = {0,0,1,0,0,0,0,0,0,0,0,0,5000,5000,0,0,0};//120  0.01  10
ST_PID g_stGM_LB_PosPID = {0,0,1,0,0,0,0,0,0,0,0,0,5000,5000,0,0,0};
ST_PID g_stGM_RF_PosPID = {0,0,1,0,0,0,0,0,0,0,0,0,5000,5000,0,0,0};
ST_PID g_stGM_RB_PosPID = {0,0,1,0,0,0,0,0,0,0,0,0,5000,5000,0,0,0};
ST_PID g_stGM_LF_SpeedPID = {0,0,120,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_PID g_stGM_LB_SpeedPID = {0,0,120,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_PID g_stGM_RF_SpeedPID = {0,0,120,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_PID g_stGM_RB_SpeedPID = {0,0,120,0,0,0,0,0,0,0,0,0,C620_CURRENT_MAX,C620_CURRENT_MAX,0,0,0};
ST_ENCODER g_stLF_Encoder = {0,0,0,0,19,8192,0};
ST_ENCODER g_stLB_Encoder = {0,0,0,0,19,8192,0};
ST_ENCODER g_stRF_Encoder = {0,0,0,0,19,8192,0};
ST_ENCODER g_stRB_Encoder = {0,0,0,0,19,8192,0};
float Chassis_Des = 0;
s16 LF_Current = 0;
s16 LB_Current = 0;
s16 RF_Current = 0;
s16 RB_Current = 0;
u8 Direction = 0;				//图传方向切换变量
u8 Direction_U = 0;             //上侧摄像头方向切换变量
u8 view_angle = false;	//视角切换
u8 view_state = 0;
u8 view_direction = 0;
float View_Distance_led = 0;
float View_Distance_close = 0;

u8 View_Run = 0;//视觉控制
u8 View_Run_angle = 0;//相机舵机

float Com_Distance_led = 28.0f;
//视觉对位
ST_PID g_stGM_View_PosPID = {0,0,15,0.3,0,0,0,0,0,0,0,0,1500,1500,0,0,0};
//激光测距
ST_PID st_ChassisTheta=	 {0,0,0.14f,0,12.0f,0,0,0,0,0,0,0,7.0f,500.0f,500.0f,500.0f,20.0f};
ST_PID st_ChassisYPosPID= {10.0f,0,10,0,0,0,0,0,0,0,0,0,750.0f,1000.0f,100.0f,500.0f,5.0f}; 



Gy53_Rx_TypeDef Gy53_DataL = {0};
Gy53_Rx_TypeDef Gy53_DataR = {0};
/*---------------------------障碍块电机2006---------------------------------------*/
ST_PID g_stOB1_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//3
ST_PID g_stOB2_PosPID = {0,0,3,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//上下
ST_PID g_stOB1_SpeedPID = {0,0,200,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};//250
ST_PID g_stOB2_SpeedPID = {0,0,200,0,0,0,0,0,0,0,0,0,C610_CURRENT_MAX,C610_CURRENT_MAX,0,0,0};
ST_ENCODER g_stOB1_Encoder = {0,0,0,0,36,8192,0};
ST_ENCODER g_stOB2_Encoder = {0,0,0,0,36,8192,0};
s16 Obstacle_Current1 = 0;
s16 Obstacle_Current2 = 0;

u8 Obstacle_Run_State = ActionStay;
u8 Obstacle_Reset_State = ActionStay;
bool OB1_first = true;
bool OB2_first = true;
float ErrOB1 = 0;
float ErrOB2 = 0;

ST_VELT st_velt;
ST_POS	st_deltaX;

bool ReadyToMove		= true;
bool LockTheChassis     = false;
/*光电开关检测定义*/
bool PhotoGateGBL 				= false;
bool PhotoGateGBR 				= false;
u32 time3 = 0;

u8 searchflag = 0;
u8 DuctState = 0;
/*---------------图传舵机--------------------*/
u8 transferState = 0;
/*---------------摄像头舵机------------------*/
u8 transferState_Up = 0;
/*----------------无线充电--------------------*/
bool wirelessState = false;
/*---------------------------USART------------------------------*/
DMA_Receive USART3_rx = {0};
DMA_Send USART3_tx = {0,0,0,0,0,0,0,0,0,0};
View_Receive USART6_rx_now = {0};
View_Send USART6_tx = {0};
bool test_Flag = true;

u8 start_flag = 1;
u8 restart = 0;
/*---------------------------OS---------------------------------------*/


/*---------------------------CAN---------------------------------------*/
CanRxMsg CAN1_RxMsg;
CanRxMsg CAN2_RxMsg;
unsigned int g_uiAirValve;	     //气缸有关全局变量
unsigned int test_air_open;      //气动板测试开启通道
unsigned int test_air_close;     //气动板测试关闭通道
USHORT16 g_usSwitch;             //开关有关全局变量
USHORT16 g_usSwitchPre;          //保存上一时刻开关值

/*-------------------------键位定义------------------------------------*/
bool PRESSED_W                  = false;
bool PRESSED_S                  = false;
bool PRESSED_A                  = false;
bool PRESSED_D                  = false;
bool PRESSED_SHIFT              = false;
bool PRESSED_CTRL               = false;
bool PRESSED_Q                  = false;
bool PRESSED_E                  = false;
bool PRESSED_R                  = false;
bool PRESSED_F                  = false;
bool PRESSED_G                  = false;
bool PRESSED_Z                  = false;
bool PRESSED_X                  = false;
bool PRESSED_C                  = false;
bool PRESSED_V                  = false;
bool PRESSED_B                  = false;
bool PRESSED_RIGHT				= false;
bool PRESSED_LEFT				= false;
bool PRESSED_CH0                = false;
bool PRESSED_CH1                = false;
bool PRESSED_CH2                = false;
bool PRESSED_CH3                = false;

/*-------------------------------一般类型变量-------------------------*/
bool BSP_Init_FLAG              = false;         	//底层初始化完成FLAG
bool Custom_Get_Data;
float test_engine_des = 0;
u8 Action_Flag = 0;

/*-------------------------------视觉识别变量-------------------------*/










