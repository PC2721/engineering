#include "general_math.h"
/*-------------------------------------------------------------------
函数功能：设置PID目标值
-------------------------------------------------------------------*/
void SetDes(ST_PID* pStPID, s32 Des)
{
	pStPID->m_fpDes = Des;
}
/*-------------------------------------------------------------------
函数功能：判断浮点数是否相等
-------------------------------------------------------------------*/
bool Is_Float_Equal(float a,float b)
{
	return fabs((double)(a - b)) < 1e-4 ? true : false;
}

/*-------------------------------------------------------------------
函数功能：削波函数，去除超过最大值与最小值之间的值，代之以最大或最小值
--------------------------------------------------------------------*/
float Clip(float fpValue, float fpMin, float fpMax)
{
	float ret;
	if(fpValue < fpMin)
	{
		ret = fpMin;
	}
	else if(fpValue > fpMax)
    {
		ret = fpMax;
	}	
	else
	{
		ret = fpValue;
	}
	return ret;
}

//抗积分饱和PID算法，系统往一个方向运动会产生较大的积分误差，会在几个周期内产生振荡或超调
void CalIResistedPID(ST_PID* pStPID)
{
	pStPID->m_fpE = pStPID->m_fpDes - pStPID->m_fpFB;  //计算当前误差
	pStPID->m_fpSumE += pStPID->m_fpE;  //计算偏差累积
	
	pStPID->m_fpSumE = Clip(pStPID->m_fpSumE, -pStPID->m_fpEiMax, pStPID->m_fpEiMax);
	pStPID->m_fpUi = pStPID->m_fpKi * pStPID->m_fpSumE;
	
	pStPID->m_fpUp = Clip(pStPID->m_fpKp * pStPID->m_fpE, -pStPID->m_fpUpMax, pStPID->m_fpUpMax);
	pStPID->m_fpUd = Clip(pStPID->m_fpKd * (pStPID->m_fpE - pStPID->m_fpPreE), -pStPID->m_fpUdMax, pStPID->m_fpUdMax);
	
	/*若偏差在死区之内，则清零积分积累项*/
	if(fabs(pStPID->m_fpE) < pStPID->m_fpEMin)
	{
		pStPID->m_fpSumE = 0;  //清除偏差累积
	}
	
	/*位置式PID计算公式*/
	pStPID->m_fpU = pStPID->m_fpUp + pStPID->m_fpUi + pStPID->m_fpUd;
	
	pStPID->m_fpPreE = pStPID->m_fpE; //保存每次偏差
	/*PID运算输出限幅*/
	pStPID->m_fpU = Clip(pStPID->m_fpU , -pStPID->m_fpUMax, pStPID->m_fpUMax);
}

//遇限削积分PID改进算法计算PID量
void CalIWeakenPID(ST_PID* pStPID)
{
	pStPID->m_fpE = pStPID->m_fpDes - pStPID->m_fpFB;  //计算当前误差
	
	if(fabs(pStPID->m_fpE) < pStPID->m_fpEMin)
	{
		pStPID->m_fpSumE += pStPID->m_fpE;  //计算偏差累积
	}
	
	pStPID->m_fpSumE = Clip(pStPID->m_fpSumE, -pStPID->m_fpEiMax, pStPID->m_fpEiMax);
	
	pStPID->m_fpUp = Clip(pStPID->m_fpKp * pStPID->m_fpE, -pStPID->m_fpUpMax, pStPID->m_fpUpMax);  //计算P项输出
	pStPID->m_fpUi = pStPID->m_fpKi * pStPID->m_fpSumE;  //计算I项输出
	pStPID->m_fpUd = Clip(pStPID->m_fpKd * (pStPID->m_fpE - pStPID->m_fpPreE), -pStPID->m_fpUdMax, pStPID->m_fpUdMax); //计算D项输出
	
	pStPID->m_fpU = pStPID->m_fpUp + pStPID->m_fpUi + pStPID->m_fpUd; //计算总输出
	
	pStPID->m_fpPreE = pStPID->m_fpE; //保存每次偏差
	/*PID运算输出限幅*/
	pStPID->m_fpU = Clip(pStPID->m_fpU , -pStPID->m_fpUMax, pStPID->m_fpUMax);
}

//void QuickPID(ST_PID* pStPID,float fpKp,float fpKi,float fpKd,float fpUMax,float fpUpMax,float fpEiMax,float fpUdMax,float fpEMin)
//{
//	pStPID->m_fpKp = fpKp;
//	pStPID->m_fpKi = fpKi;
//	pStPID->m_fpKd = fpKd;
//	pStPID->m_fpUMax = fpUMax;
//	pStPID->m_fpUpMax = fpUpMax;
//	pStPID->m_fpEiMax = fpEiMax;
//	pStPID->m_fpUdMax = fpUdMax;
//	pStPID->m_fpEMin  = fpEMin;

//}

/*-------------------------------------------------------------------------------------------------
函 数 名：RampSignal()
函数功能：斜坡输入信号
备    注：
-------------------------------------------------------------------------------------------------*/
void RampSignal(FP32* p_Output, FP32 DesValue, FP32 Step)  
{
    UCHAR8 type;
	if(*p_Output < DesValue) type = 0;  //当前速度小于目标值
	else if(*p_Output > DesValue) type = 1; //速度大于
	if(!type)   //小于目标值
	{
		if(*p_Output >= DesValue) *p_Output = DesValue;
		else 
		{
		    *p_Output += Step;
		    if(*p_Output >= DesValue) *p_Output = DesValue;
		}
	}
	else //大于目标值
	{

		if(*p_Output <= DesValue) *p_Output = DesValue;
		else 
        {
            *p_Output -= Step;
            if(*p_Output <= DesValue) *p_Output = DesValue;
        }
	}
}

/*-------------------------------------------------------------------------------------------------
函 数 名：SinSignal()
函数功能：将输入信号变成正弦
备    注：lenth输出和目标差值
-------------------------------------------------------------------------------------------------*/
FP32 D_L=0.11f;
FP32 D_L_W=1.0f;
void SinSignal(FP32* p_Output, FP32 DesValue, FP32 Lenth, FP32 Step)
{
	UCHAR8 type;
	FP32 Diff;
	if(*p_Output < DesValue)
	{
		type = 0;
		Diff=DesValue-*p_Output;
	}  //当前输出小于目标值
	else
	{
		type = 1; 
		Diff=*p_Output - DesValue;
	} //输出大于目标值
	
	D_L = 3.1415926f * Diff/Lenth;
	if(D_L>=3.0f)	D_L=3.0f;
	
	if(!type)   //小于目标值
	{
		*p_Output = *p_Output + (1+sin(D_L_W*D_L-3.14159f/2))*Step;
	}
	else
	{
		*p_Output = *p_Output - (1+sin(D_L_W*D_L-3.14159f/2))*Step;
	}
}	


//角度解算成(-180,+180]
float Angle_Inf_To_180(float angle)
{
    if(fabs(angle)>1800000)
	{
		angle = 0;
	}
    else
    {
        int temp = ((int)angle)/360;
        angle -= 360.0f*temp;
        if(angle>+180)
		{
			angle-=360;
		}
        else if(angle<=-180)
		{
			angle+=360;
		}
    }
	return angle;
}

//角度解算成[0,360)
float Angle_Inf_To_360(float angle)
{
	angle = Angle_Inf_To_180(angle);
    if(angle < 0)
	{
		angle += 360;
	}
	return angle;
}








