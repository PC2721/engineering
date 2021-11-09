#ifndef __PID_H__
#define __PID_H__

#include "math.h"
#include "stm32f4xx.h"
#include "rm_data_types.h"
#include "stdbool.h"
#define LiftMaxCurrent           (C620_CURRENT_MAX)
#define NipMaxCurrent            (C620_CURRENT_MAX)

#define MIN2(a,b)  (a<b?a:b)

typedef struct 
{
	float m_fpDes;//控制变量目标值  
	float m_fpFB;//控制变量反馈值   
	
	float m_fpKp;//比例系数Kp      
	float m_fpKi;//积分系数Ki      
	float m_fpKd;//微分系数Kd 
	
	float m_fpUp;//比例输出
	float m_fpUi;//积分输出
	float m_fpUd;//微分输出
	
	float m_fpE;//本次偏差         
	float m_fpPreE;//上次偏差      
	float m_fpSumE;//总偏差  
	
	float m_fpU;//本次PID运算结果  
	
	float m_fpUMax;//PID运算后输出最大值及做遇限削弱时的上限值  
	float m_fpUpMax;//比例项输出上限  
	float m_fpEiMax;//积分上限   
	float m_fpUdMax;//微分项输出上限

	float m_fpEMin;
}ST_PID;

void SetDes(ST_PID* pStPID, s32 Des);
bool Is_Float_Equal(float a,float b);
float Clip(float fpValue, float fpMin, float fpMax);
void CalIResistedPID(ST_PID* pStPID);
void CalIWeakenPID(ST_PID* pStPID);
float Angle_Inf_To_360(float angle);
float Angle_Inf_To_180(float angle);
void RampSignal(FP32* p_Output, FP32 DesValue, FP32 Step);
void SinSignal(FP32* p_Output, FP32 DesValue, FP32 Lenth, FP32 Step);

#endif









