#ifndef __PID_H__
#define __PID_H__

#include "math.h"
#include "stm32f4xx.h"
#include "stdbool.h"
#define LiftMaxCurrent           (C620_CURRENT_MAX)
#define NipMaxCurrent            (C620_CURRENT_MAX)

#define MIN2(a,b)  (a<b?a:b)

typedef struct 
{
	float m_fpDes;//���Ʊ���Ŀ��ֵ  
	float m_fpFB;//���Ʊ�������ֵ   
	
	float m_fpKp;//����ϵ��Kp      
	float m_fpKi;//����ϵ��Ki      
	float m_fpKd;//΢��ϵ��Kd 
	
	float m_fpUp;//�������
	float m_fpUi;//�������
	float m_fpUd;//΢�����
	
	float m_fpE;//����ƫ��         
	float m_fpPreE;//�ϴ�ƫ��      
	float m_fpSumE;//��ƫ��  
	
	float m_fpU;//����PID������  
	
	float m_fpUMax;//PID�����������ֵ������������ʱ������ֵ  
	float m_fpUpMax;//�������������  
	float m_fpEiMax;//��������   
	float m_fpUdMax;//΢�����������

	float m_fpEMin;
}ST_PID;

void SetDes(ST_PID* pStPID, s32 Des);
bool Is_Float_Equal(float a,float b);
float Clip(float fpValue, float fpMin, float fpMax);
void CalIResistedPID(ST_PID* pStPID);
void CalIWeakenPID(ST_PID* pStPID);
float Angle_Inf_To_360(float angle);
float Angle_Inf_To_180(float angle);

#endif

