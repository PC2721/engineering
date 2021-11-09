#include "pid.h"
void SetDes(ST_PID* pStPID, s32 Des)
{
	pStPID->m_fpDes = Des;
}
void CalIResistedPID(ST_PID* pStPID)
{
	pStPID->m_fpE = pStPID->m_fpDes - pStPID->m_fpFB;  //??????
	pStPID->m_fpSumE += pStPID->m_fpE;  //??????
	
	pStPID->m_fpSumE = Clip(pStPID->m_fpSumE, -pStPID->m_fpEiMax, pStPID->m_fpEiMax);
	pStPID->m_fpUi = pStPID->m_fpKi * pStPID->m_fpSumE;
	
	pStPID->m_fpUp = Clip(pStPID->m_fpKp * pStPID->m_fpE, -pStPID->m_fpUpMax, pStPID->m_fpUpMax);
	pStPID->m_fpUd = Clip(pStPID->m_fpKd * (pStPID->m_fpE - pStPID->m_fpPreE), -pStPID->m_fpUdMax, pStPID->m_fpUdMax);
	
	/*????????,????????*/
	if(fabs(pStPID->m_fpE) < pStPID->m_fpEMin)
	{
		pStPID->m_fpSumE = 0;  //??????
	}
	
	/*???PID????*/
	pStPID->m_fpU = pStPID->m_fpUp + pStPID->m_fpUi + pStPID->m_fpUd;
	
	pStPID->m_fpPreE = pStPID->m_fpE; //??????
	/*PID??????*/
	pStPID->m_fpU = Clip(pStPID->m_fpU , -pStPID->m_fpUMax, pStPID->m_fpUMax);
}

//?????PID??????PID?
void CalIWeakenPID(ST_PID* pStPID)
{
	pStPID->m_fpE = pStPID->m_fpDes - pStPID->m_fpFB;  //??????
	
	if(fabs(pStPID->m_fpE) < pStPID->m_fpEMin)
	{
		pStPID->m_fpSumE += pStPID->m_fpE;  //??????
	}
	
	pStPID->m_fpSumE = Clip(pStPID->m_fpSumE, -pStPID->m_fpEiMax, pStPID->m_fpEiMax);
	
	pStPID->m_fpUp = Clip(pStPID->m_fpKp * pStPID->m_fpE, -pStPID->m_fpUpMax, pStPID->m_fpUpMax);  //??P???
	pStPID->m_fpUi = pStPID->m_fpKi * pStPID->m_fpSumE;  //??I???
	pStPID->m_fpUd = Clip(pStPID->m_fpKd * (pStPID->m_fpE - pStPID->m_fpPreE), -pStPID->m_fpUdMax, pStPID->m_fpUdMax); //??D???
	
	pStPID->m_fpU = pStPID->m_fpUp + pStPID->m_fpUi + pStPID->m_fpUd; //?????
	
	pStPID->m_fpPreE = pStPID->m_fpE; //??????
	/*PID??????*/
	pStPID->m_fpU = Clip(pStPID->m_fpU , -pStPID->m_fpUMax, pStPID->m_fpUMax);
}
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