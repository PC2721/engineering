#include "aircan_protocol.h"
#include "can.h"
#include "rm_data_types.h"
#include "global_declare.h"

/*--------------------------------------------------------------------------------------------------------------------
������:SendAirMsgByCan1()
��������: ͨ��CAN1�������׵ı���
����: pAir Ҫ���Ƶ�����ID
�������
--------------------------------------------------------------------------------------------------------------------*/
void SendAirMsgByCan1(unsigned int* pAir)
{
	static CanTxMsg TxMessage;
	TxMessage.StdId = CAN_AIR_ID;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x04;
	
	static unsigned int s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
//	*((unsigned int*)TxMessage.Data) = s_ucLastAir;
	
	TxMessage.Data[0] = (uint8_t)(s_ucLastAir);
	TxMessage.Data[1] = (uint8_t)(s_ucLastAir>>8);
	TxMessage.Data[2] = (uint8_t)(s_ucLastAir>>16);
	TxMessage.Data[3] = (uint8_t)(s_ucLastAir>>24);
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	CAN_Transmit(CAN1,&TxMessage);
}

/*--------------------------------------------------------------------------------------------------------------------
������:SendAirMsgByCan2()
��������: ͨ��CAN2�������׵ı���
����: pAir Ҫ���Ƶ�����ID
�������
--------------------------------------------------------------------------------------------------------------------*/
void SendAirMsgByCan2(unsigned int* pAir)
{
	static CanTxMsg TxMessage;
	TxMessage.StdId = CAN_AIR_ID;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x04;
	
	static unsigned int s_ucLastAir = 0;	
	s_ucLastAir = *pAir;
//	*((unsigned int*)TxMessage.Data) = s_ucLastAir;
	
	TxMessage.Data[0] = (uint8_t)(s_ucLastAir);
	TxMessage.Data[1] = (uint8_t)(s_ucLastAir>>8);
	TxMessage.Data[2] = (uint8_t)(s_ucLastAir>>16);
	TxMessage.Data[3] = (uint8_t)(s_ucLastAir>>24);
	TxMessage.Data[4] = 0;
	TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;
	TxMessage.Data[7] = 0;
	CAN_Transmit(CAN2,&TxMessage);
}


void Air_Test(unsigned short number, bool OpenOrClose)
{
	if(OpenOrClose==true)
		OPEN_CYLINDER(number);
	if(OpenOrClose==false)
		CLOSE_CYLINDER(number);
}
