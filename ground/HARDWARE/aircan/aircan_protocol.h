#ifndef __AIRCAN_PROTOCOL_H__
#define __AIRCAN_PROTOCOL_H__

#include "stdbool.h"

#define CAN_AIR_ID 	  0x30	//气缸控制ID
#define CANTXKEYID     0x20	//光电门ID
#define PHTOTGATE_GBR_ID     0x40	//光电门ID
#define MAGNETOMETER_ID      0x08	//磁力计ID
#define MICROSWITCH_ID       0x10   //微动开关ID
#define OPEN_CYLINDER(channel)    ( g_uiAirValve |= (0x01 << (channel-1)) ) //气缸开启
#define CLOSE_CYLINDER(channel)   ( g_uiAirValve &= ( ~(0x01 << (channel-1)) ) ) //气缸关闭

void SendAirMsgByCan1(unsigned int* pAir);
void SendAirMsgByCan2(unsigned int* pAir);
void Air_Test(unsigned short number, bool OpenOrClose);


#endif




