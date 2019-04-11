#ifndef __NET_PROTOCOL_H
#define __NET_PROTOCOL_H

#include "sys.h"
#include "bcxx.h"
#include "common.h"

/******************************************************************************************
/                                    通讯错误码
/	
/	
/	
/	
/	
/	
/	
/
******************************************************************************************/
s16 NetDataFrameHandle(u8 *outbuf,u8 *hold_reg);
u16 NetDataAnalysis(u8 *buf,u16 len,u8 *outbuf,u8 *hold_reg);

u8 UnPackAckPacket(u8 cmd_code,u8 *buf,u8 len);
u16 PackAckPacket(u8 cmd_code,u8 *data,u8 *outbuf);
u16 ControlLightLevel(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetUpdateFirmWareInfo(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 ControlDeviceReset(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetDeviceUpLoadINCL(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetRegularTimeGroups(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 GetDeviceInfo(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetDeviceWorkMode(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetDeviceUUID(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 GetTimeDateFromServer(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);
u16 SetDevicePowerINTFC(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf);




































#endif
