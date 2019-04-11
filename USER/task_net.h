#ifndef __TASK_NET_H
#define __TASK_NET_H

#include "sys.h"
#include "common.h"
#include "rtos_task.h"
#include "bcxx.h"
#include "task_sensor.h"


#define MAX_NTPSERVER_NUM	28

typedef struct 
{
	char ip_adderss[16];
}NTPServerTypeDef;


extern TaskHandle_t xHandleTaskNET;
extern SensorMsg_S *p_tSensorMsgNet;

extern CONNECT_STATE_E ConnectState;
extern u8 SendDeviceUUID_OK;			//发送UUID成功标识
extern u8 SignalIntensity;				//bg96的信号强度


void vTaskNET(void *pvParameters);
u8 TryToConnectToServer(void);
s8 OnServerHandle(void);
s8 SendSensorDataToIOTPlatform(void);
u8 SyncDataTimeFormBcxxModule(time_t sync_cycle);































#endif
