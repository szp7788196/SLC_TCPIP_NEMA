#include "task_net.h"
#include "common.h"
#include "delay.h"
#include "net_protocol.h"
#include "rtc.h"


TaskHandle_t xHandleTaskNET = NULL;
SensorMsg_S *p_tSensorMsgNet = NULL;			//用于装在传感器数据的结构体变量


CONNECT_STATE_E ConnectState = UNKNOW_STATE;
u8 SocketId = 255;
u8 SendDeviceUUID_OK = 0;						//发送UUID成功标识
u8 SignalIntensity = 99;						//bg96的信号强度


unsigned portBASE_TYPE NET_Satck;
void vTaskNET(void *pvParameters)
{
	s8 ret = 0;
	time_t times_sec = 0;

	p_tSensorMsgNet = (SensorMsg_S *)mymalloc(sizeof(SensorMsg_S));

	bcxx_hard_init();

	RE_INIT:
	bcxx_soft_init();

	ConnectState = UNKNOW_STATE;

	while(1)
	{
		if(GetSysTick1s() - times_sec >= UpLoadINCL)			//每隔10秒钟获取一次信号强度
		{
			times_sec = GetSysTick1s();

			SignalIntensity = bcxx_get_AT_CSQ();
			SyncDataTimeFormBcxxModule(3600);
		}

		switch(ConnectState)
		{
			case UNKNOW_STATE:
				ret = bcxx_get_AT_CGPADDR((char **)&LocalIp);

				if(ret == 1)
				{
					ConnectState = GET_READY;
				}
				else
				{
					goto RE_INIT;
				}
			break;

			case GET_READY:
				ret = TryToConnectToServer();

				if(ret == 1)
				{
					ConnectState = ON_SERVER;
				}
				else
				{
					goto RE_INIT;
				}
			break;

			case ON_SERVER:
				ret = OnServerHandle();

				if(ret != 1)
				{
					goto RE_INIT;
				}
			break;

			case DISCONNECT:
				goto RE_INIT;
//			break;

			default:

			break;
		}

		delay_ms(100);
	}
}

u8 TryToConnectToServer(void)
{
	u8 ret = 0;

	if(ServerIP != NULL && ServerPort != NULL)
	{
		SocketId = bcxx_set_AT_NSOCR("STREAM", "6","0");

		if(SocketId <= 7)
		{
			ret = bcxx_set_AT_NSOCO(SocketId, (char *)ServerIP,(char *)ServerPort);
		}
	}

	return ret;
}

//在线处理进程
s8 OnServerHandle(void)
{
	s8 ret = 1;
	s16 len = 0;
	u8 out_buf[512];

	ret = SendSensorDataToIOTPlatform();							//向服务器定时发送传感器数据和心跳包

	if(ret == 2)
	{
		return ret;
	}

	len = NetDataFrameHandle(out_buf,HoldReg);					//读取并解析服务器下发的数据包

	if(len >= 1)
	{
		ret = bcxx_set_AT_NSOSD(SocketId, len,(char *)out_buf);
	}

	return ret;
}

//向服务器定时发送传感器数据和心跳包
s8 SendSensorDataToIOTPlatform(void)
{
	BaseType_t xResult;
	s8 ret = 1;
	u8 send_len = 0;
	u8 sensor_data_len = 0;

	u8 sensor_buf[128];
	u8 send_buf[512];

	xResult = xQueueReceive(xQueue_sensor,
							(void *)p_tSensorMsgNet,
							(TickType_t)pdMS_TO_TICKS(50));
	if(xResult == pdPASS)
	{
		memset(send_buf,0,512);
		memset(sensor_buf,0,128);

		sensor_data_len = UnPackSensorData(p_tSensorMsgNet,sensor_buf);
		send_len = PackNetData(0xE0,sensor_buf,sensor_data_len,send_buf);
	}

	if(send_len >= 1)
	{
		ret = bcxx_set_AT_NSOSD(SocketId, send_len,(char *)send_buf);
	}

	return ret;
}


//从指定的NTP服务器获取时间
u8 SyncDataTimeFormBcxxModule(time_t sync_cycle)
{
	u8 ret = 0;
	struct tm tm_time;
	static time_t time_s = 0;
	char buf[32];

	if((GetSysTick1s() - time_s >= sync_cycle))
	{
		memset(buf,0,32);

		if(bcxx_get_AT_CCLK(buf))
		{
			tm_time.tm_year = 2000 + (buf[0] - 0x30) * 10 + buf[1] - 0x30 - 1900;
			tm_time.tm_mon = (buf[3] - 0x30) * 10 + buf[4] - 0x30 - 1;
			tm_time.tm_mday = (buf[6] - 0x30) * 10 + buf[7] - 0x30;

			tm_time.tm_hour = (buf[9] - 0x30) * 10 + buf[10] - 0x30;
			tm_time.tm_min = (buf[12] - 0x30) * 10 + buf[13] - 0x30;
			tm_time.tm_sec = (buf[15] - 0x30) * 10 + buf[16] - 0x30;

			time_s = mktime(&tm_time);

			time_s += 28800;

			SyncTimeFromNet(time_s);

			GetTimeOK = 1;

			ret = 1;
		}
	}

	return ret;
}






























