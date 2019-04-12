#include "net_protocol.h"
#include "rtc.h"
#include "usart.h"
#include "24cxx.h"
#include "common.h"
#include "task_net.h"
#include "error.h"
#include "utils.h"


//读取/处理网络数据
u16 time_out = 0;
s16 NetDataFrameHandle(u8 *outbuf,u8 *hold_reg)
{
	u16 i = 0;
	s16 ret = 0;
	u16 len = 0;
	u8 buf[1024];
	u8 *msg = NULL;
	char tmp[10];

	memset(buf,0,1024);
	
	ret = nbiot_udp_recv(buf,(size_t *)&len);
	
	if(ret == NBIOT_ERR_OK)
	{
		if(len != 0)
		{
			time_out = 0;
			
			msg = (u8 *)strstr((char *)buf,",");
			
			if(msg == NULL)
				return ret;

			msg = msg + 1;
			
			memset(tmp,0,10);
			
			while(*msg != ',')
				tmp[i ++] = *(msg ++);
			
			tmp[i] = '\0';
			len = nbiot_atoi(tmp,strlen(tmp));
			
			msg = msg + 1;
			
			StrToHex(msg,(char *)msg,len);

			ret = (s16)NetDataAnalysis(msg,len,outbuf,hold_reg);
		}
		else
		{
			time_out ++;

			if(time_out >= 600)		//一分钟内未收到任何数据，强行关闭连接
			{
				time_out = 0;

				ret = -1;
			}
		}
	}

	return ret;
}

//网络数据帧协议解析
u16 NetDataAnalysis(u8 *buf,u16 len,u8 *outbuf,u8 *hold_reg)
{
	u16 ret = 0;
	u8 cmd_code = 0;
	u16 data_len = 0;
	u8 *data = NULL;
	u8 read_check_sum = 0;
	u8 cal_check_sum = 0;
	u8 msg_tail[6] = {0xFE,0xFD,0xFC,0xFB,0xFA,0xF9};

	if(MyStrstr(buf,msg_tail,len,6) == 0xFFFF)
	{
		return ret;
	}
	
	if(*(buf + 0) != 0x68 ||
	   *(buf + 7) != 0x68 ||
	   *(buf + len - 7) != 0x16)
	{
		return ret;
	}
	
	if(MyStrstr(buf,msg_tail,len,6) == 0xFFFF)
	{
		return ret;
	}
	
	if(DeviceID == NULL || DeviceUUID == NULL)
	{
		return ret;
	}
	
	if(MyStrstr(buf,DeviceID,len,DEVICE_ID_LEN - 2) == 0xFFFF)
	{
		return ret;
	}
	
	if(MyStrstr(buf,DeviceUUID,len,UU_ID_LEN - 2) == 0xFFFF)
	{
		return ret;
	}
	
	cmd_code = *(buf + 25);
	data_len = *(buf + 26);
	read_check_sum = *(buf + len - 8);
	
	cal_check_sum = CalCheckSum(buf, len - 8);
	
	if(read_check_sum != cal_check_sum)
	{
		return ret;
	}
	
	data = buf + 27;

	switch(cmd_code)
	{
		case 0xE0:			//读取设备运行参数 保留
			
		break;

		case 0xE1:			//上送心跳 保留
			
		break;

		case 0xE2:			//调光
			ret = ControlLightLevel(cmd_code,data,data_len,outbuf);
		break;

		case 0xE3:			//OTA
			ret = SetUpdateFirmWareInfo(cmd_code,data,data_len,outbuf);
		break;

		case 0xE4:			//请求固件包 保留
			
		break;

		case 0xE5:			//重启
			ret = ControlDeviceReset(cmd_code,data,data_len,outbuf);
		break;

		case 0xE6:			//设置数据上传间隔
			ret = SetDeviceUpLoadINCL(cmd_code,data,data_len,outbuf);
		break;

		case 0xE7:			//设置亮灭灯策略
			ret = SetRegularTimeGroups(cmd_code,data,data_len,outbuf);
		break;

		case 0xE8:			//读取设备信息，软件版本等
			ret = GetDeviceInfo(cmd_code,data,data_len,outbuf);
		break;
		
		case 0xE9:			//设置设备工作模式
			ret = SetDeviceWorkMode(cmd_code,data,data_len,outbuf);
		break;
		
		case 0xEA:			//请求IMEI(UUID)
			ret = SetDeviceUUID(cmd_code,data,data_len,outbuf);
		break;
		
		case 0xEB:			//请求对时
			ret = GetTimeDateFromServer(cmd_code,data,data_len,outbuf);
		break;
		
		case 0xEC:			//设置电源接口
			ret = SetDevicePowerINTFC(cmd_code,data,data_len,outbuf);
		break;
		
		case 0x80:			//应答
			ret = UnPackAckPacket(cmd_code,data,data_len);
		break;

		default:

		break;
	}

	return ret;
}

//解析ACK包
u8 UnPackAckPacket(u8 cmd_code,u8 *buf,u8 len)
{
	u8 ret = 0;

	if(len == 2)
	{
		if(*(buf + 1) == 0)
		{
			ret = 1;
		}
	}

	return ret;
}

//ACK打包
u16 PackAckPacket(u8 cmd_code,u8 *data,u8 *outbuf)
{
	u16 len = 0;

	len = PackNetData(cmd_code,data,2,outbuf);

	return len;
}

//调光
u16 ControlLightLevel(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 level = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 1)
	{
		level = *(buf + 0);

		if(level <= 100)
		{
			LightLevelPercent = 2 * level;

			DeviceWorkMode = MODE_MANUAL;		//强制转换为手动模式
			
			memcpy(&HoldReg[LIGHT_LEVEL_ADD],&LightLevelPercent,LIGHT_LEVEL_LEN - 2);
			WriteDataFromHoldBufToEeprom(&HoldReg[LIGHT_LEVEL_ADD],LIGHT_LEVEL_ADD, LIGHT_LEVEL_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//下发更新固件命令
u16 SetUpdateFirmWareInfo(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 5)
	{
		NewFirmWareVer    = (((u16)(*(buf + 0))) << 8) + (u16)(*(buf + 1));
		NewFirmWareBagNum = (((u16)(*(buf + 2))) << 8) + (u16)(*(buf + 3));
		LastBagByteNum    = *(buf + 4);
	
		if(NewFirmWareBagNum == 0 || NewFirmWareBagNum > MAX_FW_BAG_NUM \
			|| NewFirmWareVer == 0 || NewFirmWareVer > MAX_FW_VER \
			|| LastBagByteNum == 0 || LastBagByteNum > MAX_FW_LAST_BAG_NUM)  //128 + 2 + 4 = 134
		{
			data_buf[1] = 1;
		}
		else
		{
			HaveNewFirmWare = 0xAA;
			if(NewFirmWareAdd == 0xAA)
			{
				NewFirmWareAdd = 0x55;
			}
			else if(NewFirmWareAdd == 0x55)
			{
				NewFirmWareAdd = 0xAA;
			}
			else
			{
				NewFirmWareAdd = 0xAA;
			}
			
			WriteOTAInfo(HoldReg,0);		//将数据写入EEPROM
			
			NeedToReset = 1;				//重新启动
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//远程重启
u16 ControlDeviceReset(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 0)
	{
		NeedToReset = 1;
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//设置设备数据上传时间间隔
u16 SetDeviceUpLoadINCL(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u16 incl;

	data_buf[0] = cmd_code;

	if(len == 2)												//数据长度必须是64
	{
		incl = ((u16)(*(buf + 0)) << 16) + (*(buf + 1));
		
		if(incl <= MAX_UPLOAD_INVL)
		{
			UpLoadINCL = incl;
			
			memcpy(&HoldReg[UPLOAD_INVL_ADD],buf,2);
			WriteDataFromHoldBufToEeprom(&HoldReg[UPLOAD_INVL_ADD],UPLOAD_INVL_ADD, UPLOAD_INVL_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//设置策略时间
u16 SetRegularTimeGroups(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 group_num = 0;
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	u8 data_buf[2] = {0,0};
	u8 time_group[256];
	u16 crc16 = 0;

	data_buf[0] = cmd_code;

	if(len % 7 == 0)							//数据长度必须是10的倍数
	{
		group_num = len / 7;					//计算下发了几组数据

		if(group_num <= MAX_GROUP_NUM)			//组数必须是2的倍数，并且要小于MAX_GROUP_NUM
		{
			RemoveAllStrategy();				//删除所有本地存储策略
			
			TimeGroupNumber = group_num;

			crc16 = CRC16(&group_num,1);

			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 0,TimeGroupNumber);
			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 1,(u8)(crc16 >> 8));
			AT24CXX_WriteOneByte(TIME_GROUP_NUM_ADD + 2,(u8)(crc16 & 0x00FF));

			memset(time_group,0,256);

			for(i = 0; i < group_num; i ++)
			{
				for(j = i * TIME_RULE_LEN; j < i * TIME_RULE_LEN + 7; j ++, k ++)
				{
					time_group[j] = *(buf + k);
				}

				crc16 = CRC16(&time_group[j - 7],7);

				time_group[j + 0] = (u8)(crc16 >> 8);
				time_group[j + 1] = (u8)(crc16 & 0x00FF);
			}

			for(i = 0; i < group_num; i ++)
			{
				pRegularTime tmp_time = NULL;
				
				tmp_time = (pRegularTime)mymalloc(sizeof(RegularTime_S));

				tmp_time->prev = NULL;
				tmp_time->next = NULL;
				
				tmp_time->number 		= i;
				tmp_time->type 			= time_group[i * TIME_RULE_LEN + 0];
				tmp_time->year 			= time_group[i * TIME_RULE_LEN + 1];
				tmp_time->month 		= time_group[i * TIME_RULE_LEN + 2];
				tmp_time->date 			= time_group[i * TIME_RULE_LEN + 3];
				tmp_time->hour 			= time_group[i * TIME_RULE_LEN + 4];
				tmp_time->minute 		= time_group[i * TIME_RULE_LEN + 5];
				tmp_time->percent		= time_group[i * TIME_RULE_LEN + 6];
				
				switch(tmp_time->type)
				{
					case TYPE_WEEKDAY:
						RegularTimeGroupAdd(TYPE_WEEKDAY,tmp_time);
					break;
					
					case TYPE_WEEKEND:
						RegularTimeGroupAdd(TYPE_WEEKEND,tmp_time);
					break;
					
					case TYPE_HOLIDAY:
						RegularTimeGroupAdd(TYPE_HOLIDAY,tmp_time);
					break;
					
					default:
						
					break;
				}
			}

			for(i = 0; i < group_num * TIME_RULE_LEN + group_num; i ++)				//每组7个字节+2个字节(CRC16)
			{
				AT24CXX_WriteOneByte(TIME_RULE_ADD + i,time_group[i]);
			}
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//读取设备信息
u16 GetDeviceInfo(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 code = 0;
	u8 data_buf[2] = {0,0};

	if(len == 0)
	{
		code = cmd_code;
		
		data_buf[0] = (u8)((u16)SOFT_WARE_VRESION >> 8);
		data_buf[1] = (u8)((u16)SOFT_WARE_VRESION & 0x00FF);
	}
	else
	{
		data_buf[0] = cmd_code;
		data_buf[1] = 2;
		code = 0x80;
	}

	out_len = PackAckPacket(code,data_buf,outbuf);

	return out_len;
}

//设置设备的工作模式
u16 SetDeviceWorkMode(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 mode = 0;
	u8 data_buf[2] = {0,0};
	data_buf[0] = cmd_code;

	if(len == 1)
	{
		mode = *(buf + 0);

		if(mode == 0 || mode == 1)
		{
			DeviceWorkMode = mode;			//置工作模式标志

		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(0x80,data_buf,outbuf);

	return out_len;
}

//设置设备的UUID
u16 SetDeviceUUID(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u8 uuid_buf[UU_ID_LEN];

	data_buf[0] = cmd_code;

	if(len == UU_ID_LEN - 2)												//数据长度必须是64
	{
		memset(uuid_buf,0,UU_ID_LEN);

		memcpy(&HoldReg[UU_ID_ADD],buf,UU_ID_LEN - 2);
		
		GetDeviceUUID();

		WriteDataFromHoldBufToEeprom(&HoldReg[UU_ID_ADD],UU_ID_ADD, UU_ID_LEN - 2);
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(cmd_code,data_buf,outbuf);

	return out_len;
}

//从服务器获取时间戳
u16 GetTimeDateFromServer(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u8 year = 0;
	u8 mon = 0;
	u8 day = 0;
	u8 hour = 0;
	u8 min = 0;
	u8 sec = 0;

	data_buf[0] = cmd_code;

	if(len == 6)												//数据长度必须是64
	{
		year = *(buf + 0);
		mon  = *(buf + 1);
		day  = *(buf + 2);
		hour = *(buf + 3);
		min  = *(buf + 4);
		sec  = *(buf + 5);
		
		if(year >= 18 && mon <= 12 && day <= 31 && hour <= 23 && min <= 59 && sec <= 59)
		{
			RTC_Set(year + 2000,mon,day,hour,min,sec);
			
			GetTimeOK = 2;
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(cmd_code,data_buf,outbuf);

	return out_len;
}

//设置电源控制方式
u16 SetDevicePowerINTFC(u8 cmd_code,u8 *buf,u8 len,u8 *outbuf)
{
	u8 out_len = 0;
	u8 data_buf[2] = {0,0};
	u16 intfc;

	data_buf[0] = cmd_code;

	if(len == 1)												//数据长度必须是64
	{
		intfc = *(buf + 0);
		
		if(intfc <= 3)
		{
			PowerINTFC = intfc;
			
			memcpy(&HoldReg[POWER_INTFC_ADD],&intfc,1);
			WriteDataFromHoldBufToEeprom(&HoldReg[POWER_INTFC_ADD],POWER_INTFC_ADD, POWER_INTFC_LEN - 2);
		}
		else
		{
			data_buf[1] = 1;
		}
	}
	else
	{
		data_buf[1] = 2;
	}

	out_len = PackAckPacket(cmd_code,data_buf,outbuf);

	return out_len;
}


























