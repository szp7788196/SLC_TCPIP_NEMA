#include "common.h"
#include "24cxx.h"
#include "bcxx.h"


u8 HoldReg[HOLD_REG_LEN];						//保持寄存器
u8 RegularTimeGroups[TIME_BUF_LEN];				//时间策略缓存
u8 TimeGroupNumber = 0;							//时间策略组数
pRegularTime RegularTimeWeekDay = NULL;			//工作日策略
pRegularTime RegularTimeWeekEnd = NULL;			//周末策略
pRegularTime RegularTimeHoliday = NULL;			//节假日策略
HolodayRange_S HolodayRange;					//节假日起始日期

/****************************互斥量相关******************************/
SemaphoreHandle_t  xMutex_IIC1 			= NULL;	//IIC总线1的互斥量
SemaphoreHandle_t  xMutex_INVENTR 		= NULL;	//英飞特电源的互斥量
SemaphoreHandle_t  xMutex_AT_COMMAND 	= NULL;	//AT指令的互斥量
SemaphoreHandle_t  xMutex_STRATEGY 		= NULL;	//AT指令的互斥量

/***************************消息队列相关*****************************/
QueueHandle_t xQueue_sensor 		= NULL;	//用于存储传感器的数据


/***************************固件升级相关*****************************/
u8 NeedUpDateFirmWare = 0;			//有新固件需要加载
u8 HaveNewFirmWare = 0;				//0xAA有新固件 others无新固件
u8 NewFirmWareAdd = 0;				//0xAA新固件地址0x0800C000 0x55新固件地址0x08026000
u16 NewFirmWareBagNum = 0;			//固件包的数量（一个固件包含多个小包）
u16 NewFirmWareVer = 1;				//固件包的版本
u8 LastBagByteNum = 0;				//最后一包的字节数

/***************************系统心跳相关*****************************/
u32 SysTick1ms = 0;					//1ms滴答时钟
u32 SysTick10ms = 0;				//10ms滴答时钟
u32 SysTick100ms = 0;				//10ms滴答时钟
time_t SysTick1s = 0;				//1s滴答时钟

/***************************版本相关*********************************/
u8 *BootLoaderVersion = NULL;		//BootLoader版本号
u8 *SoftWareVersion = NULL;			//应用程序版本号
u8 *HardWareVersion = NULL;			//硬件版本号

/***************************设备相关*********************************/
u8 *DeviceName = NULL;				//设备名称
u8 *DeviceID = NULL;				//设备ID
u8 *DeviceUUID = NULL;				//设备UUID
u8 *DeviceICCID = NULL;				//ICCID
u8 *DeviceIMSI = NULL;				//IMSI

/***************************网络相关*********************************/
u8 Operators = 1;					//运营商编号 0:移动 1:联通 2:电信
u8 *APName = NULL;					//私有APN，不同客户APN不同
u8 *ServerDomain = NULL;			//服务器域名
u8 *ServerIP = NULL;				//服务器IP地址
u8 *ServerPort = NULL;				//服务器端口号
u8 *LocalIp = NULL;					//本地IP地址

/***************************运行参数相关*****************************/
u16 UpLoadINCL = 20;				//数据上传时间间隔0~65535秒
u8 PowerINTFC = 0;					//电源控制接口编号 0:0~10V  1:PWM  2:UART
u8 TimeZone = 8;					//时区偏移量

/***************************其他*****************************/
u8 LightLevelPercent = 200;			//灯的亮度级别
u8 NeedToReset = 0;					//复位/重启标志
u8 GetGPSOK = 0;					//成功获取位置信息和时间标志
u8 GetTimeOK = 0;					//成功获取时间标志
u8 DeviceWorkMode = 0;				//运行模式，0：自动，1：手动

u8 *GpsInfo = NULL;					//设备的位置信息


//在str1中查找str2，失败返回0xFF,成功返回str2首个元素在str1中的位置
u16 MyStrstr(u8 *str1, u8 *str2, u16 str1_len, u16 str2_len)
{
	u16 len = str1_len;
	if(str1_len == 0 || str2_len == 0)
	{
		return 0xFFFF;
	}
	else
	{
		while(str1_len >= str2_len)
		{
			str1_len --;
			if (!memcmp(str1, str2, str2_len))
			{
				return len - str1_len - 1;
			}
			str1 ++;
		}
		return 0xFFFF;
	}
}

//获得整数的位数
u8 GetDatBit(u32 dat)
{
	u8 j = 1;
	u32 i;
	i = dat;
	while(i >= 10)
	{
		j ++;
		i /= 10;
	}
	return j;
}

//用个位数换算出一个整数 1 10 100 1000......
u32 GetADV(u8 len)
{
	u32 count = 1;
	if(len == 1)
	{
		return 1;
	}
	else
	{
		len --;
		while(len --)
		{
			count *= 10;
		}
	}
	return count;
}

//整数转换为字符串
void IntToString(u8 *DString,u32 Dint,u8 zero_num)
{
	u16 i = 0;
	u8 j = GetDatBit(Dint);
	for(i = 0; i < GetDatBit(Dint) + zero_num; i ++)
	{
		DString[i] = Dint / GetADV(j) % 10 + 0x30;
		j --;
	}
}

u32 StringToInt(u8 *String)
{
	u8 len;
	u8 i;
	u32 count=0;
	u32 dev;

	len = strlen((char *)String);
	dev = 1;
	for(i = 0; i < len; i ++)//len-1
	{
		if(String[i] != '.')
		{
			count += ((String[i] - 0x30) * GetADV(len) / dev);
			dev *= 10;
		}
		else
		{
			len --;
			count /= 10;
		}
	}
	if(String[i]!=0x00)
	{
		count += (String[i] - 0x30);
	}
	return count;
}

/*
// C prototype : void HexToStr(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 存放目标字符串
// [IN] pbSrc - 输入16进制数的起始地址
// [IN] nLen - 16进制数的字节数
// return value:
// remarks : 将16进制数转化为字符串
*/
void HexToStr(char *pbDest, u8 *pbSrc, u16 len)
{
	char ddl,ddh;
	int i;

	for (i = 0; i < len; i ++)
	{
		ddh = 48 + pbSrc[i] / 16;
		ddl = 48 + pbSrc[i] % 16;
		if (ddh > 57) ddh = ddh + 7;
		if (ddl > 57) ddl = ddl + 7;
		pbDest[i * 2] = ddh;
		pbDest[i * 2 + 1] = ddl;
	}

	pbDest[len * 2] = '\0';
}

/*
// C prototype : void StrToHex(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 输出缓冲区
// [IN] pbSrc - 字符串
// [IN] nLen - 16进制数的字节数(字符串的长度/2)
// return value:
// remarks : 将字符串转化为16进制数
*/
void StrToHex(u8 *pbDest, char *pbSrc, u16 len)
{
	char h1,h2;
	u8 s1,s2;
	int i;

	for (i = 0; i < len; i ++)
	{
		h1 = pbSrc[2 * i];
		h2 = pbSrc[2 * i + 1];

		s1 = toupper(h1) - 0x30;
		if (s1 > 9)
		s1 -= 7;

		s2 = toupper(h2) - 0x30;
		if (s2 > 9)
		s2 -= 7;

		pbDest[i] = s1 * 16 + s2;
	}
}

unsigned short find_str(unsigned char *s_str, unsigned char *p_str, unsigned short count, unsigned short *seek)
{
	unsigned short _count = 1;
    unsigned short len = 0;
    unsigned char *temp_str = NULL;
    unsigned char *temp_ptr = NULL;
    unsigned char *temp_char = NULL;

	(*seek) = 0;
    if(0 == s_str || 0 == p_str)
        return 0;
    for(temp_str = s_str; *temp_str != '\0'; temp_str++)
    {
        temp_char = temp_str;

        for(temp_ptr = p_str; *temp_ptr != '\0'; temp_ptr++)
        {
            if(*temp_ptr != *temp_char)
            {
                len = 0;
                break;
            }
            temp_char++;
            len++;
        }
        if(*temp_ptr == '\0')
        {
            if(_count == count)
                return len;
            else
            {
                _count++;
                len = 0;
            }
        }
        (*seek) ++;
    }
    return 0;
}

int search_str(unsigned char *source, unsigned char *target)
{
	unsigned short seek = 0;
    unsigned short len;
    len = find_str(source, target, 1, &seek);
    if(len == 0)
        return -1;
    else
        return len;
}

unsigned short get_str1(unsigned char *source, unsigned char *begin, unsigned short count1, unsigned char *end, unsigned short count2, unsigned char *out)
{
	unsigned short i;
    unsigned short len1;
    unsigned short len2;
    unsigned short index1 = 0;
    unsigned short index2 = 0;
    unsigned short length = 0;
    len1 = find_str(source, begin, count1, &index1);
    len2 = find_str(source, end, count2, &index2);
    length = index2 - index1 - len1;
    if((len1 != 0) && (len2 != 0))
    {
        for( i = 0; i < index2 - index1 - len1; i++)
            out[i] = source[index1 + len1 + i];
        out[i] = '\0';
    }
    else
    {
        out[0] = '\0';
    }
    return length;
}

unsigned short get_str2(unsigned char *source, unsigned char *begin, unsigned short count, unsigned short length, unsigned char *out)
{
	unsigned short i = 0;
    unsigned short len1 = 0;
    unsigned short index1 = 0;
    len1 = find_str(source, begin, count, &index1);
    if(len1 != 0)
    {
        for(i = 0; i < length; i++)
            out[i] = source[index1 + len1 + i];
        out[i] = '\0';
    }
    else
    {
        out[0] = '\0';
    }
    return length;
}

unsigned short get_str3(unsigned char *source, unsigned char *out, unsigned short length)
{
	unsigned short i = 0;
    for (i = 0 ; i < length ; i++)
    {
        out[i] = source[i];
    }
    out[i] = '\0';
    return length;
}

//32位CRC校验
u32 CRC32( const u8 *buf, u32 size)
{
     uint32_t i, crc;
     crc = 0xFFFFFFFF;
     for (i = 0; i < size; i++)
      crc = crc32tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
     return crc^0xFFFFFFFF;
}

/*****************************************************
函数：u16 CRC16(u8 *puchMsgg,u8 usDataLen)
功能：CRC校验用函数
参数：puchMsgg是要进行CRC校验的消息，usDataLen是消息中字节数
返回：计算出来的CRC校验码。
*****************************************************/
u16 CRC16(u8 *puchMsgg,u8 usDataLen)
{
    u8 uchCRCHi = 0xFF ; 											//高CRC字节初始化
    u8 uchCRCLo = 0xFF ; 											//低CRC 字节初始化
    u8 uIndex ; 													//CRC循环中的索引
    while (usDataLen--) 											//传输消息缓冲区
    {
		uIndex = uchCRCHi ^ *puchMsgg++; 							//计算CRC
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
    }
    return ((uchCRCHi << 8) | uchCRCLo);
}

//计算校验和
u8 CalCheckSum(u8 *buf, u16 len)
{
	u8 sum = 0;
	u16 i = 0;

	for(i = 0; i < len; i ++)
	{
		sum += *(buf + i);
	}

	return sum;
}

//产生一个系统1毫秒滴答时钟.
void SysTick1msAdder(void)
{
	SysTick1ms = (SysTick1ms + 1) & 0xFFFFFFFF;
}

//获取系统1毫秒滴答时钟
u32 GetSysTick1ms(void)
{
	return SysTick1ms;
}

//产生一个系统10毫秒滴答时钟.
void SysTick10msAdder(void)
{
	SysTick10ms = (SysTick10ms + 1) & 0xFFFFFFFF;
}

//获取系统10毫秒滴答时钟
u32 GetSysTick10ms(void)
{
	return SysTick10ms;
}

//产生一个系统100毫秒滴答时钟.
void SysTick100msAdder(void)
{
	SysTick100ms = (SysTick100ms + 1) & 0xFFFFFFFF;
}

//获取系统100毫秒滴答时钟
u32 GetSysTick100ms(void)
{
	return SysTick1ms;
}

void SetSysTick1s(time_t sec)
{
	SysTick1s = sec;
}

//获取系统1秒滴答时钟
time_t GetSysTick1s(void)
{
	return SysTick1s;
}

//从EEPROM中读取数据(带CRC16校验码)len包括CRC16校验码
u8 ReadDataFromEepromToHoldBuf(u8 *inbuf,u16 s_add, u16 len)
{
	u16 i = 0;
	u16 ReadCrcCode;
	u16 CalCrcCode = 0;

	for(i = s_add; i < s_add + len; i ++)
	{
		*(inbuf + i) = AT24CXX_ReadOneByte(i);
	}

	ReadCrcCode=(u16)(*(inbuf + s_add + len - 2));
	ReadCrcCode=ReadCrcCode<<8;
	ReadCrcCode=ReadCrcCode|(u16)(u16)(*(inbuf + s_add + len - 1));

	CalCrcCode = CRC16(inbuf + s_add,len - 2);

	if(ReadCrcCode == CalCrcCode)
	{
		return 1;
	}

	return 0;
}

//向EEPROM中写入数据(带CRC16校验码)len不包括CRC16校验码
void WriteDataFromHoldBufToEeprom(u8 *inbuf,u16 s_add, u16 len)
{
	u16 i = 0;
	u16 j = 0;
	u16 CalCrcCode = 0;

	CalCrcCode = CRC16(inbuf,len);
	*(inbuf + len + 0) = (u8)(CalCrcCode >> 8);
	*(inbuf + len + 1) = (u8)(CalCrcCode & 0x00FF);

	for(i = s_add ,j = 0; i < s_add + len + 2; i ++, j ++)
	{
		AT24CXX_WriteOneByte(i,*(inbuf + j));
	}
}

//将数字或者缓冲区当中的数据转换成字符串，并赋值给相应的指针
//type 0:转换数字id 1:转换缓冲区数据，add为缓冲区起始地址 2将字符串长度传到参数size中
u8 GetMemoryForString(u8 **str, u8 type, u32 id, u16 add, u16 size, u8 *hold_reg)
{
	u8 ret = 0;
	u8 len = 0;
	u8 new_len = 0;

	if(*str == NULL)
	{
		if(type == 0)
		{
			len = GetDatBit(id);
		}
		else if(type == 1)
		{
			len = *(hold_reg + add);
		}
		else if(type == 2)
		{
			len = size;
		}

		*str = (u8 *)mymalloc(sizeof(u8) * len + 1);
	}

	if(*str != NULL)
	{
		len = strlen((char *)*str);
		if(type == 0)
		{
			new_len = GetDatBit(id);
		}
		else if(type == 1)
		{
			new_len = *(hold_reg + add);
		}
		else if(type == 2)
		{
			new_len = size;

			add -= 1;
		}

		if(len == new_len)
		{
			memset(*str,0,new_len + 1);

			if(type == 0)
			{
				IntToString(*str,id,0);
			}
			else if(type == 1 || type == 2)
			{
				memcpy(*str,(hold_reg + add + 1),new_len);
			}
			ret = 1;
		}
		else
		{
			myfree(*str);
			*str = (u8 *)mymalloc(sizeof(u8) * new_len + 1);
			if(*str != NULL)
			{
				memset(*str,0,new_len + 1);

				if(type == 0)
				{
					IntToString(*str,id,0);
				}
				else if(type == 1 || type == 2)
				{
					memcpy(*str,(hold_reg + add + 1),new_len);
				}
				len = new_len;
				new_len = 0;
				ret = 1;
			}
		}
	}

	return ret;
}

//将字符串拷贝到指定地址
u8 CopyStrToPointer(u8 **pointer, u8 *str, u8 len)
{
	u8 ret = 0;

	if(*pointer == NULL)
	{
		*pointer = (u8 *)mymalloc(len + 1);
	}
	else if(*pointer != NULL)
	{
		myfree(*pointer);
		*pointer = (u8 *)mymalloc(sizeof(u8) * len + 1);
	}

	if(*pointer != NULL)
	{
		memset(*pointer,0,len + 1);

		memcpy(*pointer,str,len);

		ret = 1;
	}

	return ret;
}

//获取设备名称
u8 GetDeviceName(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&DeviceName, 1, 0, DEVICE_NAME_ADD, 0,HoldReg);

	return ret;
}

//获取设备ID
u8 GetDeviceID(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&DeviceID, 2, 0, DEVICE_ID_ADD, DEVICE_ID_LEN - 2, HoldReg);

	return ret;
}

//获取设备UUID
u8 GetDeviceUUID(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&DeviceUUID, 2, 0, UU_ID_ADD, UU_ID_LEN - 2, HoldReg);

	return ret;
}

//获取设备ICCID
u8 GetDeviceICCID(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&DeviceICCID, 2, 0, ICC_ID_ADD, ICC_ID_LEN - 2, HoldReg);

	return ret;
}

//获取设备IMSI
u8 GetDeviceIMSI(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&DeviceIMSI, 2, 0, IMSI_ID_ADD, IMSI_ID_LEN - 2, HoldReg);

	return ret;
}

//获取APN
u8 GetAPN(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&APName, 1, 0, APN_ADD, 0, HoldReg);

	return ret;
}

//获取服务器域名
u8 GetServerDomain(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&ServerDomain, 1, 0, SERVER_DOMAIN_ADD, 0, HoldReg);

	return ret;
}

//获取服务器IP
u8 GetServerIP(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&ServerIP, 1, 0, SERVER_IP_ADD, 0, HoldReg);

	return ret;
}

//获取服务器Port
u8 GetServerPort(void)
{
	u8 ret = 0;

	ret = GetMemoryForString(&ServerPort, 1, 0, SERVER_PORT_ADD, 0, HoldReg);

	return ret;
}

//读取BootLoader版本号
u8 ReadBootLoaderVersion(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,BOOT_SW_VER_ADD, BOOT_SW_VER_LEN);

	if(ret)
	{
		if(BootLoaderVersion == NULL)
		{
			BootLoaderVersion = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(BootLoaderVersion,0,6);

		sprintf((char *)BootLoaderVersion, "%02d.%02d", HoldReg[BOOT_SW_VER_ADD + 0],HoldReg[BOOT_SW_VER_ADD + 1]);
	}
	else
	{
		if(BootLoaderVersion == NULL)
		{
			BootLoaderVersion = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(BootLoaderVersion,0,6);

		sprintf((char *)BootLoaderVersion, "null");
	}

	return ret;
}

//读取应用程序版本号
u8 ReadSoftWareVersion(void)
{
	u8 ret = 1;

	if(SoftWareVersion == NULL)
	{
		SoftWareVersion = (u8 *)mymalloc(sizeof(u8) * 6);
	}

	memset(SoftWareVersion,0,6);

	sprintf((char *)SoftWareVersion, "%02d.%02d", SOFT_WARE_VRESION / 100,SOFT_WARE_VRESION % 100);

	return ret;
}

//读取硬件版本号
u8 ReadHardWareVersion(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,HW_VER_ADD, HW_VER_LEN);

	if(ret)
	{
		if(HardWareVersion == NULL)
		{
			HardWareVersion = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(HardWareVersion,0,6);

		sprintf((char *)HardWareVersion, "%02d.%02d", HoldReg[HW_VER_ADD + 0],HoldReg[HW_VER_ADD + 1]);
	}
	else
	{
		if(HardWareVersion == NULL)
		{
			HardWareVersion = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(HardWareVersion,0,6);

		sprintf((char *)HardWareVersion, "null");
	}

	return ret;
}

//读取设备名称
u8 ReadDeviceName(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,DEVICE_NAME_ADD, DEVICE_NAME_LEN);

	if(ret)
	{
		GetDeviceName();
	}
	else
	{
		if(DeviceName == NULL)
		{
			DeviceName = (u8 *)mymalloc(sizeof(u8) * 5);
		}

		memset(DeviceName,0,5);

		sprintf((char *)DeviceName, "null");
	}

	return ret;
}

//读取设备ID
u8 ReadDeviceID(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,DEVICE_ID_ADD, DEVICE_ID_LEN);

	if(ret)
	{
		GetDeviceID();
	}
	else
	{
		if(DeviceID == NULL)
		{
			DeviceID = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(DeviceID,0,6);

		DeviceID[4] = 0x00;		//代表方形单灯控制器
		DeviceID[5] = 0x01;		//代表单灯控制器
	}

	return ret;
}

//读取设备UUID
u8 ReadDeviceUUID(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,UU_ID_ADD, UU_ID_LEN);

	if(ret)
	{
		GetDeviceUUID();
	}
	else
	{
		if(DeviceUUID == NULL)
		{
			DeviceUUID = (u8 *)mymalloc(sizeof(u8) * UU_ID_LEN);
		}

		memset(DeviceUUID,0,UU_ID_LEN);

		sprintf((char *)DeviceUUID, "00000000000000000");
	}

	return ret;
}

//读取设备ICCID
u8 ReadDeviceICCID(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,ICC_ID_ADD, ICC_ID_LEN);

	if(ret)
	{
		GetDeviceICCID();
	}
	else
	{
		if(DeviceICCID == NULL)
		{
			DeviceICCID = (u8 *)mymalloc(sizeof(u8) * ICC_ID_LEN);
		}

		memset(DeviceICCID,0,ICC_ID_LEN);

		sprintf((char *)DeviceICCID, "00000000000000000000");
	}

	return ret;
}

//读取设备IMSI
u8 ReadDeviceIMSI(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,IMSI_ID_ADD, IMSI_ID_LEN);

	if(ret)
	{
		GetDeviceIMSI();
	}
	else
	{
		if(DeviceIMSI == NULL)
		{
			DeviceIMSI = (u8 *)mymalloc(sizeof(u8) * IMSI_ID_LEN);
		}

		memset(DeviceIMSI,0,IMSI_ID_LEN);

		sprintf((char *)DeviceIMSI, "000000000000000");
	}

	return ret;
}

//读取运营商编号
u8 ReadOperators(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,OPERATORS_ADD, OPERATORS_LEN);

	if(ret)
	{
		if(HoldReg[OPERATORS_ADD] < 3)
		{
			Operators = HoldReg[OPERATORS_ADD];
		}
		else
		{
			Operators = 0;
		}
	}

	return ret;
}

//读取APN
u8 ReadAPN(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,APN_ADD, APN_LEN);

	if(ret)
	{
		GetAPN();
	}
	else
	{
		if(APName == NULL)
		{
			APName = (u8 *)mymalloc(sizeof(u8) * 5);
		}

		memset(APName,0,5);

		sprintf((char *)APName, "null");
	}

	return ret;
}

//读取服务器域名
u8 ReadServerDomain(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,SERVER_DOMAIN_ADD, SERVER_DOMAIN_LEN);

	if(ret)
	{
		GetServerDomain();
	}
	else
	{
		if(ServerDomain == NULL)
		{
			ServerDomain = (u8 *)mymalloc(sizeof(u8) * 19);
		}

		memset(ServerDomain,0,19);

		sprintf((char *)ServerDomain, "www.nnlighting.com");
	}

	return ret;
}

//读取服务器IP
u8 ReadServerIP(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,SERVER_IP_ADD, SERVER_IP_LEN);

	if(ret)
	{
		GetServerIP();
	}
	else
	{
		if(ServerIP == NULL)
		{
			ServerIP = (u8 *)mymalloc(sizeof(u8) * 16);
		}

		memset(ServerIP,0,16);

//		sprintf((char *)ServerIP, "180.101.147.115");
		sprintf((char *)ServerIP, "103.48.232.119");
	}

	return ret;
}

//读取服务器port
u8 ReadServerPort(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,SERVER_PORT_ADD, SERVER_PORT_LEN);

	if(ret)
	{
		GetServerPort();
	}
	else
	{
		if(ServerPort == NULL)
		{
			ServerPort = (u8 *)mymalloc(sizeof(u8) * 6);
		}

		memset(ServerPort,0,6);

//		sprintf((char *)ServerPort, "5683");
		sprintf((char *)ServerPort, "56556");
	}

	return ret;
}

//读取数据上传间隔时间
u8 ReadUpLoadINVL(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,UPLOAD_INVL_ADD, UPLOAD_INVL_LEN);

	if(ret)
	{
		UpLoadINCL = (((u16)HoldReg[UPLOAD_INVL_ADD + 0]) << 8) + (u16)HoldReg[UPLOAD_INVL_ADD + 1];

		if(UpLoadINCL > MAX_UPLOAD_INVL)
		{
			UpLoadINCL = 10;
		}
	}

	return ret;
}

//读取电源控制接口编号
u8 ReadPowerINTFCC(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,POWER_INTFC_ADD, POWER_INTFC_LEN);

	if(ret)
	{
		if(HoldReg[POWER_INTFC_ADD] < 3)
		{
			PowerINTFC = HoldReg[POWER_INTFC_ADD];
		}
		else
		{
			PowerINTFC = 0;
		}
	}

	return ret;
}

//读取时区偏移
u8 ReadTimeZone(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,TIME_ZONE_ADD, TIME_ZONE_LEN);

	if(ret)
	{
		if(HoldReg[TIME_ZONE_ADD] < 25)
		{
			TimeZone = HoldReg[TIME_ZONE_ADD];
		}
		else
		{
			TimeZone = 8;
		}
	}

	return ret;
}

//读取亮度百分比
u8 ReadLightLevelPercent(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,LIGHT_LEVEL_ADD, LIGHT_LEVEL_LEN);

	if(ret)
	{
		if(HoldReg[LIGHT_LEVEL_ADD] <= 200)
		{
			LightLevelPercent = HoldReg[LIGHT_LEVEL_ADD];
		}
		else
		{
			LightLevelPercent = 200;
		}
	}

	return ret;
}

//读取时间策略组数
u8 ReadTimeGroupNumber(void)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(HoldReg,TIME_GROUP_NUM_ADD, TIME_GROUP_NUM_LEN);

	if(ret)
	{
		if(HoldReg[TIME_GROUP_NUM_ADD] >= 2 && HoldReg[TIME_GROUP_NUM_ADD] <= MAX_GROUP_NUM)
		{
			TimeGroupNumber = HoldReg[TIME_GROUP_NUM_ADD];
		}
		else
		{
			TimeGroupNumber = 0;
		}
	}

	return ret;
}

//设定OTA参数到EEPROM中
void WriteOTAInfo(u8 *hold_reg,u8 reset)
{
	u16 crc_code = 0;
	u16 i = 0;

	if(reset == 1)
	{
		HaveNewFirmWare   = 0;
		NewFirmWareBagNum = 0;
		NewFirmWareVer    = 101;
		LastBagByteNum    = 0;
	}

	if(NewFirmWareAdd != 0xAA && NewFirmWareAdd != 0x55)
	{
		NewFirmWareAdd = 0xAA;
	}

	*(hold_reg + FIRM_WARE_FLAG_S_ADD) 			= HaveNewFirmWare;
	*(hold_reg + FIRM_WARE_TYPE_S_ADD) 			= DEVICE_TYPE;
	*(hold_reg + FIRM_WARE_STORE_ADD_S_ADD) 	= NewFirmWareAdd;
	*(hold_reg + FIRM_WARE_VER_S_ADD + 0) 		= (u8)((NewFirmWareVer >> 8) & 0x00FF);
	*(hold_reg + FIRM_WARE_VER_S_ADD + 1) 		= (u8)(NewFirmWareVer & 0x00FF);
	*(hold_reg + FIRM_WARE_BAG_NUM_S_ADD + 0) 	= (u8)((NewFirmWareBagNum >> 8) & 0x00FF);
	*(hold_reg + FIRM_WARE_BAG_NUM_S_ADD + 1) 	= (u8)(NewFirmWareBagNum & 0x00FF);
	*(hold_reg + LAST_BAG_BYTE_NUM_S_ADD) 		= LastBagByteNum;

	crc_code = CRC16(hold_reg + OTA_INFO_ADD, OTA_INFO_LEN - 2);

	*(hold_reg + OTA_INFO_ADD + OTA_INFO_LEN - 2) = (u8)(crc_code >> 8);
	*(hold_reg + OTA_INFO_ADD + OTA_INFO_LEN - 1) = (u8)(crc_code & 0x00FF);

	for(i = OTA_INFO_ADD; i < OTA_INFO_ADD + OTA_INFO_LEN; i ++)
	{
		AT24CXX_WriteOneByte(i,*(hold_reg + i));
	}
}

//从EEPROM中读取OTA信息
u8 ReadOTAInfo(u8 *hold_reg)
{
	u8 ret = 0;

	ret = ReadDataFromEepromToHoldBuf(hold_reg,OTA_INFO_ADD,OTA_INFO_LEN);

	if(ret == 1)
	{
		HaveNewFirmWare 	= *(hold_reg + FIRM_WARE_FLAG_S_ADD);
		NewFirmWareAdd 		= *(hold_reg + FIRM_WARE_STORE_ADD_S_ADD);
		NewFirmWareVer 		= (((u16)(*(hold_reg + FIRM_WARE_VER_S_ADD + 0))) << 8) + \
								(u16)(*(hold_reg + FIRM_WARE_VER_S_ADD + 1));
		NewFirmWareBagNum 	= (((u16)(*(hold_reg + FIRM_WARE_BAG_NUM_S_ADD + 0))) << 8) + \
								(u16)(*(hold_reg + FIRM_WARE_BAG_NUM_S_ADD + 1));
		LastBagByteNum 		= *(hold_reg + LAST_BAG_BYTE_NUM_S_ADD);
	}
	else
	{
		WriteOTAInfo(HoldReg,1);		//复位OTA信息
	}

	return ret;
}

//读取时间策略数组
u8 ReadRegularTimeGroups(void)
{
	u8 ret = 0;
	u16 i = 0;
	u16 j = 0;
	u16 read_crc = 0;
	u16 cal_crc = 0;
	u8 time_group[1024];
	u8 read_success_buf_flag[MAX_GROUP_NUM];
	
	RegularTimeWeekDay = (pRegularTime)mymalloc(sizeof(RegularTime_S));
	RegularTimeWeekEnd = (pRegularTime)mymalloc(sizeof(RegularTime_S));
	RegularTimeHoliday = (pRegularTime)mymalloc(sizeof(RegularTime_S));
	
	RegularTimeWeekDay->number = 0xFF;
	RegularTimeWeekEnd->number = 0xFF;
	RegularTimeHoliday->number = 0xFF;
	
	RegularTimeWeekDay->prev = NULL;
	RegularTimeWeekEnd->prev = NULL;
	RegularTimeHoliday->prev = NULL;
	
	RegularTimeWeekDay->next = NULL;
	RegularTimeWeekEnd->next = NULL;
	RegularTimeHoliday->next = NULL;

	memset(time_group,0,1024);
	memset(read_success_buf_flag,0,MAX_GROUP_NUM);

	for(i = 0; i < MAX_GROUP_NUM; i ++)
	{
		for(j = i * TIME_RULE_LEN; j < i * TIME_RULE_LEN + TIME_RULE_LEN; j ++)
		{
			time_group[j] = AT24CXX_ReadOneByte(TIME_RULE_ADD + j);
		}

		cal_crc = CRC16(&time_group[j - TIME_RULE_LEN],7);
		read_crc = (((u16)time_group[j - 2]) << 8) + (u16)time_group[j - 1];

		if(cal_crc == read_crc)
		{
			read_success_buf_flag[i] = 1;
		}
	}
	
	for(i = 0; i < MAX_GROUP_NUM; i ++)
	{
		if(read_success_buf_flag[i] == 1)
		{
			pRegularTime tmp_time = NULL;

			tmp_time = (pRegularTime)mymalloc(sizeof(RegularTime_S));
			
			tmp_time->prev = NULL;
			tmp_time->next = NULL;

			tmp_time->number	= i;
			tmp_time->type 		= time_group[i * TIME_RULE_LEN + 0];
			tmp_time->year 		= time_group[i * TIME_RULE_LEN + 1];
			tmp_time->month 	= time_group[i * TIME_RULE_LEN + 2];
			tmp_time->date 		= time_group[i * TIME_RULE_LEN + 3];
			tmp_time->hour 		= time_group[i * TIME_RULE_LEN + 4];
			tmp_time->minute 	= time_group[i * TIME_RULE_LEN + 5];
			tmp_time->percent 	= time_group[i * TIME_RULE_LEN + 6];

			switch(tmp_time->type)
			{
				case TYPE_WEEKDAY:
					RegularTimeGroupAdd(TYPE_WEEKDAY,tmp_time);
				break;

				case TYPE_HOLIDAY_START:
					RegularTimeGroupAdd(TYPE_HOLIDAY_START,tmp_time);
				break;

				case TYPE_HOLIDAY_END:
					RegularTimeGroupAdd(TYPE_HOLIDAY_END,tmp_time);
				break;

				default:

				break;
			}
		}
	}

	return ret;
}

//在EEPROM中读取运行参数
void ReadParametersFromEEPROM(void)
{
	ReadBootLoaderVersion();
	ReadSoftWareVersion();
	ReadHardWareVersion();
	ReadDeviceName();
	ReadDeviceID();
	ReadDeviceUUID();
	ReadDeviceICCID();
	ReadDeviceIMSI();
	ReadOperators();
	ReadAPN();
	ReadServerDomain();
	ReadServerIP();
	ReadServerPort();
	ReadUpLoadINVL();
	ReadPowerINTFCC();
	ReadTimeZone();
//	ReadLightLevelPercent();
	ReadRegularTimeGroups();
	ReadOTAInfo(HoldReg);
}

//将数据打包成网络格式的数据
u16 PackNetData(u8 cmd_code,u8 *inbuf,u16 inbuf_len,u8 *outbuf)
{	
	u8 i = 0;
	u8 tmp_buf[256];
	u16 out_len = 0;
	
	if(inbuf_len == 0 ||
		inbuf == NULL ||
		outbuf == NULL ||
		DeviceID == NULL ||
		DeviceUUID == NULL)
	{
		return 0;
	}
	
	memset(tmp_buf,0,256);
	
	tmp_buf[0] = 0x68;
	memcpy(tmp_buf + 1,DeviceID,6);
	tmp_buf[7] = 0x68;
	memcpy(tmp_buf + 8,DeviceUUID,17);
	tmp_buf[25] = cmd_code;
	tmp_buf[26] = inbuf_len;
	memcpy(tmp_buf + 27,inbuf,inbuf_len);
	
	for(i = 0; i < 27 + inbuf_len; i ++)
	{
		tmp_buf[27 + inbuf_len] += tmp_buf[i];
	}
	
	tmp_buf[27 + inbuf_len + 1] = 0x16;
	
	tmp_buf[27 + inbuf_len + 2] = 0xFE;
	tmp_buf[27 + inbuf_len + 3] = 0xFD;
	tmp_buf[27 + inbuf_len + 4] = 0xFC;
	tmp_buf[27 + inbuf_len + 5] = 0xFB;
	tmp_buf[27 + inbuf_len + 6] = 0xFA;
	tmp_buf[27 + inbuf_len + 7] = 0xF9;
	
	out_len = 35 + inbuf_len;
	
	HexToStr((char *)outbuf, tmp_buf, out_len);

	return out_len;
}

//将传感器数据解包到指定缓冲区
u16 UnPackSensorData(SensorMsg_S *msg,u8 *buf)
{
	u16 i = 0;
	u16 len = 0;

	if(msg != NULL)
	{
//		*(buf + i) = (u8)(msg->in_put_current >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_current;
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_voltage >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_voltage;
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_freq >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_freq;
//		i ++;

//		*(buf + i) = (u8)((msg->in_put_power_p >> 8) & 0x00FF);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_power_p & 0x00FF);
//		i ++;

//		*(buf + i) = (u8)((msg->in_put_power_q >> 8) & 0x00FF);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_power_q & 0x00FF);
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_power_s >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_power_s;
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_energy_p >> 24);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_p >> 16);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_p >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_energy_p;
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_energy_q >> 24);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_q >> 16);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_q >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_energy_q;
//		i ++;

//		*(buf + i) = (u8)(msg->in_put_energy_s >> 24);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_s >> 16);
//		i ++;
//		*(buf + i) = (u8)(msg->in_put_energy_s >> 8);
//		i ++;
//		*(buf + i) = (u8)msg->in_put_energy_s;
//		i ++;

		*(buf + i) = (u8)(msg->out_put_current >> 8);
		i ++;
		*(buf + i) = (u8)msg->out_put_current;
		i ++;

		*(buf + i) = (u8)(msg->out_put_voltage >> 8);
		i ++;
		*(buf + i) = (u8)msg->out_put_voltage;
		i ++;

		*(buf + i) = msg->signal_intensity;
		i ++;
		
		*(buf + i) = msg->hour;
		i ++;
		
		*(buf + i) = msg->minute;
		i ++;
		
		*(buf + i) = msg->second;
		i ++;

		len = i;
	}

	return len;
}

u8 RegularTimeGroupAdd(u8 type,pRegularTime group_time)
{
	u8 ret = 1;
	pRegularTime tmp_time = NULL;
	pRegularTime main_time = NULL;

	if(xSchedulerRunning == 1)
	{
		xSemaphoreTake(xMutex_STRATEGY, portMAX_DELAY);
	}
	
	switch(type)
	{
		case TYPE_WEEKDAY:
			main_time = RegularTimeWeekDay;
		break;

		case TYPE_WEEKEND:
			main_time = RegularTimeWeekEnd;
		break;

		case TYPE_HOLIDAY_START:
			main_time = RegularTimeHoliday;
			
			HolodayRange.year_s  = group_time->year;
			HolodayRange.month_s = group_time->month;
			HolodayRange.date_s  = group_time->date;
			HolodayRange.year_e  = group_time->year;
			HolodayRange.month_e = group_time->month;
			HolodayRange.date_e  = group_time->date;
		break;

		case TYPE_HOLIDAY_END:
			main_time = RegularTimeHoliday;
			
			HolodayRange.year_e  = group_time->year;
			HolodayRange.month_e = group_time->month;
			HolodayRange.date_e  = group_time->date;
		break;
		
		default:

		break;
	}
	
	if(main_time != NULL)
	{
		for(tmp_time = main_time; tmp_time != NULL; tmp_time = tmp_time->next)
		{
			if(group_time->number == tmp_time->number && tmp_time->number != 0xFF)
			{
				if(tmp_time->next != NULL)
				{
					tmp_time->prev->next = group_time;
					tmp_time->prev->next->next = tmp_time->next;
					tmp_time->next->prev = group_time;
					tmp_time->next->prev->prev = tmp_time->prev;
					
					myfree(tmp_time);
				}
				else
				{
					tmp_time->prev->next = group_time;
					tmp_time->prev->next->prev = tmp_time->prev;
					
					myfree(tmp_time);
				}
				
				break;
			}
			else if(tmp_time->next == NULL)
			{
				tmp_time->next = group_time;
				tmp_time->next->prev = tmp_time;
				
				break;
			}
		}
	}
	
	if(xSchedulerRunning == 1)
	{
		xSemaphoreGive(xMutex_STRATEGY);
	}
	
	return ret;
}

u8 RegularTimeGroupSub(u8 number)
{
	u8 ret = 0;
	pRegularTime tmp_time = NULL;

	if(xSchedulerRunning == 1)
	{
		xSemaphoreTake(xMutex_STRATEGY, portMAX_DELAY);
	}
	
	if(RegularTimeWeekDay != NULL || RegularTimeWeekDay->next != NULL)
	{
		for(tmp_time = RegularTimeWeekDay->next; tmp_time != NULL; tmp_time = tmp_time->next)
		{
			if(tmp_time->number == number)
			{
				if(tmp_time->next != NULL)
				{
					tmp_time->prev->next = tmp_time->next;
					tmp_time->next->prev = tmp_time->prev;
				}
				else
				{
					tmp_time->prev->next = NULL;
				}
				
				myfree(tmp_time);
				
				ret = 1;
			}
		}
	}

	if(RegularTimeWeekEnd != NULL || RegularTimeWeekEnd->next != NULL)
	{
		for(tmp_time = RegularTimeWeekEnd->next; tmp_time != NULL; tmp_time = tmp_time->next)
		{
			if(tmp_time->number == number)
			{
				if(tmp_time->next != NULL)
				{
					tmp_time->prev->next = tmp_time->next;
					tmp_time->next->prev = tmp_time->prev;
				}
				else
				{
					tmp_time->prev->next = NULL;
				}
				
				myfree(tmp_time);
				
				ret = 1;
			}
		}
	}

	if(RegularTimeHoliday != NULL || RegularTimeHoliday->next != NULL)
	{
		for(tmp_time = RegularTimeHoliday->next; tmp_time != NULL; tmp_time = tmp_time->next)
		{
			if(tmp_time->number == number)
			{
				if(tmp_time->next != NULL)
				{
					tmp_time->prev->next = tmp_time->next;
					tmp_time->next->prev = tmp_time->prev;
				}
				else
				{
					tmp_time->prev->next = NULL;
				}
				
				myfree(tmp_time);

				ret = 1;
			}
		}
	}
	
	if(xSchedulerRunning == 1)
	{
		xSemaphoreGive(xMutex_STRATEGY);
	}
	
	return ret;
}

void RemoveAllStrategy(void)
{
	u16 i = 0;
	
	for(i = 0; i < MAX_GROUP_NUM; i ++)
	{
		RegularTimeGroupSub(i);
		
		AT24CXX_WriteLenByte(TIME_RULE_ADD + TIME_RULE_LEN * i + 7,0x0000,2);
	}
}

	











