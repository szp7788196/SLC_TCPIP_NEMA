#include "task_main.h"
#include "common.h"
#include "delay.h"
#include "usart.h"
#include "inventr.h"


TaskHandle_t xHandleTaskMAIN = NULL;

u8 MirrorLightLevelPercent = 0;
unsigned portBASE_TYPE MAIN_Satck;

void vTaskMAIN(void *pvParameters)
{
	time_t times_sec = 0;

	InventrSetLightLevel(INIT_LIGHT_LEVEL);

	while(1)
	{
		if(DeviceWorkMode == MODE_AUTO)						//只有在自动模式下才进行策略判断
		{
			if(GetSysTick1s() - times_sec >= 1)
			{
				times_sec = GetSysTick1s();

				AutoLoopRegularTimeGroups(&LightLevelPercent);
			}
		}

		if(MirrorLightLevelPercent != LightLevelPercent)
		{
			MirrorLightLevelPercent = LightLevelPercent;

			InventrSetLightLevel(LightLevelPercent);
		}

		if(NeedToReset == 1)								//接收到重启的命令
		{
			NeedToReset = 0;
			delay_ms(1000);

			__disable_fault_irq();							//重启指令
			NVIC_SystemReset();
		}

		delay_ms(100);
//		MAIN_Satck = uxTaskGetStackHighWaterMark(NULL);
	}
}

//轮询时间策略
void AutoLoopRegularTimeGroups(u8 *percent)
{
	u8 i = 0;

	if(GetTimeOK != 0)
	{
		for(i = 0; i < MAX_GROUP_NUM; i ++)
		{
			switch(RegularTimeStruct[i].type)
			{
				case TYPE_WEEKDAY:		//周一至周五
					if(calendar.week >= 1 && calendar.week <= 5)		//判断现在是否是工作日
					{
						if(RegularTimeStruct[i].hour == calendar.hour &&
						   RegularTimeStruct[i].minute == calendar.min)
						{
							*percent = RegularTimeStruct[i].percent * 2;

							i = MAX_GROUP_NUM;
						}
					}
				break;

				case TYPE_WEEKEND:		//周六至周日
					if(calendar.week >= 6 && calendar.week <= 7)		//判断现在是否是周六或周日
					{
						if(RegularTimeStruct[i].hour == calendar.hour &&
						   RegularTimeStruct[i].minute == calendar.min)
						{
							*percent = RegularTimeStruct[i].percent * 2;

							i = MAX_GROUP_NUM;
						}
					}
				break;

				case TYPE_HOLIDAY:		//节假日
					if(RegularTimeStruct[i].year + 2000 == calendar.w_year &&
					   RegularTimeStruct[i].month == calendar.w_month &&
					   RegularTimeStruct[i].date == calendar.w_date &&
					   RegularTimeStruct[i].hour == calendar.hour &&
					   RegularTimeStruct[i].minute == calendar.min)
					{
						*percent = RegularTimeStruct[i].percent * 2;

						i = MAX_GROUP_NUM;
					}
				break;

				default:

				break;
			}
		}
	}
}



































