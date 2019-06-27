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
	u8 ret = 0;
	u16 gate0 = 0;
	u16 gate1 = 0;
	u16 gate2 = 0;
	u16 gate24 = 1440;	//24*60;
	u16 gate_n = 0;
	
	pRegularTime tmp_time = NULL;

	if(GetTimeOK != 0)
	{
		xSemaphoreTake(xMutex_STRATEGY, portMAX_DELAY);
		
		if(calendar.week <= 6)	//判断是否是工作日
		{
			if(RegularTimeWeekDay->next != NULL)		//判断策略列表是否不为空
			{
				for(tmp_time = RegularTimeWeekDay->next; tmp_time != NULL; tmp_time = tmp_time->next)	//轮训策略列表
				{
					if(tmp_time->hour 	== calendar.hour &&
					   tmp_time->minute == calendar.min)		//判断当前时间是否同该条策略时间相同
					{
						ret = 1;
					}
					else if(tmp_time->next != NULL)				//该条策略是不是最后一条
					{
						if(tmp_time->next->hour   == calendar.hour &&
					       tmp_time->next->minute == calendar.min)		//判断该条策略的next的时间是否与当前时间相同
						{
							tmp_time = tmp_time->next;
							
							ret = 1;
						}
						else
						{
							gate1 = tmp_time->hour * 60 + tmp_time->minute;					//该条策略的分钟数
							gate2 = tmp_time->next->hour * 60 + tmp_time->next->minute;		//该条策略的next的分钟数
							gate_n = calendar.hour * 60 + calendar.min;						//当前时间的分钟数

							if(gate1 < gate2)												//该条策略时间早于next的时间
							{
								if(gate1 <= gate_n && gate_n <= gate2)						//判断当前时间是否在两条策略时间段中间
								{
									ret = 1;
								}
							}
							else if(gate1 > gate2)											//该条策略时间晚于next的时间
							{
								if(gate1 <= gate_n && gate_n <= gate24)						//判断当前时间是否在该条策略时间和24点时间段中间
								{
									ret = 1;
								}
								else if(gate0 <= gate_n && gate_n <= gate2)					//判断当前时间是否在0点和next的时间段中间
								{
									ret = 1;
								}
							}
						}
					}
					else
					{
						gate1 = tmp_time->hour * 60 + tmp_time->minute;					//该条策略的分钟数
						gate_n = calendar.hour * 60 + calendar.min;						//当前时间的分钟数
						
						if(gate_n >= gate1)
						{
							ret = 1;
						}
					}

					if(ret == 1)
					{
						*percent = tmp_time->percent * 2;

						break;
					}
				}
			}
		}
		
		if(RegularTimeHoliday->next != NULL)
		{
			for(tmp_time = RegularTimeHoliday->next; tmp_time != NULL; tmp_time = tmp_time->next)
			{
				if(HolodayRange.year_s <= HolodayRange.year_e)				//起始年早于等于结束年
				{
					if(HolodayRange.year_s <= calendar.w_year - 2000 && 
					   calendar.w_year - 2000 <= HolodayRange.year_e)		//当前年处于起始和结束年之间
					{
						if(HolodayRange.month_s <= HolodayRange.month_e)
						{
							if(HolodayRange.month_s <= calendar.w_month && 
							   calendar.w_month <= HolodayRange.month_e)
							{
								if(HolodayRange.date_s <= HolodayRange.date_e)
								{
									if(HolodayRange.date_s <= calendar.w_date && 
									   calendar.w_date <= HolodayRange.date_e)
									{
										if(tmp_time->next != NULL)
										{
											if(tmp_time->next->hour   == calendar.hour &&
											   tmp_time->next->minute == calendar.min)
											{
												tmp_time = tmp_time->next;
												
												ret = 1;
											}
											else
											{
												gate1 = tmp_time->hour * 60 + tmp_time->minute;
												gate2 = tmp_time->next->hour * 60 + tmp_time->next->minute;
												gate_n = calendar.hour * 60 + calendar.min;

												if(gate1 < gate2)
												{
													if(gate1 <= gate_n && gate_n <= gate2)
													{
														ret = 1;
													}
												}
												else if(gate1 > gate2)
												{
													if(gate1 <= gate_n && gate_n <= gate24)
													{
														ret = 1;
													}
													else if(gate0 <= gate_n && gate_n <= gate2)
													{
														ret = 1;
													}
												}
											}
										}
										else
										{
											gate1 = tmp_time->hour * 60 + tmp_time->minute;					//该条策略的分钟数
											gate_n = calendar.hour * 60 + calendar.min;						//当前时间的分钟数
											
											if(gate_n >= gate1)
											{
												ret = 1;
											}
										}
									}
								}
							}
						}
					}
				}

				if(ret == 1)
				{
					*percent = tmp_time->percent * 2;

					break;
				}
			}
		}
		
		xSemaphoreGive(xMutex_STRATEGY);
	}
}



































