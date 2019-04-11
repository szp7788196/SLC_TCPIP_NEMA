#include "sys.h"
#include "usart.h"
#include "common.h"
#include "bcxx.h"


u16 Usart1RxCnt = 0;
u16 OldUsart1RxCnt = 0;
u16 Usart1FrameLen = 0;
u8 Usart1RxBuf[Usart1RxLen];
u8 Usart1TxBuf[Usart1TxLen];
u8 Usart1RecvEnd = 0;
u8 Usart1Busy = 0;
u16 Usart1SendLen = 0;
u16 Usart1SendNum = 0;

u16 Usart4RxCnt = 0;
u16 OldUsart4RxCnt = 0;
u16 Usart4FrameLen = 0;
u8 Usart4RxBuf[Usart4RxLen];
u8 Usart4TxBuf[Usart4TxLen];
u8 Usart4RecvEnd = 0;
u8 Usart4Busy = 0;
u16 Usart4SendLen = 0;
u16 Usart4SendNum = 0;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x)
{
	x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8)ch;
	return ch;
}
#endif

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	USART_Cmd(USART1, DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);		//使能USART1，GPIOA时钟
	USART_DeInit(USART1);  															//复位串口1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 										//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;									//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); 											//初始化PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;										//PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;							//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  										//初始化PA10

	USART_InitStructure.USART_BaudRate = bound;										//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART1, &USART_InitStructure); 										//初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//开启中断
	USART_Cmd(USART1, ENABLE);                    									//使能串口
}

void UART4_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	USART_Cmd(UART4, DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);							//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);							//使能USART2时钟
	USART_DeInit(UART4);  															//复位串口4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 										//PC10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;									//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure); 											//初始化PC10

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;										//PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;							//浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);  										//初始化PC11

	USART_InitStructure.USART_BaudRate = bound;										//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(UART4, &USART_InitStructure); 										//初始化串口
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);									//开启中断
	USART_Cmd(UART4, ENABLE);                    									//使能串口
}

u8 UsartSendString(USART_TypeDef* USARTx,u8 *str, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)
    {
		USART_ClearFlag(USARTx,USART_FLAG_TC);
		USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
		USART_ClearFlag(USARTx,USART_FLAG_TC);
	}
	return 1;
}

void USART1_IRQHandler(void)
{
	u8 rxdata;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  	{
		rxdata =USART_ReceiveData(USART1);

		if(Usart1RxCnt<Usart1RxLen && Usart1Busy == 0)
		{
			Usart1RxBuf[Usart1RxCnt]=rxdata;
			Usart1RxCnt++;
		}
  	}

	if(USART_GetITStatus(USART1,USART_IT_TC)!=RESET)
	{
		Usart1FrameSend();
	}

	//以下为串口中断出错后的处理  经验之谈
	else if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{
		rxdata = USART_ReceiveData(USART1);
		rxdata = rxdata;
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_NE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	else if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}
}


void UART4_IRQHandler(void)
{
	u8 rxdata;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  	{
		rxdata =USART_ReceiveData(UART4);

		if(Usart4RxCnt<Usart4RxLen && Usart4Busy == 0)
		{
			Usart4RxBuf[Usart4RxCnt]=rxdata;
			Usart4RxCnt++;
		}
  	}

	if(USART_GetITStatus(UART4,USART_IT_TC)!=RESET)
	{
		Usart4FrameSend();
	}

	//以下为串口中断出错后的处理  经验之谈
	else if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET)
	{
		rxdata = USART_ReceiveData(UART4);
		rxdata = rxdata;
		USART_ClearFlag(UART4, USART_FLAG_ORE);
	}
	else if(USART_GetFlagStatus(UART4, USART_FLAG_NE) != RESET)
	{
		USART_ClearFlag(UART4, USART_FLAG_NE);
	}
	else if(USART_GetFlagStatus(UART4, USART_FLAG_FE) != RESET)
	{
		USART_ClearFlag(UART4, USART_FLAG_FE);
	}
	else if(USART_GetFlagStatus(UART4, USART_FLAG_PE) != RESET)
	{
		USART_ClearFlag(UART4, USART_FLAG_PE);
	}
}

void Usart1ReciveFrameEnd(void)
{
	if(Usart1RxCnt)
	{
		if(OldUsart1RxCnt == Usart1RxCnt)
		{
			Usart1FrameLen = Usart1RxCnt;
			OldUsart1RxCnt = 0;
			Usart1RxCnt = 0;
			Usart1RecvEnd = 0xAA;
		}
		else
		{
			OldUsart1RxCnt = Usart1RxCnt;
		}
	}
}

void Usart4ReciveFrameEnd(void)
{
	if(Usart4RxCnt)
	{
		if(OldUsart4RxCnt == Usart4RxCnt)
		{
			Usart4FrameLen = Usart4RxCnt;
			OldUsart4RxCnt = 0;
			Usart4RxCnt = 0;
			Usart4RecvEnd = 0xAA;
		}
		else
		{
			OldUsart4RxCnt = Usart4RxCnt;
		}
	}
}

void Usart1FrameSend(void)
{
	u8 send_data = 0;
	send_data = Usart1TxBuf[Usart1SendNum];
	USART_SendData(USART1,send_data);
	Usart1SendNum ++;
	if(Usart1SendNum >= Usart1SendLen)					//发送已经完成
	{
		Usart1Busy = 0;
		Usart1SendLen = 0;								//要发送的字节数清零
		Usart1SendNum = 0;								//已经发送的字节数清零
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);	//关闭数据发送中断
		memset(Usart1TxBuf,0,Usart1TxLen);
	}
}

void Usart4FrameSend(void)
{
	u8 send_data = 0;
	send_data = Usart4TxBuf[Usart4SendNum];
	USART_SendData(UART4,send_data);
	Usart4SendNum ++;
	if(Usart4SendNum >= Usart4SendLen)					//发送已经完成
	{
		Usart4Busy = 0;
		Usart4SendLen = 0;								//要发送的字节数清零
		Usart4SendNum = 0;								//已经发送的字节数清零
		USART_ITConfig(UART4, USART_IT_TC, DISABLE);	//关闭数据发送中断
		memset(Usart4TxBuf,0,Usart4TxLen);
	}
}

void TIM2_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 		//时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 			//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(TIM2,TIM_IT_Update ,ENABLE);
	 							//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  									//使能TIMx外设
}

void TIM2_IRQHandler(void)
{
	static u8 tick_10ms = 0;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 			//检查指定的TIM中断发生与否:TIM 中断源
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  			//清除TIMx的中断待处理位:TIM 中断源

		SysTick10msAdder();			//10ms滴答计数器累加

		Usart1ReciveFrameEnd();		//检测USART1是否接收数据结束
 		Usart4ReciveFrameEnd();		//检测UART1是否接收数据结束

		tick_10ms ++;
		if(tick_10ms >= 10)
		{
			tick_10ms = 0;

			SysTick100msAdder();	//100ms滴答计数器累加
		}
	}
}









