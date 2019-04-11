#include "bcxx.h"
#include "usart.h"
#include "common.h"
#include "delay.h"
#include <string.h>
#include <stdlib.h>

CONNECT_STATE_E ConnectState = UNKNOW_STATE;	//连接状态
u8 net_temp_data_rx_buf[8];
u8 net_idle_data_rx_buf[128];

char 			bcxx_rx_cmd_buf[CMD_DATA_BUFFER_SIZE];
u16  			bcxx_rx_cnt;
pRingBuf    	bcxx_net_buf;
u16  			bcxx_net_temp_data_rx_cnt;
u16  			bcxx_net_idle_data_rx_cnt;
u16  			bcxx_net_data_len;
u8   			bcxx_break_out_wait_cmd;
u8   			bcxx_init_ok;
u32    			bcxx_last_time;
CMD_STATE_E 	bcxx_cmd_state;
BCXX_MODE_E 	bcxx_mode;
USART_TypeDef* 	bcxx_USARTx = USART2;
char   			*bcxx_imei;


void bcxx_hard_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	BCXX_PWREN_LOW;
	BCXX_RST_LOW;
}

void bcxx_hard_reset(void)
{
	BCXX_PWREN_LOW;						//关闭电源
	delay_ms(300);
	BCXX_PWREN_HIGH;					//打开电源

	delay_ms(100);

	BCXX_RST_HIGH;						//硬件复位
	delay_ms(300);
	BCXX_RST_LOW;

	bcxx_init_ok = 1;
}


void bcxx_init(void)
{
	u8 ret = 0;
	static u8 hard_inited = 0;

	if(hard_inited == 0)
	{
		ret = RingBuf_Init(&bcxx_net_buf, NET_DATA_BUFFER_SIZE);
		if(ret == 0)
		{
			return;
		}

		bcxx_hard_init();
		
		hard_inited = 1;
	}

	RE_HARD_RESET:
	bcxx_hard_reset();

	delay_ms(5000);

	bcxx_clear_rx_cmd_buffer();
	bcxx_available();
	bcxx_net_temp_data_rx_cnt = 0;
	bcxx_net_idle_data_rx_cnt = 0;
	bcxx_net_data_len = 0;
	bcxx_break_out_wait_cmd = 0;
	bcxx_rx_cnt = 0;
	bcxx_mode = NET_MODE;
	bcxx_last_time = GetSysTick1ms();

	if(!bcxx_set_AT_ATE(0,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CFUN(0,3,TIMEOUT_90S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_NBAND(0,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CELL_RESELECTION(3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_NCDP((char *)ServerIP,(char *)ServerPort,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_get_AT_CGSN(3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_NRB(1,TIMEOUT_10S,10000))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CFUN(1,3,TIMEOUT_90S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CEDRXS(0,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CPSMS(0,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CSCON(0,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CEREG(4,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_QREGSWT(1,3,TIMEOUT_2S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_NSONMI(3,3,TIMEOUT_1S,100))
		goto RE_HARD_RESET;
	
	if(!bcxx_set_AT_CGATT(1,3,TIMEOUT_2S,100))
		goto RE_HARD_RESET;

	delay_ms(10000);
}


u8 bcxx_send_string(USART_TypeDef* USARTx,u8 *str, u16 len)
{
	if(USARTx == USART1)
	{
		memcpy(Usart1TxBuf,str,len);
		Usart1SendLen = len;
	}
	else if(USARTx == USART2)
	{
		memcpy(Usart2TxBuf,str,len);
		Usart2SendLen = len;
	}
	else if(USARTx == UART4)
	{
		memcpy(Usart4TxBuf,str,len);
		Usart4SendLen = len;
	}
	else
	{
		return 0;
	}

	USART_ITConfig(USARTx, USART_IT_TC, ENABLE);

	return 1;
}



u16 bcxx_read(u8 *buf)
{
    int i = 0;
    u16 len = bcxx_available();
    if(len > 0)
    {
        for(i = 0; i < len; i++)
        {
            buf[i] = bcxx_net_buf->read(&bcxx_net_buf);
        }
    }
    else
    {
        len = 0;
    }
    return len;
}

u8 bcxx_set_AT_NRB(u8 execute_times,u32 time_out,u16 wait_time)//TIMEOUT_10S
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NRB\r\n");
		if(bcxx_wait_cmd2("REBOOTING",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "REBOOTING") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
	
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}


//基本AT指令测试
u8 bcxx_set_AT(u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT\r\n");
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
	
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
  
	delay_ms(wait_time);
	
    return ret;
}

//设置回显功能
u8 bcxx_set_AT_ATE(char cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("ATE%d\r\n", cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;

}

u8 bcxx_set_AT_UART(u32 baud_rate,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;


    return ret;
}

//查询信号强度
u8 bcxx_get_AT_CSQ(u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
	u16 pos = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CSQ\r\n");
		if(bcxx_wait_cmd2("+CSQ:",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "+CSQ:") != -1)
			{
				pos = MyStrstr((u8 *)bcxx_rx_cmd_buf, "+CSQ:", 128, 5);
				if(pos != 0xFFFF)
				{
					if(bcxx_rx_cmd_buf[pos + 6] != ',')
					{
						ret = (bcxx_rx_cmd_buf[pos + 5] - 0x30) * 10 +\
							bcxx_rx_cmd_buf[pos + 6] - 0x30;
					}
					else
					{
						ret = bcxx_rx_cmd_buf[pos + 5] - 0x30;
					}

					if(ret == 0 || ret == 99)
					{
						ret = 0;
						
						break;
					}
					else
					{
						break;
					}
				}
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

u8 bcxx_set_AT_CFUN(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CFUN=%d\r\n",cmd);
		if(bcxx_wait_cmd1(time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置频带
u8 bcxx_set_AT_NBAND(u8 operators,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
	u8 band = 8;
	
	switch(operators)
	{
		case 0:				//移动
			band = 8;
		break;
		
		case 1:				//联通
			band = 8;
		break;
		
		case 2:				//电信
			band = 5;
		break;
		
		default:
			band = 8;
		break;
	}
	
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NBAND=%d\r\n",band);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//获取IMEI号
u8 bcxx_get_AT_CGSN(u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
	char buf[32];

    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CGSN=1\r\n");
		if(bcxx_wait_cmd1(time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				memset(buf,0,32);

				get_str1((u8 *)bcxx_rx_cmd_buf, "CGSN:", 1, "\r\n", 2, (u8 *)buf);

				if(strlen(buf) == 15)
				{
					if(bcxx_imei == NULL)
					{
						bcxx_imei = (char *)mymalloc(sizeof(char) * 16);
					}
					if(bcxx_imei != NULL)
					{
						memset(bcxx_imei,0,16);
						memcpy(bcxx_imei,buf,15);

						ret = 1;
						
						break;
					}
				}
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置IOT平台IP和端口
u8 bcxx_set_AT_NCDP(char *addr, char *port,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NCDP=%s,%s\r\n",addr,port);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置连接状态自动回显
u8 bcxx_set_AT_CSCON(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CSCON=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//EPS Network Registration Status
u8 bcxx_set_AT_CEREG(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CEREG=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置网络数据接收模式
u8 bcxx_set_AT_NNMI(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NNMI=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

u8 bcxx_set_AT_CGATT(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CGATT=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

u8 bcxx_set_AT_QREGSWT(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+QREGSWT=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置eDRX开关
u8 bcxx_set_AT_CEDRXS(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	unsigned char ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CEDRXS=%d,5\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置PMS开关
u8 bcxx_set_AT_CPSMS(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time)
{
	unsigned char ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CPSMS=%d\r\n",cmd);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
				ret = 0;
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//向IOT平台发送数据
u8 bcxx_set_AT_NMGS(u16 len,char *buf,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NMGS=%d,%s\r\n",len,buf);
		if(bcxx_wait_cmd1(time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
			else
			{
				ConnectState = DISCONNECT;

				ret = 0;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//设置接收模式
u8 bcxx_set_AT_NSONMI(u8 mode,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSONMI=%d\r\n",mode);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//开启小区重选功能
u8 bcxx_set_AT_CELL_RESELECTION(u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NCONFIG=CELL_RESELECTION,TRUE\r\n");
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//获取本地IP地址
u8 bcxx_get_AT_CGPADDR(char **ip,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
	u8 len = 0;
	u8 new_len = 0;
	u8 msg[20];
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CGPADDR\r\n");
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				memset(msg,0,20);

				get_str1((u8 *)bcxx_rx_cmd_buf, "+CGPADDR:0,", 1, "\r\n", 2, (u8 *)msg);

				new_len = strlen((char *)msg);

				if(new_len != 0)
				{
					if(*ip == NULL)
					{
						*ip = (char *)mymalloc(sizeof(u8) * len + 1);
					}

					if(*ip != NULL)
					{
						len = strlen((char *)*ip);

						if(len == new_len)
						{
							memset(*ip,0,new_len + 1);
							memcpy(*ip,msg,new_len);
							ret = 1;
							
							break;
						}
						else
						{
							myfree(*ip);
							*ip = (char *)mymalloc(sizeof(u8) * new_len + 1);
							if(ip != NULL)
							{
								memset(*ip,0,new_len + 1);
								memcpy(*ip,msg,new_len);
								len = new_len;
								new_len = 0;
								ret = 1;
								
								break;
							}
						}
					}
				}
			}
		}
		
		delay_ms(300);
	}
   
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//新建一个SOCKET
u8 bcxx_set_AT_NSOCR(char *type, char *protocol,char *port,u8 execute_times,u32 time_out,u16 wait_time)
{
	unsigned char ret = 255;
	unsigned char buf[3] = {0,0,0};
    bcxx_wait_mode(CMD_MODE);

	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSOCR=%s,%s,%s,1\r\n",type,protocol,port);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				get_str1((u8 *)bcxx_rx_cmd_buf, "\r\n", 1, "\r\n", 2, (u8 *)buf);
				if(strlen((char *)buf) == 1)
				{
					ret = buf[0] - 0x30;
					
					break;
				}
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//关闭一个SOCKET
u8 bcxx_set_AT_NSOCL(u8 socket,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSOCL=%d\r\n",socket);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//向UDP服务器发送数据并等待响应数据
u8 bcxx_set_AT_NSOFT(u8 socket, char *ip,char *port,u16 len,char *inbuf,char *outbuf,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSOST=%d,%s,%s,%d,%s\r\n",socket,ip,port,len,inbuf);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				bcxx_clear_rx_cmd_buffer();
				if(bcxx_wait_cmd2("+NSONMI:",TIMEOUT_15S) == RECEIVED)
				{
					bcxx_clear_rx_cmd_buffer();
					printf("AT+NSORF=%d,%d\r\n",socket,1358);
					if(bcxx_wait_cmd2("OK",TIMEOUT_2S) == RECEIVED)
					{
						get_str1((u8 *)bcxx_rx_cmd_buf, ",", 4, ",", 5, (u8 *)outbuf);

						ret = strlen((char *)outbuf);
						
						break;
					}
				}
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//建立一个TCP连接
u8 bcxx_set_AT_NSOCO(u8 socket, char *ip,char *port,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSOCO=%d,%s,%s\r\n",socket,ip,port);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				ret = 1;
				
				break;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//通过TCP连接发送数据，并等待响应包
u8 bcxx_set_AT_NSOSD(u8 socket, u16 len,char *inbuf,u8 execute_times,u32 time_out,u16 wait_time)
{
	unsigned char ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+NSOSD=%d,%d,%s,0x100,100\r\n",socket,len,inbuf);
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "OK") != -1)
			{
				bcxx_clear_rx_cmd_buffer();
				
				if(bcxx_wait_cmd2("+NSOSTR:",TIMEOUT_15S) == RECEIVED)
				{
					ret = 1;
					
					break;
				}
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//从模块获取时间
u8 bcxx_get_AT_CCLK(char *buf,u8 execute_times,u32 time_out,u16 wait_time)
{
	u8 ret = 0;
    bcxx_wait_mode(CMD_MODE);
	
	while(execute_times --)
	{
		bcxx_clear_rx_cmd_buffer();
		printf("AT+CCLK?\r\n");
		if(bcxx_wait_cmd2("OK",time_out) == RECEIVED)
		{
			if(search_str((u8 *)bcxx_rx_cmd_buf, "+CCLK:") != -1)
			{
				get_str1((u8 *)bcxx_rx_cmd_buf, "+CCLK:", 1, "\r\n\r\nOK", 1, (u8 *)buf);

				ret = 1;
				
				break;
			}
		}
		
		delay_ms(300);
	}
    
    bcxx_mode = NET_MODE;
#ifdef BCXX_PRINTF_RX_BUF
	bcxx_print_rx_buf();
#endif
	
	delay_ms(wait_time);
	
    return ret;
}

//清空AT指令接收缓存
void bcxx_clear_rx_cmd_buffer(void)
{
	uint16_t i;
    for(i = 0; i < CMD_DATA_BUFFER_SIZE; i++)
    {
        bcxx_rx_cmd_buf[i] = 0;
    }
    bcxx_rx_cnt = 0;
}

//清空网络数据缓存
int bcxx_available(void)
{
	return bcxx_net_buf->available(&bcxx_net_buf);
}

void bcxx_print_rx_buf(void)
{
	UsartSendString(USART1,(u8 *)bcxx_rx_cmd_buf,bcxx_rx_cnt);
}

void bcxx_print_cmd(CMD_STATE_E cmd)
{

}

CMD_STATE_E bcxx_wait_cmd1(u32 wait_time)
{
	u32 time = GetSysTick1ms();
	u32 time_now = 0;
    bcxx_cmd_state = WAITING;

    while(1)
    {
		time_now = GetSysTick1ms();
        if((time_now - time) > wait_time)
        {
            bcxx_cmd_state = TIMEOUT;
            break;
        }

        if(
            search_str((u8 *)bcxx_rx_cmd_buf, "OK"   ) != -1  || \
            search_str((u8 *)bcxx_rx_cmd_buf, "FAIL" ) != -1  || \
            search_str((u8 *)bcxx_rx_cmd_buf, "ERROR") != -1
        )
        {
            while(GetSysTick1ms() - bcxx_last_time < 20);
				bcxx_cmd_state = RECEIVED;
			break;
        }
		delay_ms(10);
    }
    bcxx_print_cmd(bcxx_cmd_state);

    return bcxx_cmd_state;
}

CMD_STATE_E bcxx_wait_cmd2(const char *spacial_target, u32 wait_time)
{
	u32 time = GetSysTick1ms();
	u32 time_now = 0;
	bcxx_cmd_state = WAITING;

    while(1)
    {
		time_now = GetSysTick1ms();
        if((time_now - time) > wait_time)
        {
            bcxx_cmd_state = TIMEOUT;
            break;
        }

        else if(search_str((u8 *)bcxx_rx_cmd_buf, (u8 *)spacial_target) != -1)
        {
            while(GetSysTick1ms() - bcxx_last_time < 20);
				bcxx_cmd_state = RECEIVED;
            break;
        }
		else if(bcxx_break_out_wait_cmd == 1)
		{
			bcxx_break_out_wait_cmd = 0;
			break;
		}
		delay_ms(10);
    }
    bcxx_print_cmd(bcxx_cmd_state);

    return bcxx_cmd_state;
}

u8 bcxx_wait_mode(BCXX_MODE_E mode)
{
	u8 ret = 0;

	ret = 1;
	if(GetSysTick1ms() - bcxx_last_time > 20)
	{
		bcxx_mode = mode;
	}
    else
    {
        while(GetSysTick1ms() - bcxx_last_time < 20);
        bcxx_mode = mode;
    }
    return ret;
}

//从串口获取一个字符
void bcxx_get_char(void)
{
	u8 c;

	c = USART_ReceiveData(bcxx_USARTx);
	bcxx_last_time = GetSysTick1ms();

	if(bcxx_mode == CMD_MODE)
	{
		bcxx_rx_cmd_buf[bcxx_rx_cnt] = c;
		if(bcxx_rx_cnt ++ > CMD_DATA_BUFFER_SIZE)
		{
			bcxx_rx_cnt = 0;
		}
		bcxx_cmd_state = RECEIVING;
	}
	bcxx_net_data_state_process(c);
}

void bcxx_uart_interrupt_event(void)
{
	bcxx_get_char();
}

void bcxx_net_data_state_process(char c)
{
	static NET_DATA_STATE_E net_data_state = NEED_PLUS;
	
	net_idle_data_rx_buf[bcxx_net_temp_data_rx_cnt ++] = c;
	
	if(bcxx_net_temp_data_rx_cnt == 128)
	{
		bcxx_net_temp_data_rx_cnt = 0;
	}

	switch((u8)net_data_state)
	{
		case (u8)NEED_PLUS:
			if(c == '+')
			{
				net_data_state  = NEED_N1;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_N1:
			if(c == 'N')
			{
				net_data_state = NEED_S;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_S:
			if(c == 'S')
			{
				net_data_state = NEED_O;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_O:
			if(c == 'O')
			{
				net_data_state = NEED_N2;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_N2:
			if(c == 'N')
			{
				net_data_state = NEED_M;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;
			
		case (u8)NEED_M:
			if(c == 'M')
			{
				net_data_state = NEED_I;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;
			
		case (u8)NEED_I:
			if(c == 'I')
			{
				net_data_state = NEED_MAO;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_MAO:
			if(c == ':')
			{
				net_data_state = NEED_ID_DATA;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;
			
		case (u8)NEED_ID_DATA:
			if(c >= '0' && c <= '9')
			{
				
			}
			else if(c == ',')
			{
				net_data_state = NEED_LEN_DATA;
			}
			else
			{
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_LEN_DATA:
			if(c >= '0' && c <= '9')
			{
				net_temp_data_rx_buf[bcxx_net_temp_data_rx_cnt++] = c;
			}
			else if(c == ',')
			{
				net_temp_data_rx_buf[bcxx_net_temp_data_rx_cnt++] = '\0';
				bcxx_net_data_len = atoi((const char *)net_temp_data_rx_buf);
				bcxx_net_data_len *= 2;
				net_data_state = NEED_USER_DATA;
				bcxx_net_temp_data_rx_cnt = 0;
			}
			else
			{
				bcxx_net_temp_data_rx_cnt = 0;
				net_data_state = NEED_PLUS;
			}
		break;

		case (u8)NEED_USER_DATA:
			if(bcxx_net_temp_data_rx_cnt < bcxx_net_data_len)
			{
				bcxx_net_buf->write(&bcxx_net_buf,c);
				
				bcxx_net_temp_data_rx_cnt ++;
			}
			else
			{
				bcxx_net_temp_data_rx_cnt = 0;
				net_data_state = NEED_PLUS;
			}
		break;
			
		default:
			net_data_state = NEED_PLUS;
			bcxx_net_temp_data_rx_cnt = 0;
		break;
	}
}







































