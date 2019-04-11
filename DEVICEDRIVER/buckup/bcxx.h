#ifndef __BCXX_H
#define __BCXX_H

#include "sys.h"
#include "ringbuf.h"


#define BCXX_RST_HIGH		GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define BCXX_RST_LOW		GPIO_ResetBits(GPIOC,GPIO_Pin_2)

#define BCXX_PWREN_HIGH		GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define BCXX_PWREN_LOW		GPIO_ResetBits(GPIOC,GPIO_Pin_3)


#define BCXX_PRINTF_RX_BUF

#define CMD_DATA_BUFFER_SIZE 256
#define NET_DATA_BUFFER_SIZE 1500

#define TIMEOUT_1S 1000
#define TIMEOUT_2S 2000
#define TIMEOUT_5S 5000
#define TIMEOUT_6S 6000
#define TIMEOUT_7S 7000
#define TIMEOUT_10S 10000
#define TIMEOUT_11S 10000
#define TIMEOUT_12S 10000
#define TIMEOUT_13S 10000
#define TIMEOUT_14S 10000
#define TIMEOUT_15S 15000
#define TIMEOUT_20S 20000
#define TIMEOUT_25S 25000
#define TIMEOUT_30S 30000
#define TIMEOUT_35S 35000
#define TIMEOUT_40S 40000
#define TIMEOUT_45S 45000
#define TIMEOUT_50S 50000
#define TIMEOUT_55S 55000
#define TIMEOUT_60S 60000
#define TIMEOUT_65S 65000
#define TIMEOUT_70S 70000
#define TIMEOUT_75S 75000
#define TIMEOUT_80S 80000
#define TIMEOUT_85S 85000
#define TIMEOUT_90S 90000
#define TIMEOUT_95S 95000
#define TIMEOUT_100S 100000
#define TIMEOUT_105S 105000
#define TIMEOUT_110S 110000
#define TIMEOUT_115S 115000
#define TIMEOUT_120S 120000
#define TIMEOUT_125S 125000
#define TIMEOUT_130S 130000
#define TIMEOUT_135S 135000
#define TIMEOUT_140S 140000
#define TIMEOUT_145S 145000
#define TIMEOUT_150S 150000
#define TIMEOUT_155S 155000
#define TIMEOUT_160S 160000
#define TIMEOUT_165S 165000
#define TIMEOUT_170S 170000
#define TIMEOUT_175S 175000
#define TIMEOUT_180S 180000

#define IMEI_LEN	15

#define NET_NULL 0x00000000

typedef enum
{
	NEED_PLUS = 0,
	NEED_N1,
	NEED_S,
	NEED_O,
	NEED_N2,
	NEED_M,
	NEED_I,
	NEED_MAO,
	NEED_DOU,
	NEED_LEN_DATA,
	NEED_ID_DATA,
	NEED_USER_DATA,
} NET_DATA_STATE_E;

//接收数据的状态
typedef enum
{
    WAITING = 0,
    RECEIVING = 1,
    RECEIVED  = 2,
    TIMEOUT = 3
} CMD_STATE_E;

//当前的模式
typedef enum
{
    NET_MODE = 0,
    CMD_MODE = 1,
} BCXX_MODE_E;

//连接状态
typedef enum
{
    UNKNOW_STATE 	= 255,	//获取连接状态失败
	GOT_IP 			= 0,	//已获得IP
    NEED_CLOSE 		= 1,	//需要关闭移动场景
    NEED_WAIT 		= 2,	//需要等待
	ON_SERVER 		= 3,	//连接建立成功
	DISCONNECT 		= 4,	//连接已断开
} CONNECT_STATE_E;


extern CONNECT_STATE_E ConnectState;	//连接状态

extern u8 net_idle_data_rx_buf[128];

extern char 			bcxx_rx_cmd_buf[CMD_DATA_BUFFER_SIZE];
extern u16  			bcxx_rx_cnt;
extern pRingBuf     	bcxx_net_buf;
extern u16 				bcxx_net_data_rx_cnt;
extern u16  			bcxx_net_data_len;
extern u8   			bcxx_break_out_wait_cmd;
extern u8   			bcxx_init_ok;
extern u32    			bcxx_last_time;
extern CMD_STATE_E 		bcxx_cmd_state;
extern BCXX_MODE_E 		bcxx_mode;
extern USART_TypeDef* 	bcxx_USARTx;
extern char   			*bcxx_imei;



void			bcxx_init(void);
void 			bcxx_hard_reset(void);


u16 	bcxx_read(u8 *buf);


u8 	bcxx_set_AT_NRB(u8 execute_times,u32 time_out,u16 wait_time);//TIMEOUT_10S;
u8	bcxx_set_AT(u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_ATE(char cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_UART(u32 baud_rate,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_get_AT_CSQ(u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_CFUN(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_NBAND(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8  bcxx_get_AT_CGSN(u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_NCDP(char *addr, char *port,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_CSCON(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_CEREG(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_NNMI(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_CGATT(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8  bcxx_set_AT_QREGSWT(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_CEDRXS(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_CPSMS(u8 cmd,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_NSONMI(u8 mode,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_CELL_RESELECTION(u8 execute_times,u32 time_out,u16 wait_time);
u8  bcxx_get_AT_CGPADDR(char **ip,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_NMGS(u16 len,char *buf,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_get_AT_CCLK(char *buf,u8 execute_times,u32 time_out,u16 wait_time);

u8	bcxx_set_AT_NSOCR(char *type, char *protocol,char *port,u8 execute_times,u32 time_out,u16 wait_time);
u8	bcxx_set_AT_NSOCL(u8 socket,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_NSOFT(u8 socket, char *ip,char *port,u16 len,char *inbuf,char *outbuf,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_NSOCO(u8 socket, char *ip,char *port,u8 execute_times,u32 time_out,u16 wait_time);
u8 	bcxx_set_AT_NSOSD(u8 socket, u16 len,char *inbuf,u8 execute_times,u32 time_out,u16 wait_time);

void        	bcxx_clear_rx_cmd_buffer(void);
int	 			bcxx_available(void);

void        	bcxx_print_rx_buf(void);
void        	bcxx_print_cmd(CMD_STATE_E cmd);
CMD_STATE_E 	bcxx_wait_cmd1(u32 wait_time);
CMD_STATE_E 	bcxx_wait_cmd2(const char *spacial_target, u32 wait_time);
u8   			bcxx_wait_mode(BCXX_MODE_E mode);
void 			bcxx_get_char(void);
void 			bcxx_uart_interrupt_event(void);
void 			bcxx_net_data_state_process(char c);


































#endif
