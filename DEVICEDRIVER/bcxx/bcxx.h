#ifndef __BCXX_H
#define __BCXX_H

#include "sys.h"
#include <stdint.h>
#include <string.h>
//#include "internal.h"


#define BCXX_RST_HIGH		GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define BCXX_RST_LOW		GPIO_ResetBits(GPIOC,GPIO_Pin_2)

#define BCXX_PWREN_HIGH		GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define BCXX_PWREN_LOW		GPIO_ResetBits(GPIOC,GPIO_Pin_3)


#define BCXX_PRINTF_RX_BUF


#define TIMEOUT_1S 100
#define TIMEOUT_2S 200
#define TIMEOUT_3S 300
#define TIMEOUT_4S 400
#define TIMEOUT_5S 500
#define TIMEOUT_6S 600
#define TIMEOUT_7S 700
#define TIMEOUT_10S 1000
#define TIMEOUT_11S 1000
#define TIMEOUT_12S 1000
#define TIMEOUT_13S 1000
#define TIMEOUT_14S 1000
#define TIMEOUT_15S 1500
#define TIMEOUT_20S 2000
#define TIMEOUT_25S 2500
#define TIMEOUT_30S 3000
#define TIMEOUT_35S 3500
#define TIMEOUT_40S 4000
#define TIMEOUT_45S 4500
#define TIMEOUT_50S 5000
#define TIMEOUT_55S 5500
#define TIMEOUT_60S 6000
#define TIMEOUT_65S 6500
#define TIMEOUT_70S 7000
#define TIMEOUT_75S 7500
#define TIMEOUT_80S 8000
#define TIMEOUT_85S 8500
#define TIMEOUT_90S 9000
#define TIMEOUT_95S 9500
#define TIMEOUT_100S 10000
#define TIMEOUT_105S 10500
#define TIMEOUT_110S 11000
#define TIMEOUT_115S 11500
#define TIMEOUT_120S 12000
#define TIMEOUT_125S 12500
#define TIMEOUT_130S 13000
#define TIMEOUT_135S 13500
#define TIMEOUT_140S 14000
#define TIMEOUT_145S 14500
#define TIMEOUT_150S 15000
#define TIMEOUT_155S 15500
#define TIMEOUT_160S 16000
#define TIMEOUT_165S 16500
#define TIMEOUT_170S 17000
#define TIMEOUT_175S 17500
#define TIMEOUT_180S 18000


typedef enum
{
    UNKNOW_STATE 	= 255,	//获取连接状态失败
    GET_READY 		= 0,	//就绪状态
    NEED_CLOSE 		= 1,	//需要关闭移动场景
    NEED_WAIT 		= 2,	//需要等待
	ON_SERVER 		= 4,	//连接建立成功
	DISCONNECT		= 5,	//连接已断开
} CONNECT_STATE_E;

typedef enum
{
	MIPL_DEBUG_LEVEL_NONE = 0,
	MIPL_DEBUG_LEVEL_RXL,
	MIPL_DEBUG_LEVEL_RXL_RXD,
	MIPL_DEBUG_LEVEL_TXL_TXD,
} MIPL_DEBUG_LEVEL_E;

//接收数据的状态
typedef enum
{
    WAITING = 0,
    RECEIVING = 1,
    RECEIVED  = 2,
    TIMEOUT = 3
} CMD_STATE_E;


#define MIPL_BLOCK1  		5
#define MIPL_BLOCK2  		5
#define MIPL_BLOCK2TH  		2
#define MIPL_PORT  			0
#define MIPL_KEEPALIVE  	300
#define MIPL_DEBUG  		MIPL_DEBUG_LEVEL_NONE
#define MIPL_BOOT  			0
#define MIPL_ENCRYPT  		0

#define NBIOT_SOCK_BUF_SIZE 1024

typedef struct
{
	uint8_t boot;
	uint8_t encrypt;
	MIPL_DEBUG_LEVEL_E debug;
	uint16_t port;
	uint32_t keep_alive;
	size_t uri_len;
	const char* uri;
	size_t ep_len;
	const char* ep;
	uint8_t block1;		//COAP option BLOCK1(PUT or POST),0-6. 2^(4+n)  bytes
	uint8_t block2;		//COAP option BLOCK2(GET),0-6. 2^(4+n)  bytes
	uint8_t block2th;	//max size to trigger block-wise operation,0-2. 2^(8+n) bytes
} MIPL_T;


extern char cmd_rx_buff[256];

void bcxx_hard_init(void);
void bcxx_soft_init(void);

uint32_t ip_SendData(int8_t *buf, uint32_t len);

void netif_rx(uint8_t *buf,uint16_t *read);



unsigned char bcxx_set_AT_ATE(char cmd);
unsigned char bcxx_set_NATSPEED(u32 baud_rate);
unsigned char bcxx_set_AT_CFUN(char cmd);
unsigned char bcxx_set_AT_NBAND(unsigned char *imsi);
unsigned char bcxx_get_AT_CSQ(void);
unsigned char bcxx_get_AT_CGSN(void);
unsigned char bcxx_get_AT_NCCID(void);
unsigned char bcxx_get_AT_CIMI(void);
unsigned char bcxx_set_AT_CELL_RESELECTION(void);
unsigned char bcxx_set_AT_NRB(void);
unsigned char bcxx_set_AT_NCDP(char *addr, char *port);
unsigned char bcxx_set_AT_CSCON(unsigned char cmd);
unsigned char bcxx_set_AT_CEREG(unsigned char cmd);
unsigned char bcxx_set_AT_NNMI(unsigned char cmd);
unsigned char bcxx_set_AT_NSONMI(unsigned char cmd);
unsigned char bcxx_set_AT_CGATT(unsigned char cmd);
unsigned char bcxx_set_AT_QREGSWT(unsigned char cmd);
unsigned char bcxx_set_AT_CEDRXS(unsigned char cmd);
unsigned char bcxx_set_AT_CPSMS(unsigned char cmd);
unsigned char bcxx_set_AT_NMGS(unsigned int len,char *buf);
unsigned char bcxx_get_AT_CGPADDR(char **ip);
unsigned char bcxx_set_AT_NSOCR(char *type, char *protocol,char *port);
unsigned char bcxx_set_AT_NSOFT(unsigned char socket, char *ip,char *port,unsigned int len,char *inbuf);
unsigned char bcxx_set_AT_NSOCL(unsigned char socket);
unsigned char bcxx_set_AT_NSOCO(unsigned char socket, char *ip,char *port);
unsigned char bcxx_set_AT_NSOSD(unsigned char socket, unsigned int len,char *inbuf);
unsigned char bcxx_get_AT_CCLK(char *buf);






























#endif
