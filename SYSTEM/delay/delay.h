#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"
#include <time.h>

void xPortSysTickHandler( void );

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

void nbiot_sleep(int milliseconds);
time_t nbiot_time(void);

#endif





























