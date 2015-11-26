#ifndef __CONFIG0_H__
#define __CONFIG0_H__

/*************************************
*串口1使用定时器2作为波特率发生器
*************************************/
#include<STC15F2K60S2.H>
#include<intrins.h>
#define MAIN_Fosc	24000000L	//定义主时钟
/*************	本地变量声明	**************/

typedef 	unsigned char	u8;
typedef 	unsigned int	u16;
typedef 	unsigned long	u32;



/************* 	标志位声明  	**************/
extern bit dth11_flag;
extern bit _flag;

#endif

