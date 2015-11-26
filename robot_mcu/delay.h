#ifndef __DELAY_H__
#define __DELAY_H__
#include <config0.H>
//========================================================================
// 函数: void  delay_ms(u8 ms)
// 描述: 延时函数。
// 参数: ms,要延时的ms数, 这里只支持1~255ms. 自动适应主时钟.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
extern void  delay_ms(unsigned char ms);

extern void DHT11Delay_10us();		//@24.000MHz

#endif 