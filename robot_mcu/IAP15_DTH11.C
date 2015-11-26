
/*************	功能说明	**************
接收遥控端发来的指令进行舵机控制及温湿度采集回传
T1实现软件16位软件PWM（T0实现软件16位软件PWM时不可屏蔽，因此选择T1）
TO定时
T2作为串口1的波特率发生器

串口设置为：9600,8,n,1.


******************************************/
#include "config0.H"
#include "delay.h"
#include "dth.h"
/*************	本地常量声明	**************/	//													
#define 	MAIN_Fosc		24000000L	//定义主时钟
#define		RX1_Lenth		5			//串口接收缓冲长度
#define		RX2_Lenth		5			//串口接收缓冲长度
#define 	S2RI  0x01              	//S2CON.0
#define 	S2TI  0x02           	  	 //S2CON.1
#define 	S2_S0 0x01           	   //P_SW2.0
#define		BaudRate1		9600L	//选择波特率
#define	Timer2_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate1))		//Timer 2 重装值，

#define step_period 2        //定义步进电机输出脉冲    
#define step_motor_right_step_pin P00
#define step_motor_right_dir_pin P01
#define step_motor_left_step_pin P02
#define step_motor_left_dir_pin	P03
#define motor_en_right P04
#define motor_en_left P05
#define sw1 P06

/*************	本地变量声明	**************/
u8 xx = 0;
u8 numberaa[] = {'0','1','2','3','4','5','6','7','8','9'};
u8	idata RX1_Buffer[RX1_Lenth];	//接收缓冲
u8	idata RX2_Buffer[RX2_Lenth];	//接收缓冲
u8	TX1_Cnt = 0;	//发送计数
u8	RX1_Cnt = 0;	//接收计数
bit	B_TX1_Busy; 	//发送忙标志
bit uart2_busy = 0; 
bit test = 0;

bit step_motor_left_flag = 0;
bit step_motor_right_flag = 0;

bit turn_flag = 0;//左转或右转标志位

bit dth11_flag = 0;
u8 uart_flag = 0; 
u8 Int0_flag = 0;
u16 step_t_1ms = 0;

bit right_dir = 0;//右步进电机运行方向标志位
bit left_dir = 0;//左步进电机运行方向标志位

u16 left_step_set = 0;//左电机运行步数设置
u16 right_step_set = 0;//右电机运行步数设置

u16 left_step_temp = 0;//左电机运行步数临时值
u16 right_step_temp = 0;//右电机运行步数临时值
u8 uart_motor_com = 0;	 //uart接收控制子缓冲区

u8 flag = 0;

u8 step_clock = 1; //抱死变量
/*************	本地函数声明	**************/
void UART_send_byte(u8 dat);
void UART_send_string(u8 *buf);

void UART2_SendData(u8 dat);
void UART2_SendString(u8 *s);
void Timer0Init(void);		//1毫秒@24.000MHz
void handle(void);
void step_motor_control(u8 motor,u16 step,u8 dir);//步进电机控制函数
void Delay1ms(u16 x);		//@24.000MHz
void Delay1us(void);		//@24.000MHz
void clock_change(void);//抱死切换函数
/**********************************************/
void main(void)
{  


	SCON  = (SCON & 0x3f) | 0x40;//UART1 8位可变波特率
	S2CON = 0x50;                //UART2 8位可变波特率				//8位数据
//	P_SW1 &= ~0xc0;			//UART1 使用P30 P31口	默认	
	P_SW1 = (P_SW1 & ~0xc0) | 0x40;	//UART1 使用P36 P37口
//	P_SW1 = (P_SW1 & ~0xc0) | 0x80	//UART1 使用P16 P17口
	
	P_SW2 &= ~S2_S0;            //S2_S0=0 UART2 使用(P1.0/RxD2, P1.1/TxD2)

	AUXR &= ~(1<<4);	//Timer stop		波特率使用Timer2产生
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	T2H = (u8)(Timer2_Reload >> 8);
	T2L = (u8)Timer2_Reload;
	AUXR |=  (1<<4);	//Timer run enable
	REN = 1;	//允许接收
	PS = 1;//串口最高优先级
	ES  = 1;	//使能串口1中断
	//IE2 = 0x01; //使能串口2中断

	Timer0Init();
	EA = 1;
	motor_en_left = step_clock;
	motor_en_right = step_clock;	
   //UART_send_string("STC15F2K60S2\r\nUart1 Test !\r\n");
	while (1)
	{
			clock_change();//是否抱死切换函数
			if(uart_flag == 1)
			{
				uart_flag = 0;
				 handle();
			}
		
		
	}
}

/********************* 定时器0初始化函数************************/
void Timer0Init(void)		//1毫秒@24.000MHz
{
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x40;		//设置定时初值
	TH0 = 0xA2;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	PT0 = 0;
	ET0 = 1;
	EA = 1;
	TR0 = 1;		//定时器0开始计时
	

}
/********************* 定时器0中断函数************************/
void Timer0_interrupt(void) interrupt 1
{
	step_t_1ms++; //步进电机计时变量
	if(step_t_1ms == step_period)		  //步进电机脉冲周期 4ms
	{
		//UART_send_string("STC15F2K60S2\r\nUart1 Test !\r\n");
		step_t_1ms = 0;
		
		//右转向电机电机运行
		if(step_motor_right_flag == 1) 
		{
			if(turn_flag == 1) //如果在转向状态
			{
				motor_en_left = 0;//激活左电机驱动，抱死左电机，防止在转向时小球移动，
			}
			
			step_motor_right_dir_pin = right_dir;  //右步进电机运行方向判断
		    if(right_step_temp < right_step_set)
			{
				motor_en_right = 0;//使能右电机驱动板
			    
				right_step_temp++;
				step_motor_right_step_pin = 1;
				Delay1us();
				step_motor_right_step_pin = 0;
			
			
			}
			else
			{
				step_motor_right_flag = 0;
				right_step_temp = 0;
				motor_en_right = step_clock;//右电机驱动板失效，防止抱死发烫耗电
				if(turn_flag == 1) //转向完成后，取消动力电机抱死状态
				{
					 turn_flag = 0;//退出转向状态
					 motor_en_left = 1;//左（动力）电机驱动失效，不抱死左电机
				}
			}
		}
		
		//左电机电机运行
		if(step_motor_left_flag == 1) 
		{
			
			step_motor_left_dir_pin = left_dir;  //左步进电机运行方向判断
		    if(left_step_temp < left_step_set)
			{
				motor_en_left = 0;//使能左电机驱动板
				left_step_temp++;
				step_motor_left_step_pin = 1;
				Delay1us();
				step_motor_left_step_pin = 0;
			}
			else
			{
				step_motor_left_flag = 0;
				left_step_temp =0;
				motor_en_left = step_clock;//左电机驱动板失效，防止抱死发烫耗电
			}
		}

	}
}
/********************* UART1中断函数************************/
void UART1_int (void) interrupt 4
{
	if(RI)
	{
		RI = 0;
		ES = 0;
	RX1_Buffer[RX1_Cnt] = SBUF;
		RX1_Cnt++;
		if(RX1_Cnt==3)	
		{
			if((RX1_Buffer[0]=='x') && ( (RX1_Buffer[1]=='y')||(RX1_Buffer[1]=='z') ) )
			{
				RX1_Cnt= 0;
				uart_flag = 1;
			}
			else
			{}
		}
		ES = 1;
	}

}
/********************* UART2中断函数************************/
void Uart2() interrupt 8 //using 1
{
    if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         //清除S2RI位
        RX2_Buffer[0] = S2BUF;            
		test = 1;
    }
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;         //清除S2TI位
		test = 0;
        uart2_busy = 0;               //清忙标志
    }
}
/*
 * UART1 发送一字节
*/
void UART_send_byte(u8 dat)
{
	SBUF = dat;
	while (TI == 0);  //等待发送完成
	TI = 0;
}

/*
 * UART1 发送字符串
*/
void UART_send_string(u8 *buf) //函数形参是指针变量，也就是说实参必须是地址
{
	while (*buf != '\0')//这是C语言中的，一行字符串数据的结尾系统都会加一个'\0'，程序运行到这边就会终止
	{
		UART_send_byte(*buf++);
	}
}
 /*----------------------------
串口2发送字节数据
----------------------------*/
void UART2_SendData(u8 dat)
{
    S2BUF = dat;                //写数据到UART2数据寄存器
	uart2_busy = 1;
	while (uart2_busy);               //等待前面的数据发送完成
    
}

/*----------------------------
串口2发送字符串
----------------------------*/
void UART2_SendString(u8 *s)
{
    while (*s)                  //检测字符串结束标志
    {
        UART2_SendData(*s++);         //发送当前字符
    }
}
/************************************************
主处理程序
************************************************/
void handle(void)
{
//   	if(step_motor_time_flag ==1)
//	{
//		step_motor_time_flag = 0;
//	
//	}
	if(RX1_Buffer[0]=='x')
	{
			if(RX1_Buffer[1]=='y')
			{
			 	RX1_Buffer[0] = 0;
				RX1_Buffer[1] = 0;
				uart_motor_com= RX1_Buffer[2];
				//RX1_Buffer[2] = 0;

				switch(uart_motor_com)
				{
					case 'm':step_motor_control(1,2000,1);step_motor_control(0,2000,0);break;//左电机运行5圈：前进	step(0,1,1);
					case 'n':step_motor_control(1,2000,0);step_motor_control(0,2000,1);break;//左电机运行：后推	step(0,1,0);
					case 'q':step_motor_control(1,0,1);step_motor_control(0,0,1);break;//左右电机同时停止运行break;//左电机停止运行  step(0,0,1);

					case 'o':step_motor_control(0,100,1);turn_flag = 1;break;//右电机运行：左转
					case 'p':step_motor_control(0,100,0);turn_flag = 1;break;//右电机运行：右转
			
					default: break;
				} 
				
				if(RX1_Buffer[2]=='z')
				{
					dth11_flag = 1;
					if(dth11_flag == 1)
					{
					dth11_flag = 0;
					xx = DHT11_ReadTempAndHumi();
						if(xx!=0)
						{
						
						UART_send_byte(numberaa[temp_value/100]);
						UART_send_byte(numberaa[temp_value % 100/ 10]);
						UART_send_byte(numberaa[humi_value/100]);
						UART_send_byte(numberaa[humi_value % 100/ 10]);
						
						}
						else
						{
						//	UART_send_string("ERRER!:");
						}
					}
				}	
				RX1_Buffer[2] = 0;
				
//			else if(RX1_Buffer[1]=='z')
//			{
//				dth11_flag = 1;
//				if(dth11_flag == 1)
//				{
//					dth11_flag = 0;
//					xx = DHT11_ReadTempAndHumi();
//					if(xx!=0)
//					{
//					    
//						UART_send_byte(numberaa[temp_value/100]);
//						UART_send_byte(numberaa[temp_value % 100/ 10]);
//						UART_send_byte(numberaa[humi_value/100]);
//						UART_send_byte(numberaa[humi_value % 100/ 10]);
//
//					}
//					else
//					{
//					//	UART_send_string("ERRER!:");
//					}
//				}
			}

	}
	else
	{
	
	}
}
void Delay1ms(u16 x)		//@24.000MHz
{
	unsigned char i, j;
	while(x--)
	{
	  	i = 24;
		j = 85;
		do
		{
			while (--j);
		} while (--i);
	}

}
void Delay1us(void)		//@24.000MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	i = 3;
	while (--i);
}
/*----------------------------
步进电机控制函数
motor      : 1, left motor control
	         0, rihgt motor control

step       : step number

dir        : 1, go
	         0, back;
---------------------------*/
void step_motor_control(u8 motor,u16 step,u8 dir)
{
	//左电机 控制
	if(motor == 1)
	{
		step_motor_left_flag = 1;
		left_step_set = step;
		left_dir = dir;
	}
	//右电机 控制
	else
	{
		step_motor_right_flag = 1;
		right_step_set = step;
		right_dir = dir;
	}

}

//步进电机抱死切换函数（驱动是否有效切换函数）
void clock_change(void)
{
	if(sw1 == 1)
	{
		step_clock = 1;//默认不报死，（驱动板en失效）防止发烫	
	}
	else
	{
		step_clock = 0;//报死（驱动板起效）	
	}	
}

