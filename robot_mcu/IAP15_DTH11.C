
/*************	����˵��	**************
����ң�ض˷�����ָ����ж�����Ƽ���ʪ�Ȳɼ��ش�
T1ʵ������16λ����PWM��T0ʵ������16λ����PWMʱ�������Σ����ѡ��T1��
TO��ʱ
T2��Ϊ����1�Ĳ����ʷ�����

��������Ϊ��9600,8,n,1.


******************************************/
#include "config0.H"
#include "delay.h"
#include "dth.h"
/*************	���س�������	**************/	//													
#define 	MAIN_Fosc		24000000L	//������ʱ��
#define		RX1_Lenth		5			//���ڽ��ջ��峤��
#define		RX2_Lenth		5			//���ڽ��ջ��峤��
#define 	S2RI  0x01              	//S2CON.0
#define 	S2TI  0x02           	  	 //S2CON.1
#define 	S2_S0 0x01           	   //P_SW2.0
#define		BaudRate1		9600L	//ѡ������
#define	Timer2_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate1))		//Timer 2 ��װֵ��

#define step_period 2        //���岽������������    
#define step_motor_right_step_pin P00
#define step_motor_right_dir_pin P01
#define step_motor_left_step_pin P02
#define step_motor_left_dir_pin	P03
#define motor_en_right P04
#define motor_en_left P05
#define sw1 P06

/*************	���ر�������	**************/
u8 xx = 0;
u8 numberaa[] = {'0','1','2','3','4','5','6','7','8','9'};
u8	idata RX1_Buffer[RX1_Lenth];	//���ջ���
u8	idata RX2_Buffer[RX2_Lenth];	//���ջ���
u8	TX1_Cnt = 0;	//���ͼ���
u8	RX1_Cnt = 0;	//���ռ���
bit	B_TX1_Busy; 	//����æ��־
bit uart2_busy = 0; 
bit test = 0;

bit step_motor_left_flag = 0;
bit step_motor_right_flag = 0;

bit turn_flag = 0;//��ת����ת��־λ

bit dth11_flag = 0;
u8 uart_flag = 0; 
u8 Int0_flag = 0;
u16 step_t_1ms = 0;

bit right_dir = 0;//�Ҳ���������з����־λ
bit left_dir = 0;//�󲽽�������з����־λ

u16 left_step_set = 0;//�������в�������
u16 right_step_set = 0;//�ҵ�����в�������

u16 left_step_temp = 0;//�������в�����ʱֵ
u16 right_step_temp = 0;//�ҵ�����в�����ʱֵ
u8 uart_motor_com = 0;	 //uart���տ����ӻ�����

u8 flag = 0;

u8 step_clock = 1; //��������
/*************	���غ�������	**************/
void UART_send_byte(u8 dat);
void UART_send_string(u8 *buf);

void UART2_SendData(u8 dat);
void UART2_SendString(u8 *s);
void Timer0Init(void);		//1����@24.000MHz
void handle(void);
void step_motor_control(u8 motor,u16 step,u8 dir);//����������ƺ���
void Delay1ms(u16 x);		//@24.000MHz
void Delay1us(void);		//@24.000MHz
void clock_change(void);//�����л�����
/**********************************************/
void main(void)
{  


	SCON  = (SCON & 0x3f) | 0x40;//UART1 8λ�ɱ䲨����
	S2CON = 0x50;                //UART2 8λ�ɱ䲨����				//8λ����
//	P_SW1 &= ~0xc0;			//UART1 ʹ��P30 P31��	Ĭ��	
	P_SW1 = (P_SW1 & ~0xc0) | 0x40;	//UART1 ʹ��P36 P37��
//	P_SW1 = (P_SW1 & ~0xc0) | 0x80	//UART1 ʹ��P16 P17��
	
	P_SW2 &= ~S2_S0;            //S2_S0=0 UART2 ʹ��(P1.0/RxD2, P1.1/TxD2)

	AUXR &= ~(1<<4);	//Timer stop		������ʹ��Timer2����
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	T2H = (u8)(Timer2_Reload >> 8);
	T2L = (u8)Timer2_Reload;
	AUXR |=  (1<<4);	//Timer run enable
	REN = 1;	//��������
	PS = 1;//����������ȼ�
	ES  = 1;	//ʹ�ܴ���1�ж�
	//IE2 = 0x01; //ʹ�ܴ���2�ж�

	Timer0Init();
	EA = 1;
	motor_en_left = step_clock;
	motor_en_right = step_clock;	
   //UART_send_string("STC15F2K60S2\r\nUart1 Test !\r\n");
	while (1)
	{
			clock_change();//�Ƿ����л�����
			if(uart_flag == 1)
			{
				uart_flag = 0;
				 handle();
			}
		
		
	}
}

/********************* ��ʱ��0��ʼ������************************/
void Timer0Init(void)		//1����@24.000MHz
{
	AUXR |= 0x80;		//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0x40;		//���ö�ʱ��ֵ
	TH0 = 0xA2;		//���ö�ʱ��ֵ
	TF0 = 0;		//���TF0��־
	PT0 = 0;
	ET0 = 1;
	EA = 1;
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	

}
/********************* ��ʱ��0�жϺ���************************/
void Timer0_interrupt(void) interrupt 1
{
	step_t_1ms++; //���������ʱ����
	if(step_t_1ms == step_period)		  //��������������� 4ms
	{
		//UART_send_string("STC15F2K60S2\r\nUart1 Test !\r\n");
		step_t_1ms = 0;
		
		//��ת�����������
		if(step_motor_right_flag == 1) 
		{
			if(turn_flag == 1) //�����ת��״̬
			{
				motor_en_left = 0;//��������������������������ֹ��ת��ʱС���ƶ���
			}
			
			step_motor_right_dir_pin = right_dir;  //�Ҳ���������з����ж�
		    if(right_step_temp < right_step_set)
			{
				motor_en_right = 0;//ʹ���ҵ��������
			    
				right_step_temp++;
				step_motor_right_step_pin = 1;
				Delay1us();
				step_motor_right_step_pin = 0;
			
			
			}
			else
			{
				step_motor_right_flag = 0;
				right_step_temp = 0;
				motor_en_right = step_clock;//�ҵ��������ʧЧ����ֹ�������̺ĵ�
				if(turn_flag == 1) //ת����ɺ�ȡ�������������״̬
				{
					 turn_flag = 0;//�˳�ת��״̬
					 motor_en_left = 1;//�󣨶������������ʧЧ������������
				}
			}
		}
		
		//�����������
		if(step_motor_left_flag == 1) 
		{
			
			step_motor_left_dir_pin = left_dir;  //�󲽽�������з����ж�
		    if(left_step_temp < left_step_set)
			{
				motor_en_left = 0;//ʹ������������
				left_step_temp++;
				step_motor_left_step_pin = 1;
				Delay1us();
				step_motor_left_step_pin = 0;
			}
			else
			{
				step_motor_left_flag = 0;
				left_step_temp =0;
				motor_en_left = step_clock;//����������ʧЧ����ֹ�������̺ĵ�
			}
		}

	}
}
/********************* UART1�жϺ���************************/
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
/********************* UART2�жϺ���************************/
void Uart2() interrupt 8 //using 1
{
    if (S2CON & S2RI)
    {
        S2CON &= ~S2RI;         //���S2RIλ
        RX2_Buffer[0] = S2BUF;            
		test = 1;
    }
    if (S2CON & S2TI)
    {
        S2CON &= ~S2TI;         //���S2TIλ
		test = 0;
        uart2_busy = 0;               //��æ��־
    }
}
/*
 * UART1 ����һ�ֽ�
*/
void UART_send_byte(u8 dat)
{
	SBUF = dat;
	while (TI == 0);  //�ȴ��������
	TI = 0;
}

/*
 * UART1 �����ַ���
*/
void UART_send_string(u8 *buf) //�����β���ָ�������Ҳ����˵ʵ�α����ǵ�ַ
{
	while (*buf != '\0')//����C�����еģ�һ���ַ������ݵĽ�βϵͳ�����һ��'\0'���������е���߾ͻ���ֹ
	{
		UART_send_byte(*buf++);
	}
}
 /*----------------------------
����2�����ֽ�����
----------------------------*/
void UART2_SendData(u8 dat)
{
    S2BUF = dat;                //д���ݵ�UART2���ݼĴ���
	uart2_busy = 1;
	while (uart2_busy);               //�ȴ�ǰ������ݷ������
    
}

/*----------------------------
����2�����ַ���
----------------------------*/
void UART2_SendString(u8 *s)
{
    while (*s)                  //����ַ���������־
    {
        UART2_SendData(*s++);         //���͵�ǰ�ַ�
    }
}
/************************************************
����������
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
					case 'm':step_motor_control(1,2000,1);step_motor_control(0,2000,0);break;//��������5Ȧ��ǰ��	step(0,1,1);
					case 'n':step_motor_control(1,2000,0);step_motor_control(0,2000,1);break;//�������У�����	step(0,1,0);
					case 'q':step_motor_control(1,0,1);step_motor_control(0,0,1);break;//���ҵ��ͬʱֹͣ����break;//����ֹͣ����  step(0,0,1);

					case 'o':step_motor_control(0,100,1);turn_flag = 1;break;//�ҵ�����У���ת
					case 'p':step_motor_control(0,100,0);turn_flag = 1;break;//�ҵ�����У���ת
			
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
����������ƺ���
motor      : 1, left motor control
	         0, rihgt motor control

step       : step number

dir        : 1, go
	         0, back;
---------------------------*/
void step_motor_control(u8 motor,u16 step,u8 dir)
{
	//���� ����
	if(motor == 1)
	{
		step_motor_left_flag = 1;
		left_step_set = step;
		left_dir = dir;
	}
	//�ҵ�� ����
	else
	{
		step_motor_right_flag = 1;
		right_step_set = step;
		right_dir = dir;
	}

}

//������������л������������Ƿ���Ч�л�������
void clock_change(void)
{
	if(sw1 == 1)
	{
		step_clock = 1;//Ĭ�ϲ���������������enʧЧ����ֹ����	
	}
	else
	{
		step_clock = 0;//��������������Ч��	
	}	
}
