#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//IIC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2019/9/18
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO方向设置

/*	
SDA:PB7
SCL:PB6

修改方式：
	查看你需要使用的IIC引脚
	SDA_IN：设置SDA引脚为输入模式，这里通过直接修改寄存器的方式，更快
					引脚为0~7时，操作GPIOx->CRL寄存器，每四位对应一个IO口,先清除对应位的设置，再按照输入(1000，即8)或者输出(0011,即3)来设置
					引脚位8~15时，操作GPIOx->CRH寄存器
*/

#define SCL_GPIO_PORT	GPIOB
#define SCL_GPIO_PIN 6
#define SDA_GPIO_PORT GPIOB
#define SDA_GPIO_PIN 7
/*
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7输入模式
28=4*7,可以参考STM32手册GPIO寄存器章节修改
*/
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7输入模式
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;} 	//PB7输出模式

//IO操作
#define IIC_SCL   PBout(6) //SCL
#define IIC_SDA   PBout(7) //SDA
#define READ_SDA  PBin(7)  //输入SDA

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

#endif

