#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//IIC��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2019/9/18
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO��������

/*	
SDA:PB7
SCL:PB6

�޸ķ�ʽ��
	�鿴����Ҫʹ�õ�IIC����
	SDA_IN������SDA����Ϊ����ģʽ������ͨ��ֱ���޸ļĴ����ķ�ʽ������
					����Ϊ0~7ʱ������GPIOx->CRL�Ĵ�����ÿ��λ��Ӧһ��IO��,�������Ӧλ�����ã��ٰ�������(1000����8)�������(0011,��3)������
					����λ8~15ʱ������GPIOx->CRH�Ĵ���
*/

#define SCL_GPIO_PORT	GPIOB
#define SCL_GPIO_PIN 6
#define SDA_GPIO_PORT GPIOB
#define SDA_GPIO_PIN 7
/*
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7����ģʽ
28=4*7,���Բο�STM32�ֲ�GPIO�Ĵ����½��޸�
*/
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;}	//PB7����ģʽ
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;} 	//PB7���ģʽ

//IO����
#define IIC_SCL   PBout(6) //SCL
#define IIC_SDA   PBout(7) //SDA
#define READ_SDA  PBin(7)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

#endif

