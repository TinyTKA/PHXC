#ifndef __DEBUG_H
#define __DEBUG_H

#include "main.h"
#include "imu.h"
#define debug 
#define ano
#define usart huart2    //ʹ�õĴ��ں�


typedef struct
{
    uint8_t head;
    uint8_t addr;
    uint8_t id;
    uint8_t length;
    uint8_t *databuf;
    uint8_t sc;
    uint8_t ac;
    uint8_t m_sendbuf[50];
} debug_info_dev;


#ifdef debug 
/*
    ������λ���㱨����
    @param dataLength ���ݳ��ȣ���������8λ
    @param sendBuffer ���������ַ 
    @param code ֡��
*/
#ifdef ano
void myReport_Euler(int16_t data1, int16_t data2, int16_t data3, uint8_t code);
void myReport_gyro_Acc(_imu_dev *dev, uint8_t code);
void myReport_PWM(int16_t data1,int16_t data2,int16_t roll,int16_t data4,uint8_t code);
void ano_report_Data(int dataLength,char* sendBuffer,char code);
void ano_report(debug_info_dev *dev);
#endif
#endif

#endif
