//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : Evk123
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明: 
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PD6（SCL）
//              SDA   接PD7（SDA）            
//              ----------------------------------------------------------------
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"
#include "stdlib.h"	    	
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口定义----------------  					   
#define u8 uint8_t
#define u32 uint32_t

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define OLED_ADDR				0X78
#define COM				0x00  // OLED 指令（禁止修改）
#define DAT 			0x40  // OLED 数据（禁止修改）


//OLED控制用函数
void OLED_WR_Byte(unsigned char dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
// 显示数字，大小为12或者16
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
// 设定当前坐标
void OLED_Set_Pos(unsigned char x, unsigned char y);
// 显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
// 绘图
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
// 延时，不必使用
void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);
// 填充指定数据
void fill_picture(unsigned char fill_Data);

void OLED_DrawLine(uint16_t begin_x,uint16_t begin_y,uint16_t end_x,uint16_t end_y);
void OLED_DrawPoint(uint8_t x,uint8_t y);
void OLED_StartFloat(uint8_t dir);
void OLED_StopFloat(void);

// 写数据和指令，不必修改
void Write_IIC_Command(unsigned char* IIC_Command,int8_t len);
void Write_IIC_Data(unsigned char *IIC_Data,int8_t len);

#endif  
	 



