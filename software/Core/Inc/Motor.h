#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "tim.h"

#define Motor_go_forward 1
#define Motor_go_back 2
#define COUNTERNUM_M1 __HAL_TIM_GET_COUNTER(&htim1)
#define COUNTERNUM_M2 __HAL_TIM_GET_COUNTER(&htim2)

#define AIN1(state) HAL_GPIO_WritePin(M_AIN1_GPIO_Port,M_AIN1_Pin,(GPIO_PinState)(state))
#define AIN2(state) HAL_GPIO_WritePin(M_AIN2_GPIO_Port,M_AIN2_Pin,(GPIO_PinState)(state))
#define BIN1(state) HAL_GPIO_WritePin(M_BIN1_GPIO_Port,M_BIN1_Pin,(GPIO_PinState)(state))
#define BIN2(state) HAL_GPIO_WritePin(M_BIN2_GPIO_Port,M_BIN2_Pin,(GPIO_PinState)(state))
#define START_ACTION HAL_GPIO_WritePin(M_STBY_GPIO_Port,M_STBY_Pin,1)
#define STOP_ACTION HAL_GPIO_WritePin(M_STBY_GPIO_Port,M_STBY_Pin,0)
typedef struct 
{
    short last_counter;
    short now_counter;
    uint16_t lastAngle;
    uint16_t totalAngle;
    uint16_t loopNum;
    uint8_t dir;
    int16_t speed;
    uint8_t id;
}Motor;
//初始化电机
void Motor_Init(void);
//控制电机方向
void Motor_go(Motor* m,uint8_t dir);
void Motor_speed(Motor* m,uint16_t speed);
void Motor_stop(void);

#endif

