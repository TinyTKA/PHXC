#include "Motor.h"


// 一共两个电机
Motor m1;
Motor m2;
void Motor_Init()
{
    STOP_ACTION;
    m1.id = 1;
    m2.id = 2;
    m1.last_counter=0;
    m2.last_counter=0;
    m1.now_counter=0;
    m2.now_counter=0;
    __HAL_TIM_SET_COUNTER(&htim1, 25000);
    __HAL_TIM_SET_COUNTER(&htim2, 25000);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
   
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim3);
    Motor_go(&m1, 1);
    Motor_go(&m2, 1);
    START_ACTION;
}
// 控制电机方向
void Motor_go(Motor *m, uint8_t dir)
{
    m->dir = dir;
    if (m->id == 1)
    {
        if (dir == Motor_go_back)
        {
            AIN1(0);
            AIN2(1);
        }
        else if (dir == Motor_go_forward)
        {
            AIN1(1);
            AIN2(0);
        }
    }
    else if (m->id == 2)
    {
        if (dir == Motor_go_back)
        {
            BIN1(1);
            BIN2(0);
        }
        else if (dir == Motor_go_forward)
        {
            BIN1(0);
            BIN2(1);
        }
    }
}
void Motor_speed(Motor *m, uint16_t speed)
{
    speed = speed > 999 ? 800 : speed;
    if (m->id == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
    }
    else if (m->id == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    }
}
void Motor_stop()
{
    STOP_ACTION;
}
