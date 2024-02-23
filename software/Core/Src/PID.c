#include "PID.h"
#include "math.h"
#include "main.h"
extern Motor m1;
extern Motor m2;
PID_POS pid_p;
PID_POS pid_v;
int output = 0;

void PID_Init(void)
{
    pid_p.kp = -700;
    pid_p.ki = 0;
    pid_p.kd = -10;
    pid_p.last_error = 0;
    pid_p.now_error = 0;
    pid_p.now_position = 0;
    pid_p.ori_output = 0;
    pid_p.target = 0;
    pid_p.output = 0;
    pid_p.total_error = 0;

    pid_v.kp = 10;
    pid_v.ki = 1;
    pid_v.kd = 0;
    pid_v.last_error = 0;
    pid_v.now_error = 0;
    pid_v.now_position = 0;
    pid_v.ori_output = 0;
    pid_v.target = 0;
    pid_v.output = 0;
    pid_v.total_error = 0;
}

uint16_t Vertical(float target_angel, float target_speed)
{

    // 设置目标角度
    pid_p.target = target_angel;
    pid_v.target = target_speed;

    pid_p.now_error = pid_p.now_position - pid_p.target; // 计算本次误差
    pid_v.now_error = pid_v.now_position - pid_v.target;

    // 计算当前角度和目标角度的误差值，若误差小于0.1则认为已经平衡
    if (fabs(pid_p.now_error) < 1e-1)
    {
        pid_p.now_error = 0;
    }
    // 如果当前倾角过大，则放弃调整
    if (fabs(pid_p.now_position) < 50.0)
    {
        pid_p.total_error += pid_p.now_error * dt; // 积分
        pid_v.total_error += pid_v.now_error;

        pid_p.ori_output = pid_p.kp * pid_p.now_error + pid_p.ki * pid_p.total_error + pid_p.kd * (pid_p.now_error - pid_p.last_error) / dt;

        pid_v.ori_output = pid_v.kp * pid_v.now_error + pid_v.ki * pid_v.total_error + pid_v.kd * (pid_v.now_error - pid_v.last_error);
        // 计算输出
        pid_p.last_error = pid_p.now_error;

        pid_v.last_error = pid_v.now_error;

        output = pid_p.ori_output + pid_v.ori_output;

        output = output > MAX_PWM_OUTPUT ? MAX_PWM_OUTPUT : output;
        output = output < -MAX_PWM_OUTPUT ? -MAX_PWM_OUTPUT : output; // 上限限幅
        if (output > 0)
        {
            Motor_go(&m1, 2);
            Motor_go(&m2, 2);
        }
        else
        {
            Motor_go(&m1, 1);
            Motor_go(&m2, 1);
        }
        output = output < 0 ? -output : output;
    }
    else
    {
        output = 0;
    }
    if (output <= 150)
        output = 0;

    pid_p.output = output;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, output);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, output);
}
