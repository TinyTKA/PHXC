#ifndef __PID_H
#define __PID_H
#include "main.h"
#include "tim.h"
#include "Motor.h"
typedef struct
{
    int kp;
    int ki;
    int kd;
    float last_error; //上次误差 微分用
    float now_error;  //本次误差 p用
    float total_error;    //总误差 积分用
    float now_position;   //当前状态
    
    int output;        //本次输出
    float target;        //目标
    int ori_output;

}PID_POS;

#define dt 0.01
#define Mechan_Balance -1.2
#define MAX_PWM_OUTPUT 4999
extern PID_POS pid_p;
extern PID_POS pid_v;
extern int output;
void PID_Init(void);
uint16_t Vertical(float target_angel ,float target_speed);


#endif
