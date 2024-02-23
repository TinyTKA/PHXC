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
    float last_error; //�ϴ���� ΢����
    float now_error;  //������� p��
    float total_error;    //����� ������
    float now_position;   //��ǰ״̬
    
    int output;        //�������
    float target;        //Ŀ��
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
