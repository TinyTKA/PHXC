#ifndef __IMU_H
#define __IMU_H
#include "main.h"

typedef struct
{
    volatile float pitch; // 32位
    volatile float roll;
    volatile float yaw;
    volatile int16_t aac_x; // 16位
    volatile int16_t aac_y;
    volatile int16_t aac_z;
    volatile short gyro_x;
    volatile short gyro_y;
    volatile short gyro_z;

    volatile char AC_buffer[6];
    volatile char GY_buffer[6];

    uint8_t address;     // 器件地址
    uint8_t sample_rate; // 采样周期 单位ms

} _imu_dev;

#define dt 0.005f
// 四元数
typedef volatile struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

#define LSB_PER_G 16384 // 加速度计灵敏度
#define LSB_PER_S 16.4  // 陀螺仪灵敏度

#define Kp 2.0f   // 加速度权重，越大则加速度测量值收敛越快
#define Ki 0.001f // 误差积分增益
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define YAW_GYRO
extern _imu_dev dev;
// IMU部分
void IMU_self_test(void);

void IMU_get_origion_data(_imu_dev *dev);
void IMU_get_euler_Inter(_imu_dev *dev);
void IMU_get_euler(_imu_dev *dev);

// 滤波
typedef struct
{
    float LastP;    //上次的协方差
    float Now_P;    //本次协方差
    float out;      //卡尔曼滤波后本次输出
    float Kg;       //卡尔曼增益
    float Q;        //系统过程的协方差
    float R;        //测量的协方差
} _1_ekf_filter;

/*
    一阶卡尔曼滤波
    @param ekf 一阶卡尔曼滤波参数
    @param input 滤波输入
*/
void kalman_1(_1_ekf_filter *ekf, float input);
float Q_rsqrt(float number);
#endif
