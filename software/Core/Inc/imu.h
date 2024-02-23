#ifndef __IMU_H
#define __IMU_H
#include "main.h"

typedef struct
{
    volatile float pitch; // 32λ
    volatile float roll;
    volatile float yaw;
    volatile int16_t aac_x; // 16λ
    volatile int16_t aac_y;
    volatile int16_t aac_z;
    volatile short gyro_x;
    volatile short gyro_y;
    volatile short gyro_z;

    volatile char AC_buffer[6];
    volatile char GY_buffer[6];

    uint8_t address;     // ������ַ
    uint8_t sample_rate; // �������� ��λms

} _imu_dev;

#define dt 0.005f
// ��Ԫ��
typedef volatile struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

#define LSB_PER_G 16384 // ���ٶȼ�������
#define LSB_PER_S 16.4  // ������������

#define Kp 2.0f   // ���ٶ�Ȩ�أ�Խ������ٶȲ���ֵ����Խ��
#define Ki 0.001f // ����������
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define YAW_GYRO
extern _imu_dev dev;
// IMU����
void IMU_self_test(void);

void IMU_get_origion_data(_imu_dev *dev);
void IMU_get_euler_Inter(_imu_dev *dev);
void IMU_get_euler(_imu_dev *dev);

// �˲�
typedef struct
{
    float LastP;    //�ϴε�Э����
    float Now_P;    //����Э����
    float out;      //�������˲��󱾴����
    float Kg;       //����������
    float Q;        //ϵͳ���̵�Э����
    float R;        //������Э����
} _1_ekf_filter;

/*
    һ�׿������˲�
    @param ekf һ�׿������˲�����
    @param input �˲�����
*/
void kalman_1(_1_ekf_filter *ekf, float input);
float Q_rsqrt(float number);
#endif
