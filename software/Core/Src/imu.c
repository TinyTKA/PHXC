#include "imu.h"
#include "math.h"
#include "MPU6050.h"

_imu_dev dev;
static float NormAcc;
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;

// 陀螺仪角速度原始值转换为实际的值
// 量程±2000时，65536/4000=16.384,所以实际转换时是：原始值/16.384,即原始值*1/16.384即原始值*0.061035
const float Gyro_G = 0.03051756f * 2; // 陀螺仪初始化量程+-2000度每秒于1 / (65536 /s 4000) = 0.03051756*2
// 角度转换为弧度
const float Gyro_Gr = 0.0005326f * 2; // 面计算度每秒,转换弧度每秒则 2*0.03051756	 * 0.0174533f = 0.0005326*2

void IMU_self_test(void);

void IMU_get_origion_data(_imu_dev *dev)
{
    // 卡尔曼系数
    static _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}};
    // 一阶低通滤波系数
    const float factor = 0.15f;
    // 保存上次数据
    static float buff[3];

    // 读原始数据,
    MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, dev->AC_buffer);
    MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, dev->GY_buffer);
    // 转换成16位数据
    dev->aac_x = ((short)(dev->AC_buffer[0]) << 8) | dev->AC_buffer[1];
    dev->aac_y = ((short)(dev->AC_buffer[2]) << 8) | dev->AC_buffer[3];
    dev->aac_z = ((short)(dev->AC_buffer[4]) << 8) | dev->AC_buffer[5];
    dev->gyro_x = ((short)(dev->GY_buffer[0]) << 8) | dev->GY_buffer[1];
    dev->gyro_y = ((short)(dev->GY_buffer[2]) << 8) | dev->GY_buffer[3];
    dev->gyro_z = ((short)(dev->GY_buffer[4]) << 8) | dev->GY_buffer[5];

    // 对加速度做一维卡尔曼滤波
    kalman_1(ekf + 0, (float)dev->aac_x);
    dev->aac_x = (int16_t)ekf[0].out;
    kalman_1(ekf + 1, (float)dev->aac_y);
    dev->aac_y = (int16_t)ekf[1].out;
    kalman_1(ekf + 2, (float)dev->aac_z);
    dev->aac_z = (int16_t)ekf[2].out;

    // 对角速度做一阶低通滤波
    dev->gyro_x = buff[0] * (1 - factor) + dev->gyro_x * factor;
    dev->gyro_y = buff[1] * (1 - factor) + dev->gyro_y * factor;
    dev->gyro_z = buff[2] * (1 - factor) + dev->gyro_z * factor;
    buff[0] = dev->gyro_x;
    buff[1] = dev->gyro_y;
    buff[2] = dev->gyro_z;
}

/*



*/

void IMU_get_euler_Inter(_imu_dev *dev)
{
    static _imu_dev last_imu_dev;
    // 从原始ADC值转换成实际值
    float aac_x, aac_y, aac_z;
    float gyro_x, gyro_y, gyro_z;
    gyro_x = (dev->gyro_x + 64) * Gyro_G;
    gyro_y = (dev->gyro_y - 7) * Gyro_G;
    gyro_z = (dev->gyro_z) * Gyro_G;
    dev->pitch += gyro_x * dt;
    dev->roll += gyro_y * dt;
    dev->yaw += gyro_z * dt;
}

void IMU_get_euler(_imu_dev *dev)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.9f;
    static float KiDef = 0.001f;
    static Quaternion NumQ = {1, 0, 0, 0}; // 零时刻四元数初值  w i j k
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量，即四元数的等效余弦矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 加速度归一化,归一化操作也省去了原始值向实际值的转换
    NormAcc = Q_rsqrt(squa(dev->aac_x) + squa(dev->aac_y) + squa(dev->aac_z));
    Acc.x = dev->aac_x * NormAcc;
    Acc.y = dev->aac_y * NormAcc;
    Acc.z = dev->aac_z * NormAcc;
    // 加速度计获得的重力向量归一化之后的值与提取的姿态矩阵的重力向量叉乘获取姿态误差，向量叉乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 对误差进程积分，从而消除误差，再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // 角速度融合加速度积分补偿值 *gyro_x操作同时暗含原始值向实际值的转换
    Gyro.x = dev->gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; // 弧度制
    Gyro.y = dev->gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = dev->gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // 一阶龙格库塔法, 更新四元数

    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 四元数转欧拉角
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // 数据太小可以认为是干扰，不是偏航动作
        {
            dev->yaw += yaw_G * dt;
        }
#endif
        dev->pitch = asin(2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;

        dev->roll = atan2(2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA; // PITCH

        //dev->roll = dev->roll < 0 ? dev->roll + 180 : dev->roll - 180;
    }
}
/*********************************************************************************************************************
**卡尔曼滤波
**@brief: 线性最优评估滤波
**@param[in]  InputData 滤波前的数据，QR误差
**@param[out] None
**@return 滤波后的数据
**@remark: 通过修改过程噪声和测量噪声R,Q值优化输出值
X(k)=A X(k-1)+B U(k)+W(k)
Z(k)=H X(k)+V(k)
AB是系统参数
X?K时刻的系统状态
H：测量系统的参数
Z：K时刻的测量值
WV：过程和测量噪声

X(k|k-1)=X(k-1|k-1) 预测下一状态的系统
P(k|k-1)=P(k-1|k-1) +Q      //预测协方差
Kg(k)= P(k|k-1) / (P(k|k-1) + R)   计算Kg卡尔曼增益
X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))   根据预测值结合估算值得出当前状态值
P(k|k)=(1-Kg(k))P(k|k-1)  更新协方差


(k-1|k-1)上一个状态的最优评估
(k|k-1) 由上一个状态的最优评估预测当前状态的最优评估
(k|k)  由预测本状态的评估具体实现最优评估

Q:系统过程的协方差
R:测量的协方差
高斯白 = Q+R
Kg：卡尔曼增益
P：协方差

注:本卡尔曼滤波器争对单模型，单测量H,I均为1，没有控制量U=0，通常对A,B初始值取1
注：X会逐渐收敛，X(0|0)初始测量值状态根据，测量前的数值而定。
注 :   P (0|0)一般不取0，取0意味在0时候的方差为0，系统认为0时刻的值是最优的。然而在实际系统中往往0时刻不是最优的
*/
void kalman_1(_1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;                   // 预测本次协方差
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);       // 计算卡尔曼增益
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out); // 根据预测值结合估算值得出当前状态值
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;            // 更新协方差
}

float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration （第一次牛顿迭代）
    return y;
}
