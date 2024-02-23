#include "imu.h"
#include "math.h"
#include "MPU6050.h"

_imu_dev dev;
static float NormAcc;
const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;

// �����ǽ��ٶ�ԭʼֵת��Ϊʵ�ʵ�ֵ
// ���̡�2000ʱ��65536/4000=16.384,����ʵ��ת��ʱ�ǣ�ԭʼֵ/16.384,��ԭʼֵ*1/16.384��ԭʼֵ*0.061035
const float Gyro_G = 0.03051756f * 2; // �����ǳ�ʼ������+-2000��ÿ����1 / (65536 /s 4000) = 0.03051756*2
// �Ƕ�ת��Ϊ����
const float Gyro_Gr = 0.0005326f * 2; // ������ÿ��,ת������ÿ���� 2*0.03051756	 * 0.0174533f = 0.0005326*2

void IMU_self_test(void);

void IMU_get_origion_data(_imu_dev *dev)
{
    // ������ϵ��
    static _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}, {0.02, 0, 0, 0, 0.001, 0.543}};
    // һ�׵�ͨ�˲�ϵ��
    const float factor = 0.15f;
    // �����ϴ�����
    static float buff[3];

    // ��ԭʼ����,
    MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, dev->AC_buffer);
    MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, dev->GY_buffer);
    // ת����16λ����
    dev->aac_x = ((short)(dev->AC_buffer[0]) << 8) | dev->AC_buffer[1];
    dev->aac_y = ((short)(dev->AC_buffer[2]) << 8) | dev->AC_buffer[3];
    dev->aac_z = ((short)(dev->AC_buffer[4]) << 8) | dev->AC_buffer[5];
    dev->gyro_x = ((short)(dev->GY_buffer[0]) << 8) | dev->GY_buffer[1];
    dev->gyro_y = ((short)(dev->GY_buffer[2]) << 8) | dev->GY_buffer[3];
    dev->gyro_z = ((short)(dev->GY_buffer[4]) << 8) | dev->GY_buffer[5];

    // �Լ��ٶ���һά�������˲�
    kalman_1(ekf + 0, (float)dev->aac_x);
    dev->aac_x = (int16_t)ekf[0].out;
    kalman_1(ekf + 1, (float)dev->aac_y);
    dev->aac_y = (int16_t)ekf[1].out;
    kalman_1(ekf + 2, (float)dev->aac_z);
    dev->aac_z = (int16_t)ekf[2].out;

    // �Խ��ٶ���һ�׵�ͨ�˲�
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
    // ��ԭʼADCֵת����ʵ��ֵ
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
    static Quaternion NumQ = {1, 0, 0, 0}; // ��ʱ����Ԫ����ֵ  w i j k
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // ��ȡ��Ч��ת�����е���������������Ԫ���ĵ�Ч���Ҿ����е���������
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // ���ٶȹ�һ��,��һ������Ҳʡȥ��ԭʼֵ��ʵ��ֵ��ת��
    NormAcc = Q_rsqrt(squa(dev->aac_x) + squa(dev->aac_y) + squa(dev->aac_z));
    Acc.x = dev->aac_x * NormAcc;
    Acc.y = dev->aac_y * NormAcc;
    Acc.z = dev->aac_z * NormAcc;
    // ���ٶȼƻ�õ�����������һ��֮���ֵ����ȡ����̬���������������˻�ȡ��̬��������˵ó���ֵ
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // �������̻��֣��Ӷ��������������ٶȻ��ֲ������ٶȵĲ���ֵ
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // ���ٶ��ںϼ��ٶȻ��ֲ���ֵ *gyro_x����ͬʱ����ԭʼֵ��ʵ��ֵ��ת��
    Gyro.x = dev->gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; // ������
    Gyro.y = dev->gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = dev->gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // һ�����������, ������Ԫ��

    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // ��Ԫ����һ��
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // ��Ԫ��תŷ����
    {

#ifdef YAW_GYRO
        dev->yaw = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        float yaw_G = dev->gyro_z * Gyro_G;
        if ((yaw_G > 1.2f) || (yaw_G < -1.2f)) // ����̫С������Ϊ�Ǹ��ţ�����ƫ������
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
**�������˲�
**@brief: �������������˲�
**@param[in]  InputData �˲�ǰ�����ݣ�QR���
**@param[out] None
**@return �˲��������
**@remark: ͨ���޸Ĺ��������Ͳ�������R,Qֵ�Ż����ֵ
X(k)=A X(k-1)+B U(k)+W(k)
Z(k)=H X(k)+V(k)
AB��ϵͳ����
X?Kʱ�̵�ϵͳ״̬
H������ϵͳ�Ĳ���
Z��Kʱ�̵Ĳ���ֵ
WV�����̺Ͳ�������

X(k|k-1)=X(k-1|k-1) Ԥ����һ״̬��ϵͳ
P(k|k-1)=P(k-1|k-1) +Q      //Ԥ��Э����
Kg(k)= P(k|k-1) / (P(k|k-1) + R)   ����Kg����������
X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))   ����Ԥ��ֵ��Ϲ���ֵ�ó���ǰ״ֵ̬
P(k|k)=(1-Kg(k))P(k|k-1)  ����Э����


(k-1|k-1)��һ��״̬����������
(k|k-1) ����һ��״̬����������Ԥ�⵱ǰ״̬����������
(k|k)  ��Ԥ�Ȿ״̬����������ʵ����������

Q:ϵͳ���̵�Э����
R:������Э����
��˹�� = Q+R
Kg������������
P��Э����

ע:���������˲������Ե�ģ�ͣ�������H,I��Ϊ1��û�п�����U=0��ͨ����A,B��ʼֵȡ1
ע��X����������X(0|0)��ʼ����ֵ״̬���ݣ�����ǰ����ֵ������
ע :   P (0|0)һ�㲻ȡ0��ȡ0��ζ��0ʱ��ķ���Ϊ0��ϵͳ��Ϊ0ʱ�̵�ֵ�����ŵġ�Ȼ����ʵ��ϵͳ������0ʱ�̲������ŵ�
*/
void kalman_1(_1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;                   // Ԥ�Ȿ��Э����
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);       // ���㿨��������
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out); // ����Ԥ��ֵ��Ϲ���ֵ�ó���ǰ״ֵ̬
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;            // ����Э����
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
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration ����һ��ţ�ٵ�����
    return y;
}
