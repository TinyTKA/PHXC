#include "debug.h"
#include "imu.h"
#include "usart.h"
#include "stdio.h"
#ifdef debug
#ifdef ano

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&usart, (uint8_t *)&ch, 1,0xff); // 阻塞方式打印
    return ch;
}




uint8_t XsendBuffer[50];

void myReport_Euler(int16_t data1, int16_t data2, int16_t data3, uint8_t code)
{
    uint8_t Euler_buffer[7];
    debug_info_dev m_dev;
    m_dev.head = 0xAA;
    m_dev.addr = 0xFF;
    m_dev.id = 0x03;
    m_dev.length = 7;
    m_dev.databuf = Euler_buffer;

    Euler_buffer[0] = data1 & 0xff;
    Euler_buffer[1] = (data1 >> 8) & 0xff;
    Euler_buffer[2] = data2 & 0xff;
    Euler_buffer[3] = (data2 >> 8) & 0xff;
    Euler_buffer[4] = data3 & 0xff;
    Euler_buffer[5] = (data3 >> 8) & 0xff;
    Euler_buffer[6] = 1;

    ano_report(&m_dev);
}
void myReport_gyro_Acc(_imu_dev *dev, uint8_t code)
{
    debug_info_dev m_dev;
    uint8_t buffer[13];

    m_dev.head = 0xaa;
    m_dev.addr = 0xff;
    m_dev.id = code;
    m_dev.length = 13;
    m_dev.databuf = buffer;
    buffer[0] = dev->aac_x & 0xff;
    buffer[1] = (dev->aac_x >> 8) & 0xff;
    buffer[2] = dev->aac_y & 0xff;
    buffer[3] = (dev->aac_y >> 8) & 0xff;
    buffer[4] = dev->aac_z & 0xff;
    buffer[5] = (dev->aac_z >> 8) & 0xff;
    buffer[6] = dev->gyro_x & 0xff;
    buffer[7] = (dev->gyro_x >> 8) & 0xff;
    buffer[8] = dev->gyro_y & 0xff;
    buffer[9] = (dev->gyro_y >> 8) & 0xff;
    buffer[10] = dev->gyro_z & 0xff;
    buffer[11] = (dev->gyro_z >> 8) & 0xff;
    buffer[12] = 0;
    ano_report(&m_dev);
}

void myReport_PWM(int16_t data1,int16_t data2,int16_t roll,int16_t data4,uint8_t code)
{
     uint8_t Euler_buffer[8];
    debug_info_dev m_dev;
    m_dev.head = 0xAA;
    m_dev.addr = 0xFF;
    m_dev.id = code;
    m_dev.length = 8;
    m_dev.databuf = Euler_buffer;

    Euler_buffer[0] = data1 & 0xff;
    Euler_buffer[1] = (data1 >> 8) & 0xff;
    Euler_buffer[2] = data2 & 0xff;
    Euler_buffer[3] = (data2 >> 8) & 0xff;
    Euler_buffer[4] = roll & 0xff;
    Euler_buffer[5] = (roll >> 8) & 0xff;
    Euler_buffer[6] = data4 & 0xff;
    Euler_buffer[7] = (data4 >> 8) & 0xff;

    ano_report(&m_dev);
}

void ano_report_Data(int dataLength, char *sendBuffer, char code)
{
    uint8_t i = dataLength + 6;
    uint8_t sc = 0;
    uint8_t ac = 0;
    XsendBuffer[0] = 0xAA;
    XsendBuffer[1] = 0xFF;
    XsendBuffer[2] = code;
    XsendBuffer[3] = dataLength;
    for (uint8_t j = 0; j < dataLength; j++)
    {
        XsendBuffer[4 + j] = sendBuffer[j];
    }
    for (uint8_t j = 0; j < dataLength + 4; j++)
    {
        sc += XsendBuffer[j];
        ac += sc;
    }
    XsendBuffer[i - 2] = sc;
    XsendBuffer[i - 1] = ac;
}

void ano_report(debug_info_dev *dev)
{
    dev->sc = 0;
    dev->ac = 0;
    dev->m_sendbuf[0] = dev->head;
    dev->m_sendbuf[1] = dev->addr;
    dev->m_sendbuf[2] = dev->id;
    dev->m_sendbuf[3] = dev->length;
    // 从数据缓冲区搬运到发送缓冲区

    for (uint8_t j = 0; j < dev->length; j++)
    {
        dev->m_sendbuf[4 + j] = dev->databuf[j];
    }
    for (uint8_t j = 0; j < dev->length + 4; j++)
    {
        dev->sc += dev->m_sendbuf[j];
        dev->ac += dev->sc;
    }
    dev->m_sendbuf[dev->length + 4] = dev->sc;
    dev->m_sendbuf[dev->length + 5] = dev->ac;
    HAL_UART_Transmit(&huart2,dev->m_sendbuf,dev->length+6,0xff);
}
#endif
#endif
