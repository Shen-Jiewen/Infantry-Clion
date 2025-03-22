/**
 * @file vofa.c
 * @brief VOFA数据传输接口实现
 */

#include "vofa.h"
#include "usbd_cdc_if.h"
#include <math.h>

/* 私有变量定义 */
static VofaDataBuffer vofa_buffer;                  /**< VOFA数据缓冲区 */
static uint8_t send_buf[VOFA_MAX_BUFFER_SIZE];      /**< 发送缓冲区 */

/* 帧尾定义 */
static const uint8_t VOFA_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

/**
 * @brief 初始化VOFA功能
 */
void vofa_init(void)
{
    /* 清空缓冲区 */
    vofa_clear_buffer();
}

/**
 * @brief 启动VOFA演示
 */
void vofa_start_demo(void)
{
    static float angle = 0.0f;
    const float PI = 3.14159f;

    /* 角度递增 */
    angle += 0.01f;
    if (angle >= 360.0f) {
        angle = 0.0f;
    }

    /* 产生演示数据 */
    const float v1 = angle;  /* 角度值 */
    const float v2 = sinf(angle * PI / 180.0f) * 180.0f + 180.0f;  /* 正弦波 */
    const float v3 = sinf((angle + 120.0f) * PI / 180.0f) * 180.0f + 180.0f;  /* 相位偏移120度 */
    const float v4 = sinf((angle + 240.0f) * PI / 180.0f) * 180.0f + 180.0f;  /* 相位偏移240度 */

    /* 添加数据到缓冲区 */
    vofa_append_data(v1);
    vofa_append_data(v2);
    vofa_append_data(v3);
    vofa_append_data(v4);

    /* 发送数据帧 */
    vofa_send_frame();
}

/**
 * @brief 底层数据传输函数
 * @param buf 数据缓冲区
 * @param len 数据长度
 * @return 传输结果，0表示成功
 */
uint8_t vofa_transmit_raw(uint8_t* buf, const uint16_t len)
{
    /* 使用USB CDC发送数据 */
    return CDC_Transmit_HS(buf, len);
}

/**
 * @brief 添加浮点数据到缓冲区
 * @param data 要添加的浮点数据
 * @return 操作是否成功
 */
bool vofa_append_data(const float data)
{
    /* 检查缓冲区是否已满 */
    if (vofa_buffer.count >= VOFA_MAX_BUFFER_SIZE / 4) {
        return false;
    }

    /* 添加数据到缓冲区 */
    vofa_buffer.data[vofa_buffer.count++] = data;
    return true;
}

/**
 * @brief 发送当前缓冲区中的所有数据
 * @return 发送结果，0表示成功
 */
uint8_t vofa_send_frame(void)
{
    uint16_t send_index = 0;

    /* 检查是否有数据需要发送 */
    if (vofa_buffer.count == 0) {
        return 1; /* 无数据可发送 */
    }

    /* 将浮点数据打包到发送缓冲区 */
    for (uint16_t i = 0; i < vofa_buffer.count; i++) {
        const float value = vofa_buffer.data[i];
        send_buf[send_index++] = vofa_float_byte0(value);
        send_buf[send_index++] = vofa_float_byte1(value);
        send_buf[send_index++] = vofa_float_byte2(value);
        send_buf[send_index++] = vofa_float_byte3(value);
    }

    /* 添加帧尾 */
    for (uint8_t i = 0; i < 4; i++) {
        send_buf[send_index++] = VOFA_FRAME_TAIL[i];
    }

    /* 发送数据 */
    const uint8_t result = vofa_transmit_raw(send_buf, send_index);

    /* 清空缓冲区 */
    vofa_clear_buffer();

    return result;
}

/**
 * @brief 在指定位置插入浮点数据
 * @param index 插入位置索引
 * @param data 要插入的浮点数据
 * @return 操作是否成功
 */
bool vofa_insert_data(const uint16_t index, const float data)
{
    /* 检查索引是否有效且缓冲区未满 */
    if (index > vofa_buffer.count || vofa_buffer.count >= VOFA_MAX_BUFFER_SIZE / 4) {
        return false;
    }

    /* 移动数据以便插入 */
    for (uint16_t i = vofa_buffer.count; i > index; i--) {
        vofa_buffer.data[i] = vofa_buffer.data[i - 1];
    }

    /* 插入数据 */
    vofa_buffer.data[index] = data;
    vofa_buffer.count++;

    return true;
}

/**
 * @brief 删除指定位置的浮点数据
 * @param index 要删除的数据索引
 * @return 操作是否成功
 */
bool vofa_delete_data(const uint16_t index)
{
    /* 检查索引是否有效 */
    if (index >= vofa_buffer.count) {
        return false;
    }

    /* 移动数据以填补空缺 */
    for (uint16_t i = index; i < vofa_buffer.count - 1; i++) {
        vofa_buffer.data[i] = vofa_buffer.data[i + 1];
    }

    /* 减少计数 */
    vofa_buffer.count--;

    return true;
}

/**
 * @brief 清空数据缓冲区
 */
void vofa_clear_buffer(void)
{
    vofa_buffer.count = 0;
}

/**
 * @brief 获取浮点数的字节，用于跨平台数据传输
 */
uint8_t vofa_float_byte0(float f)
{
    return *((uint8_t*)&f + 0);
}

uint8_t vofa_float_byte1(float f)
{
    return *((uint8_t*)&f + 1);
}

uint8_t vofa_float_byte2(float f)
{
    return *((uint8_t*)&f + 2);
}

uint8_t vofa_float_byte3(float f)
{
    return *((uint8_t*)&f + 3);
}