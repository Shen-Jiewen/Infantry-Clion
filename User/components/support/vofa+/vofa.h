/**
* @file vofa.h
 * @brief VOFA数据传输接口头文件
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 最大缓冲区大小
 */
#define VOFA_MAX_BUFFER_SIZE 1024

/**
 * @brief VOFA数据缓冲区结构体
 */
typedef struct {
    float data[VOFA_MAX_BUFFER_SIZE / 4]; /**< 数据缓冲区 */
    uint16_t count;                       /**< 当前缓冲区中的数据数量 */
} VofaDataBuffer;

/**
 * @brief 初始化VOFA功能
 */
void vofa_init(void);

/**
 * @brief 启动VOFA演示，发送示例数据
 */
void vofa_start_demo(void);

/**
 * @brief 底层数据传输函数
 * @param buf 数据缓冲区
 * @param len 数据长度
 * @return 传输结果，0表示成功
 */
uint8_t vofa_transmit_raw(uint8_t* buf, uint16_t len);

/**
 * @brief 添加浮点数据到缓冲区
 * @param data 要添加的浮点数据
 * @return 操作是否成功
 */
bool vofa_append_data(float data);

/**
 * @brief 发送当前缓冲区中的所有数据
 * @return 发送结果，0表示成功
 */
uint8_t vofa_send_frame(void);

/**
 * @brief 在指定位置插入浮点数据
 * @param index 插入位置索引
 * @param data 要插入的浮点数据
 * @return 操作是否成功
 */
bool vofa_insert_data(uint16_t index, float data);

/**
 * @brief 删除指定位置的浮点数据
 * @param index 要删除的数据索引
 * @return 操作是否成功
 */
bool vofa_delete_data(uint16_t index);

/**
 * @brief 清空数据缓冲区
 */
void vofa_clear_buffer(void);

/**
 * @brief 获取浮点数的字节，用于跨平台数据传输
 */
uint8_t vofa_float_byte0(float f);
uint8_t vofa_float_byte1(float f);
uint8_t vofa_float_byte2(float f);
uint8_t vofa_float_byte3(float f);

#endif /* __VOFA_H__ */