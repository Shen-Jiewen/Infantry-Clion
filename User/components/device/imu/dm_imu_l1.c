/*******************************************************************************
 * @file: dm_imu_v1.c/.h
 * @author: X.yu
 * @date: 2025年4月20日
 * @brief: DM IMU L1,MC02支持包
 * @note: 该文件定义了DM IMU L1的相关数据结构、和函数声明。
 *        支持CAN通信解析
 *@brief 使用方法：
 *              1.DM_imu依旧为一收一发模式，例如发送获取欧拉角，返回欧拉角
 *              2.调用DM_imu_l1_get_xxx()后将CAN_DM_imu_callback放入对应的CAN中断回调中
 *              3.imu解算的所有数据将放置到静态结构体DM_imu_v1中
 *              4.通过get_dm_imu_point()函数获取对应的结构体指针
 * @copyright: Copyright (c) 2025
 * @license: MIT
 ******************************************************************************/

#include "dm_imu_l1.h"
#include <stdint.h>

static dm_imu_t DM_imu_l1[2];

static float dm_imu_uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

/**
 * @brief DM_IMU数据解包加速度
 * @param pData CAN回调数据
 * @param i 不同IMU的索引
 */
static void IMU_UpdateAccel(const uint8_t *pData, uint8_t i) {
    uint16_t accel[3];

    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];

    DM_imu_l1[i].accel[0] = dm_imu_uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX, 16);
    DM_imu_l1[i].accel[1] = dm_imu_uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX, 16);
    DM_imu_l1[i].accel[2] = dm_imu_uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX, 16);
}

/**
 * @brief DM_IMU数据解包角速度
 * @param pData CAN回调数据
 * @param i 不同IMU的索引
 */
static void IMU_UpdateGyro(const uint8_t *pData, uint8_t i) {
    uint16_t gyro[3];

    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];

    DM_imu_l1[i].gyro[0] = dm_imu_uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX, 16);
    DM_imu_l1[i].gyro[1] = dm_imu_uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX, 16);
    DM_imu_l1[i].gyro[2] = dm_imu_uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX, 16);
}

/**
 * @brief DM_IMU数据解包欧拉角
 * @param pData CAN回调数据
 * @param i 不同IMU的索引
 */
static void IMU_UpdateEuler(const uint8_t *pData, uint8_t i) {
    int euler[3];

    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];

    DM_imu_l1[i].pitch = dm_imu_uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX, 16);
    DM_imu_l1[i].yaw = dm_imu_uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX, 16);
    DM_imu_l1[i].roll = dm_imu_uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX, 16);
}

/**
 * @brief DM_IMU数据解包四元数
 * @param pData CAN回调数据
 */
static void IMU_UpdateQuaternion(const uint8_t *pData, uint8_t i) {
    int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
    int z = (pData[6] & 0x3F) << 8 | pData[7];

    DM_imu_l1[i].quat[0] = dm_imu_uint_to_float(w,Quaternion_MIN,Quaternion_MAX, 14);
    DM_imu_l1[i].quat[1] = dm_imu_uint_to_float(x,Quaternion_MIN,Quaternion_MAX, 14);
    DM_imu_l1[i].quat[2] = dm_imu_uint_to_float(y,Quaternion_MIN,Quaternion_MAX, 14);
    DM_imu_l1[i].quat[3] = dm_imu_uint_to_float(z,Quaternion_MIN,Quaternion_MAX, 14);
}

/**
 * @brief DM_IMU的回调函数，使用时将它放到对应的CAN回调中
 * @param can_id DM imu的CAN_ID
 * @param rx_data CAN接收的数据
 */
void CAN_DM_imu_callback(uint32_t can_id, const uint8_t *rx_data) {
    if (can_id == DM_IMU1_MASTER_ID) {
        switch (rx_data[0]) {
            case 1:
                IMU_UpdateAccel(rx_data, 0);
                break;
            case 2:
                IMU_UpdateGyro(rx_data, 0);
                break;
            case 3:
                IMU_UpdateEuler(rx_data, 0);
                break;
            case 4:
                IMU_UpdateQuaternion(rx_data, 0);
                break;
            default:
                break;
        }
    } else if (can_id == DM_IMU2_MASTER_ID) {
        switch (rx_data[0]) {
            case 1:
                IMU_UpdateAccel(rx_data, 1);
                break;
            case 2:
                IMU_UpdateGyro(rx_data, 1);
                break;
            case 3:
                IMU_UpdateEuler(rx_data, 1);
                break;
            case 4:
                IMU_UpdateQuaternion(rx_data, 1);
                break;
            default:
                break;
        }
    }
}

uint32_t watch_count = 0;

/**
 *
 * @param can_id DM imu的CAN_ID
 * @param reg 获取达妙IMU测量值的对应寄存器地址
 */
void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg) {
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t cmd[4] = {(uint8_t) can_id, (uint8_t) (can_id >> 8), reg, 0xCC};
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.MessageMarker = 0;
    tx_header.Identifier = 0x6FF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, cmd) != HAL_OK) {
        watch_count++;
    }
}

/**
 * @brief 获取达妙陀螺仪的加速度命令
 */
void DM_imu_l1_get_accel(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id) {
    IMU_RequestData(hfdcan, can_id,DM_IMU_L1_ACCEL_REG);
}

/**
 * @brief 获取达妙陀螺仪的角速度命令
 */
void DM_imu_l1_get_gyro(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id) {
    IMU_RequestData(hfdcan, can_id,DM_IMU_L1_GYRO_REG);
}

/**
 * @brief 获取达妙陀螺仪的欧拉角命令
 */
void DM_imu_l1_get_angle(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id) {
    IMU_RequestData(hfdcan, can_id,DM_IMU_L1_ANGLE_REG);
}

/**
 * @brief 获取达妙陀螺仪的四元数命令
 */
void DM_imu_l1_get_quaternion(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id) {
    IMU_RequestData(hfdcan, can_id,DM_IMU_L1_QUAT_REG);
}

/**
 *
 * @return 返回达妙IMU数据结构体指针
 */
dm_imu_t *get_dm_imu_point(uint8_t i) {
    return &DM_imu_l1[i];
}
