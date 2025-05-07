#ifndef MC02_GENERAL_FRAMEWORK_LITE_DM_IMU_L1_H
#define MC02_GENERAL_FRAMEWORK_LITE_DM_IMU_L1_H

#include "struct_typedef.h"
#include "main.h"

#define DM_IMU1_CAN_ID 0x01
#define DM_IMU1_MASTER_ID 0x11
#define DM_IMU2_CAN_ID 0x02
#define DM_IMU2_MASTER_ID 0x12

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define DM_IMU_L1_ACCEL_REG 0x01
#define DM_IMU_L1_GYRO_REG 0x02
#define DM_IMU_L1_ANGLE_REG 0x03
#define DM_IMU_L1_QUAT_REG 0x04

typedef struct {
    float pitch;
    float roll;
    float yaw;

    float gyro[3];
    float accel[3];

    float quat[4];

    float temperature;
} dm_imu_t;


void DM_imu_l1_get_accel(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

void DM_imu_l1_get_gyro(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

void DM_imu_l1_get_angle(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

void DM_imu_l1_get_quaternion(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id);

dm_imu_t *get_dm_imu_point(uint8_t i);

void CAN_DM_imu_callback(uint32_t can_id, const uint8_t *rx_data);

#endif //MC02_GENERAL_FRAMEWORK_LITE_DM_IMU_L1_H
