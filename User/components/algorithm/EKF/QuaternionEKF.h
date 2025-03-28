/**
******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   Quaternion EKF for attitude estimation with gyro bias correction
 *          and chi-square test.
 ******************************************************************************
 */

#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"

/* 布尔类型定义 */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * @brief 四元数 EKF 状态结构体定义
 */
typedef struct {
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4]; ///< 估计的四元数
    float GyroBias[3]; ///< 陀螺仪零漂估计

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数过程噪声
    float Q2; // 陀螺仪偏置过程噪声
    float R; // 加速度计量测噪声

    float dt; // 更新周期（秒）
    mat ChiSquare;
    float ChiSquare_Data[1]; ///< 卡方检验值
    float ChiSquareTestThreshold; ///< 卡方检验阈值
    float lambda; ///< 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt,
                            float lpf);

void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);

void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);

float Get_Roll(void);

float Get_Yaw(void);
#endif
