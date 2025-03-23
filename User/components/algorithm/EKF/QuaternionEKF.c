/**
******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   Attitude estimation using Quaternion EKF with gyro bias correction
 *          and chi-square test.
 ******************************************************************************
 * @attention
 * Implements a first-order low-pass filter (LPF):
 *
 *        1
 *  ————————
 *   as + 1
 ******************************************************************************
 */

#include "QuaternionEKF.h"
#include "arm_math.h"

QEKF_INS_t QEKF_INS = {0};

// 状态转移矩阵和初始协方差矩阵常量
const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

const float IMU_QuaternionEKF_P_Const[36] = {
    100000.f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000.f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000.f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000.f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 100.f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100.f
};

float IMU_QuaternionEKF_P[36] = {
    100000.f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000.f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000.f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000.f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 100.f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100.f
};

float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);

static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief 初始化四元数 EKF，包括过程噪声、量测噪声、渐消因子等参数。
 *
 * @param process_noise1 四元数更新过程噪声。
 * @param process_noise2 陀螺仪偏置过程噪声。
 * @param measure_noise  加速度计量测噪声。
 * @param lambda         渐消因子（建议不大于 1）。
 * @param dt             更新周期（秒）。
 * @param lpf            加速度低通滤波系数。
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt,
                            float lpf) {
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8f;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    QEKF_INS.dt = dt;

    if (lambda > 1) {
        lambda = 1;
    }
    QEKF_INS.lambda = 1.f / lambda; // 渐消因子取倒数

    // 初始化 KF，状态维度6，控制变量0，量测维度3
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, QEKF_INS.ChiSquare_Data);

    // 初始四元数置为单位四元数
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    // 设置用户自定义函数，替代部分标准 KF 步骤
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 跳过标准 KF 的 K 计算与状态更新，采用自定义实现
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief 重置四元数 EKF，使其恢复到初始状态。
 */
void IMU_QuaternionEKF_Reset(void) {
    Kalman_Filter_Reset(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    memcpy(IMU_QuaternionEKF_P, IMU_QuaternionEKF_P_Const, sizeof(IMU_QuaternionEKF_P));

    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;

    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
}

/**
 * @brief 更新四元数 EKF，使用最新的陀螺仪和加速度计数据。
 *
 * @param gx 陀螺仪 x 轴数据。
 * @param gy 陀螺仪 y 轴数据。
 * @param gz 陀螺仪 z 轴数据。
 * @param ax 加速度计 x 轴数据。
 * @param ay 加速度计 y 轴数据。
 * @param az 加速度计 z 轴数据。
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    // 校正陀螺仪数据，减去偏置
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    const float halfgxdt = 0.5f * QEKF_INS.Gyro[0] * QEKF_INS.dt;
    const float halfgydt = 0.5f * QEKF_INS.Gyro[1] * QEKF_INS.dt;
    const float halfgzdt = 0.5f * QEKF_INS.Gyro[2] * QEKF_INS.dt;

    // 更新 F 矩阵中用于四元数传播的部分
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    QEKF_INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // 加速度数据低通滤波及归一化
    if (QEKF_INS.UpdateCount == 0) {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
        QEKF_INS.UpdateCount++;
    }
    float temp_quick = 1.f / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef * temp_quick + ax * QEKF_INS.dt * temp_quick;
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef * temp_quick + ay * QEKF_INS.dt * temp_quick;
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef * temp_quick + az * QEKF_INS.dt * temp_quick;

    QEKF_INS.accl_norm = sqrtf(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
                               QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
                               QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    const float accelInvNorm = 1.0f / QEKF_INS.accl_norm;
    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[0] = QEKF_INS.Accel[0] * accelInvNorm;
    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[1] = QEKF_INS.Accel[1] * accelInvNorm;
    QEKF_INS.IMU_QuaternionEKF.MeasuredVector[2] = QEKF_INS.Accel[2] * accelInvNorm;

    QEKF_INS.gyro_norm = sqrtf(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                               QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                               QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);

    if (QEKF_INS.gyro_norm < 0.3f && QEKF_INS.accl_norm > 9.8f - 0.5f &&
        QEKF_INS.accl_norm < 9.8f + 0.5f) {
        QEKF_INS.StableFlag = 1;
    } else {
        QEKF_INS.StableFlag = 0;
    }

    QEKF_INS.IMU_QuaternionEKF.Q_data[0] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[7] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[14] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[21] = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[28] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q_data[35] = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R_data[0] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[4] = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R_data[8] = QEKF_INS.R;

    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];

    QEKF_INS.Roll = atan2f(QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3],
                           0.5f - QEKF_INS.q[1] * QEKF_INS.q[1] - QEKF_INS.q[2] * QEKF_INS.q[2]) * 57.29578f;
    QEKF_INS.Pitch = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.29578f;
    QEKF_INS.Yaw = atan2f(QEKF_INS.q[1] * QEKF_INS.q[2] + QEKF_INS.q[0] * QEKF_INS.q[3],
                          0.5f - QEKF_INS.q[2] * QEKF_INS.q[2] - QEKF_INS.q[3] * QEKF_INS.q[3]) * 57.29578f;

    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // z 轴偏置不可观测

    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
        QEKF_INS.YawRoundCount--;
    } else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * (float) QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
}

/**
 * @brief 更新线性化后的 F 矩阵右上角分块，并对零漂相关协方差进行渐消与限幅处理。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf) {
    const float q0 = kf->xhatminus_data[0];
    const float q1 = kf->xhatminus_data[1];
    const float q2 = kf->xhatminus_data[2];
    const float q3 = kf->xhatminus_data[3];

    const float qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++) {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    kf->F_data[4] = q1 * QEKF_INS.dt * 0.5f;
    kf->F_data[5] = q2 * QEKF_INS.dt * 0.5f;
    kf->F_data[10] = -q0 * QEKF_INS.dt * 0.5f;
    kf->F_data[11] = q3 * QEKF_INS.dt * 0.5f;
    kf->F_data[16] = -q3 * QEKF_INS.dt * 0.5f;
    kf->F_data[17] = -q0 * QEKF_INS.dt * 0.5f;
    kf->F_data[22] = q2 * QEKF_INS.dt * 0.5f;
    kf->F_data[23] = -q1 * QEKF_INS.dt * 0.5f;

    kf->P_data[28] *= QEKF_INS.lambda;
    kf->P_data[35] *= QEKF_INS.lambda;
    if (kf->P_data[28] > 10000) {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000) {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在当前状态下计算测量函数的雅可比矩阵 H。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf) {
    const float doubleq0 = 2 * kf->xhatminus_data[0];
    const float doubleq1 = 2 * kf->xhatminus_data[1];
    const float doubleq2 = 2 * kf->xhatminus_data[2];
    const float doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;
    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;
    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测数据和先验状态更新状态估计，包含卡方检验与自适应增益。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf) {
    float q0, q1, q2, q3;
    Matrix_Transpose(&kf->H, &kf->HT);
    Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
    Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
    Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    Matrix_Inverse(&kf->S, &kf->temp_matrix1);

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.OrientationCosine[i] = arm_cos_f32(fabsf(kf->temp_vector_data[i]));
    }

    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);
    Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);

    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
        QEKF_INS.ConvergeFlag = 1;
    }
    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
        if (QEKF_INS.StableFlag) {
            QEKF_INS.ErrorCount++;
        } else {
            QEKF_INS.ErrorCount = 0;
        }
        if (QEKF_INS.ErrorCount > 50) {
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = FALSE;
        } else {
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            return;
        }
    } else {
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) /
                                         (0.9f * QEKF_INS.ChiSquareTestThreshold);
        } else {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    for (uint16_t i = 0; i < kf->K.numRows * kf->K.numCols; i++) {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] * (1 / 1.5707963f);
        }
    }
    Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
    if (QEKF_INS.ConvergeFlag) {
        for (uint8_t i = 4; i < 6; i++) {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt)
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt)
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
        }
    }
    kf->temp_vector.pData[3] = 0;
    Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief 复制内部矩阵，便于调试或后续处理。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf) {
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义快速求逆平方根函数。
 *
 * @param x 输入值。
 * @return x 的逆平方根。
 */
static float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

float Get_Pitch(void) {
    return QEKF_INS.Pitch;
}

float Get_Roll(void) {
    return QEKF_INS.Roll;
}

float Get_Yaw(void) {
    return QEKF_INS.Yaw;
}
