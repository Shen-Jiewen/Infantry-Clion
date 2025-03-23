/**
******************************************************************************
 * @file    kalman_filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of Kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可在不同传感器采样率下动态调整 H、R、K 矩阵的维度和数值。
 * 如果不需要动态调整量测，可将 `UseAutoAdjustment` 设为 0，并像 P 矩阵一样初始化 H、R、z。
 *
 * 量测向量 z 和控制向量 u 需在传感器回调函数中更新。若量测无效（设为 0），则在滤波更新时清零。
 *
 * 过度收敛的 P 矩阵会降低滤波器对状态变化的适应性，本实现通过设定最小方差抑制该问题。
 * 详情请参考示例代码。
 ******************************************************************************
 */

#include "kalman_filter.h"
#include "FreeRTOS.h"

uint16_t sizeof_float = sizeof(float);
uint16_t sizeof_double = sizeof(double);

// 内部函数：根据有效测量数据动态调整 H、R 与 K
static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
 * @brief 内存分配包装函数，使用 FreeRTOS 的 pvPortMalloc。
 */
void *user_malloc(const size_t size) {
    return pvPortMalloc(size);
}

/**
 * @brief 初始化卡尔曼滤波器的矩阵，并为各矩阵分配内存。
 *
 * @param kf       指向 KalmanFilter_t 结构体的指针。
 * @param xhatSize 状态变量维度。
 * @param uSize    控制变量维度。
 * @param zSize    观测量维度。
 */
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize) {
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;
    kf->MeasurementValidNum = 0;

    // 分配并清零测量相关数组
    kf->MeasurementMap = (uint8_t *) user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float *) user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float *) user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *) user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // 分配滤波数据数组
    kf->FilteredValue = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float *) user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float *) user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // 初始化状态估计向量 xhat 与预测 xhatminus
    kf->xhat_data = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, xhatSize, 1, kf->xhat_data);
    kf->xhatminus_data = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, xhatSize, 1, kf->xhatminus_data);

    if (uSize != 0) {
        kf->u_data = (float *) user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, uSize, 1, kf->u_data);
    }

    // 初始化量测向量 z
    kf->z_data = (float *) user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, zSize, 1, kf->z_data);

    // 分配并初始化状态协方差矩阵 P 与预测协方差矩阵 Pminus
    kf->P_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);
    kf->Pminus_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);

    // 状态转移矩阵 F 及其转置 FT
    kf->F_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);

    if (uSize != 0) {
        kf->B_data = (float *) user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    }

    // 量测矩阵 H 及其转置 HT
    kf->H_data = (float *) user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float *) user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);

    // 过程噪声矩阵 Q
    kf->Q_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);

    // 量测噪声矩阵 R
    kf->R_data = (float *) user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, zSize, zSize, kf->R_data);

    // 卡尔曼增益矩阵 K
    kf->K_data = (float *) user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);

    // 辅助矩阵与向量，用于矩阵计算中间结果
    kf->S_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->S_data, 0, sizeof_float * xhatSize * xhatSize);
    kf->temp_matrix_data = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->temp_matrix_data, 0, sizeof_float * xhatSize * xhatSize);
    kf->temp_matrix_data1 = (float *) user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->temp_matrix_data1, 0, sizeof_float * xhatSize * xhatSize);
    kf->temp_vector_data = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->temp_vector_data, 0, sizeof_float * xhatSize);
    kf->temp_vector_data1 = (float *) user_malloc(sizeof_float * xhatSize);
    memset(kf->temp_vector_data1, 0, sizeof_float * xhatSize);

    Matrix_Init(&kf->S, xhatSize, xhatSize, kf->S_data);
    Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize, kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize, kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, xhatSize, 1, kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, xhatSize, 1, kf->temp_vector_data1);

    // 默认所有跳过标志关闭，执行标准 KF 更新步骤
    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief 重置卡尔曼滤波器的状态和矩阵数据。
 *
 * @param kf       指向 KalmanFilter_t 结构体的指针。
 * @param xhatSize 状态变量维度。
 * @param uSize    控制变量维度。
 * @param zSize    观测量维度。
 */
void Kalman_Filter_Reset(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize) {
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    if (uSize != 0) {
        memset(kf->u_data, 0, sizeof_float * uSize);
    }
    memset(kf->z_data, 0, sizeof_float * zSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    if (uSize != 0) {
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
    }
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief 测量更新阶段，依据有效测量数据更新量测向量及控制向量。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf) {
    if (kf->UseAutoAdjustment != 0) {
        H_K_R_Adjustment(kf);
    } else {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }
    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
 * @brief 利用状态转移矩阵和控制输入更新先验状态 xhatminus。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf) {
    if (!kf->SkipEq1) {
        if (kf->uSize > 0) {
            Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        } else {
            Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

/**
 * @brief 利用状态转移矩阵更新预测协方差矩阵 Pminus。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf) {
    if (!kf->SkipEq2) {
        Matrix_Transpose(&kf->F, &kf->FT);
        Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix);
        Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}

/**
 * @brief 计算卡尔曼增益 K。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf) {
    if (!kf->SkipEq3) {
        Matrix_Transpose(&kf->H, &kf->HT);
        Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
        Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
        Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
        Matrix_Inverse(&kf->S, &kf->temp_matrix1);
        Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
        Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}

/**
 * @brief 利用测量更新状态估计 xhat。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf) {
    if (!kf->SkipEq4) {
        Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector);
        Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);
        Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);
        Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}

/**
 * @brief 根据卡尔曼增益更新协方差矩阵 P。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf) {
    if (!kf->SkipEq5) {
        Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);
        Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1);
        Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
 * @brief 执行卡尔曼滤波完整更新周期，包括预测和测量更新。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 * @return 指向滤波后状态向量的指针。
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf) {
    // 0. 更新测量与控制向量
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 1. 预测：xhatminus = F*xhat + B*u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 2. 协方差预测：Pminus = F*P*FT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    // 3. 测量更新：有有效量测则进行融合，否则仅采用预测值
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0) {
        Kalman_Filter_SetK(kf);
        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);
        Kalman_Filter_xhatUpdate(kf);
        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);
        Kalman_Filter_P_Update(kf);
    } else {
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 防止协方差过度收敛
    for (uint8_t i = 0; i < kf->xhatSize; i++) {
        uint16_t idx = i * kf->xhatSize + i;
        if (kf->P_data[idx] < kf->StateMinVariance[i])
            kf->P_data[idx] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);
    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

/**
 * @brief 根据有效测量数据调整测量向量 z、矩阵 H 和噪声矩阵 R。
 *
 * @param kf 指向 KalmanFilter_t 结构体的指针。
 */
static inline void H_K_R_Adjustment(KalmanFilter_t *kf) {
    kf->MeasurementValidNum = 0;
    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; i++) {
        if (kf->z_data[i] != 0) {
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++) {
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
