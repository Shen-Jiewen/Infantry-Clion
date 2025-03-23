/**
******************************************************************************
 * @file    kalman_filter.h
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

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "stdint.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

/**
 * @brief 卡尔曼滤波器数据结构
 */
typedef struct kf_t {
    float *FilteredValue; ///< 滤波后的状态向量 x(k|k)
    float *MeasuredVector; ///< 量测向量
    float *ControlVector; ///< 控制向量

    uint8_t xhatSize; ///< 状态向量维度
    uint8_t uSize; ///< 控制向量维度
    uint8_t zSize; ///< 量测向量维度

    uint8_t UseAutoAdjustment; ///< 自动调整测量矩阵标志
    uint8_t MeasurementValidNum; ///< 有效量测数目

    uint8_t *MeasurementMap; ///< 量测与状态映射关系
    float *MeasurementDegree; ///< 测量系数，对应 H 矩阵元素
    float *MatR_DiagonalElements; ///< 量测噪声 R 对角元素
    float *StateMinVariance; ///< 状态最小方差，防止协方差过收敛
    uint8_t *temp; ///< 临时存储有效量测索引

    // 跳过标准 KF 中部分更新步骤的标志位
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // 矩阵实例（状态、协方差、转移矩阵、量测矩阵等）
    mat xhat; ///< 状态估计 x(k|k)
    mat xhatminus; ///< 预测状态 x(k|k-1)
    mat u; ///< 控制向量 u
    mat z; ///< 量测向量 z
    mat P; ///< 协方差矩阵 P(k|k)
    mat Pminus; ///< 预测协方差 P(k|k-1)
    mat F, FT; ///< 状态转移矩阵 F 及其转置
    mat B; ///< 控制矩阵 B
    mat H, HT; ///< 量测矩阵 H 及其转置
    mat Q; ///< 过程噪声协方差 Q
    mat R; ///< 量测噪声协方差 R
    mat K; ///< 卡尔曼增益 K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus; ///< 矩阵操作状态

    // 用户回调函数，用于扩展或替换标准 KF 更新步骤
    void (*User_Func0_f)(struct kf_t *kf);

    void (*User_Func1_f)(struct kf_t *kf);

    void (*User_Func2_f)(struct kf_t *kf);

    void (*User_Func3_f)(struct kf_t *kf);

    void (*User_Func4_f)(struct kf_t *kf);

    void (*User_Func5_f)(struct kf_t *kf);

    void (*User_Func6_f)(struct kf_t *kf);

    // 各矩阵数据存储区指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

void Kalman_Filter_Reset(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

void Kalman_Filter_Measure(KalmanFilter_t *kf);

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);

void Kalman_Filter_SetK(KalmanFilter_t *kf);

void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);

void Kalman_Filter_P_Update(KalmanFilter_t *kf);

float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif // __KALMAN_FILTER_H
