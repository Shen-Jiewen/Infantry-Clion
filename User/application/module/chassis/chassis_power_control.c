/**
******************************************************************************
  * @file    chassis_power_control.c
  * @brief   底盘功率控制模块实现
  * @details 此模块用于RoboMaster比赛中的机器人底盘功率管理与优化分配
  * @note
  * 功能概述：
  * - 实时监控并控制底盘功率，确保不超过裁判系统功率上限
  * - 智能管理超级电容能量，提供不同模式下的功率加成
  * - 根据电机需求动态分配功率，保证整体性能最优
  * - 支持正常模式和暴走(Boost)模式两种工作状态
  *
  * 使用说明：
  * 1. 初始化要求：
  *    - 调用前需初始化各电机CAN接口
  *    - 设置正确的能量阈值和功率加成参数
  *    - 确保裁判系统数据接收正常，能够获取实时功率数据
  *
  * 2. 接口说明：
  *    - chassis_power_control()为主入口函数，需在主控制循环中周期性调用（推荐大于500Hz）
  *    - 需实现get_chassis_power_and_buffer()获取当前功率和最大限制功率
  *    - 需对接电容控制模块CAP_CAN_DataSend()进行电容充放电管理
  *    - 使用 KEY_SPEED_UP 按键可触发暴走模式，提供更高功率输出
  *
  * 3. 参数整定指南：
  *    - 能量阈值（CAP_ENERGY_THRES）：根据电容特性设置，默认为5%
  *    - 功率加成配置：根据比赛策略和机器人定位调整各能量区间的加成值
  *    - 电机功率模型参数：需通过实验标定，当前值适用于3508电机
  *
  * 4. 工作流程：
  *    获取功率状态 → 计算输入功率 → 电容控制 → 确定最大功率 → 电机功率计算 → 功率分配与限制
  *
  * 5. 电机功率模型：
  *    P = τ*ω + k2*ω² + a*τ² + const
  *    其中：
  *    - τ 为电机扭矩（与PID输出成正比）
  *    - ω 为电机角速度
  *    - k2, a, const 为实验标定的常数项
  ******************************************************************************
  */

#include "chassis_power_control.h"
#include "referee.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* 常量定义 ----------------------------------------------------------------*/
/**
 * @brief 电机最大扭矩输出值
 * @note 用于限制最终计算的电机扭矩，防止过载
 */
#define MAX_MOTOR_TORQUE          16000.0f

/**
 * @brief 超级电容低电量阈值（百分比）
 * @note 当电容电量低于此值时进入低能量策略
 */
#define CAP_ENERGY_THRES      5.0f

/**
 * @brief 值限制宏，确保x在[min, max]范围内
 */
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 不同能量状态下的功率加成配置结构体
 * @details 用于配置不同能量状态和模式下的功率加成值
 */
typedef struct {
    float normal_mode_add;  /**< 正常模式下的功率加成值(W) */
    float boost_mode_add;   /**< 暴走模式下的功率加成值(W) */
} PowerBoostConfig;

/**
 * @brief 功率加成配置表
 * @details 索引0对应电容能量充足状态，索引1对应电容能量不足状态
 * @note 数值单位为瓦特(W)，根据比赛规则和策略调整
 */
static const PowerBoostConfig POWER_BOOST_CONFIG[] = {
    [0] = {100.0f, 150.0f},  /**< 电容能量充足：正常+100W，暴走+150W */
    [1] = {0.0f, 0.0f}       /**< 电容能量不足：无功率加成 */
};

/* 私有函数声明 ------------------------------------------------------------*/
/**
 * @brief 计算输入功率
 * @details 根据当前底盘功率和最大功率限制，计算适合的输入功率
 * @param max_power_limit 最大功率限制(W)
 * @param chassis_power 当前底盘功率(W)
 * @return 计算后的输入功率(W)
 */
static float calculate_input_power(float max_power_limit, float chassis_power);

/**
 * @brief 确定底盘最大可用功率
 * @details 根据输入功率、电容状态和能量百分比，确定底盘可用的最大功率
 * @param input_power 基础输入功率(W)
 * @param cap_state 电容器状态(0:正常模式, 1:暴走模式)
 * @param cap_energy 电容器剩余能量百分比(0-100)
 * @return 最大可用功率(W)
 */
static float determine_chassis_max_power(float input_power, uint8_t cap_state, float cap_energy);

/**
 * @brief 计算各电机初始功率需求
 * @details 基于电机模型计算每个电机的功率需求和总功率需求
 * @param control 底盘控制结构体指针
 * @param powers 电机功率数组(输出参数)
 * @param total_power 总功率(输出参数)
 */
static void calculate_motor_powers(const chassis_control_t *control, float *powers, float *total_power);

/**
 * @brief 缩放电机功率并重新计算扭矩
 * @details 当总功率超过限制时，按比例缩放各电机功率并重新计算对应的扭矩输出
 * @param control 底盘控制结构体指针
 * @param powers 原始功率数组
 * @param total_power 原始总功率(W)
 * @param max_power 允许的最大功率(W)
 */
static void scale_motor_powers(chassis_control_t *control, const float *powers, float total_power, float max_power);

/**
 * @brief 底盘功率主控制函数
 * @details 整体功率控制流程的入口函数，需在主循环中周期性调用
 * @param chassis_power_control 底盘控制结构体指针
 */
void chassis_power_control(chassis_control_t *chassis_power_control) {
    /* 初始化局部变量 */
    float chassis_power = 0.0f;      // 当前底盘功率
    float max_power_limit = 40.0f;   // 最大功率限制(默认值，会被更新)
    float initial_power_per_motor[4] = {0};  // 各电机初始功率需求
    float total_motor_power = 0.0f;  // 总电机功率需求

    /* 步骤1：获取底盘功率状态 */
    get_chassis_power_and_buffer(&chassis_power, &max_power_limit);

    /* 步骤2：计算输入功率 */
    const float input_power = calculate_input_power(max_power_limit, chassis_power);

    /* 步骤3：电容器控制
     * 说明：发送电容充电功率和使能信号
     * 最理想的情况是超电以底盘最大限制功率充电
     * 如果充满电了，发送的充电功率应为(1.7 * max_power_limit)
     */
    CAP_CAN_DataSend(&hfdcan1, input_power, ENABLE_CAP);

    /* 步骤4：确定底盘最大功率
     * 检测Shift键是否按下，决定是否进入暴走模式
     */
    const uint8_t boost_mode = (chassis_power_control->chassis_RC->key.v & KEY_SPEED_UP) ? 1 : 0;
    const float chassis_max_power = determine_chassis_max_power(
        input_power,
        boost_mode,
        CAP_CANData.cap_energy  // 电容剩余能量百分比
    );

    /* 步骤5：计算电机功率需求 */
    calculate_motor_powers(chassis_power_control, initial_power_per_motor, &total_motor_power);

    /* 步骤6：功率限制与分配
     * 当总功率需求超过最大可用功率时，进行功率分配
     */
    if (total_motor_power > chassis_max_power) {
        scale_motor_powers(chassis_power_control, initial_power_per_motor, total_motor_power, chassis_max_power);
    }
}

/**
 * @brief 计算输入功率
 * @details 根据当前底盘功率和最大功率限制，计算适合的输入功率
 * @param max_power_limit 最大功率限制(W)
 * @param chassis_power 当前底盘功率(W)
 * @return 计算后的输入功率(W)
 */
static float calculate_input_power(const float max_power_limit, const float chassis_power) {
    const float POWER_FILTER_GAIN = 0.3f;  /**< 功率滤波增益，控制响应速度 */

    // 自适应功率计算:
    // 在低负载时(底盘功率<90%最大限制)，快速响应以充分利用剩余功率
    // 在高负载时(底盘功率≥90%最大限制)，平滑过渡以避免功率突变
    if (chassis_power < 0.9f * max_power_limit) {
        // 低负载快速响应公式
        return (1.0f - POWER_FILTER_GAIN) * (max_power_limit - chassis_power) + max_power_limit;
    }
    // 高负载平滑过渡公式
    return POWER_FILTER_GAIN * (max_power_limit - chassis_power) + max_power_limit;
}

/**
 * @brief 确定底盘最大可用功率
 * @details 根据输入功率、电容状态和能量百分比，确定底盘可用的最大功率
 * @param input_power 基础输入功率(W)
 * @param cap_state 电容器状态(0:正常模式, 1:暴走模式)
 * @param cap_energy 电容器剩余能量百分比(0-100)
 * @return 最大可用功率(W)
 */
static float determine_chassis_max_power(const float input_power, const uint8_t cap_state, const float cap_energy) {
    uint8_t energy_level = 0;  // 能量等级

    /* 确定能量区间
     * 0: 电容能量充足(>低电量阈值)
     * 1: 电容能量不足(≤低电量阈值)
     */
    if (cap_energy > CAP_ENERGY_THRES) {
        energy_level = 0;  // 能量充足
    } else {
        energy_level = 1;  // 能量不足
    }

    /* 获取功率加成配置
     * 根据能量等级和模式(正常/暴走)选择相应的功率加成值
     */
    const float power_boost = cap_state
                            ? POWER_BOOST_CONFIG[energy_level].boost_mode_add  // 暴走模式加成
                            : POWER_BOOST_CONFIG[energy_level].normal_mode_add; // 正常模式加成

    // 返回基础功率加上加成值
    return input_power + power_boost;
}

/**
 * @brief 计算各电机初始功率需求
 * @details 基于电机模型计算每个电机的功率需求和总功率需求
 * @param control 底盘控制结构体指针
 * @param powers 电机功率数组(输出参数)
 * @param total_power 总功率(输出参数)
 */
static void calculate_motor_powers(const chassis_control_t *control, float *powers, float *total_power) {
    *total_power = 0.0f;  // 初始化总功率为0

    // 计算每个电机的功率
    for (uint8_t i = 0; i < 4; i++) {
        // 电机功率模型常数(通过实验标定)
        const float TORQUE_COEFF = 1.99688994e-6f;
        const float K2 = 1.453e-07f;
        const float A = 1.23e-07f;
        const float CONST_TERM = 4.081f;
        const motor_3508_t *motor = &control->motor_chassis[i];  // 当前电机
        const pid_type_def *pid = &control->motor_speed_pid[i];  // 电机速度PID控制器

        /* 计算单电机功率：P = τ*ω + k2*ω² + a*τ² + const
         * τ: 电机扭矩(与PID输出成正比)
         * ω: 电机角速度
         */
        powers[i] = pid->out * TORQUE_COEFF * motor->speed  // 扭矩功率项
                    + K2 * motor->speed * motor->speed      // 速度平方项
                    + A * pid->out * pid->out               // 扭矩平方项
                    + CONST_TERM;                           // 常数项

        /* 忽略负功率（暂态过程中可能出现）
         * 负功率表示电机在制动或反向工作，可能向系统回馈能量
         * 在功率分配中仅考虑正功率
         */
        if (powers[i] > 0.0f) {
            *total_power += powers[i];
        }
    }
}

/**
 * @brief 缩放电机功率并重新计算扭矩
 * @details 当总功率超过限制时，按比例缩放各电机功率并重新计算对应的扭矩输出
 * @param control 底盘控制结构体指针
 * @param powers 原始功率数组
 * @param total_power 原始总功率(W)
 * @param max_power 允许的最大功率(W)
 */
static void scale_motor_powers(chassis_control_t *control, const float *powers, const float total_power, const float max_power) {
    // 计算功率缩放因子
    const float scale_factor = max_power / total_power;

    // 处理每个电机
    for (uint8_t i = 0; i < 4; i++) {
        // 电机功率模型常数(通过实验标定)
        const float TORQUE_COEFF = 1.99688994e-6f;
        const float K2 = 1.453e-07f;
        const float A = 1.23e-07f;
        pid_type_def *pid = &control->motor_speed_pid[i];      // 电机速度PID控制器
        const motor_3508_t *motor = &control->motor_chassis[i]; // 当前电机

        /* 计算缩放后功率
         * 按比例缩放，保持电机间的功率分配比例不变
         */
        const float scaled_power = powers[i] * scale_factor;

        // 跳过负功率或无效功率
        if (scaled_power < 0.0f) continue;

        /* 构建二次方程以求解扭矩：aτ² + (k1ω)τ + (k2ω² - P_scaled + C) = 0
         * 这是从电机功率模型反推所需扭矩的过程
         * P_scaled = τ*ω + k2*ω² + a*τ² + const
         */
        const float b = TORQUE_COEFF * motor->speed;  // 一次项系数
        const float c = K2 * motor->speed * motor->speed - scaled_power + 4.081f;  // 常数项

        /* 求解二次方程，根据原始扭矩方向选择合适的解
         * 二次方程ax²+bx+c=0的判别式为b²-4ac
         */
        const float discriminant = b * b - 4 * A * c;
        if (discriminant < 0.0f) {
            pid->out = 0.0f;  // 判别式小于0，无实数解，将扭矩设为0
            continue;
        }

        // 计算平方根
        const float sqrt_d = sqrtf(discriminant);

        // 根据原扭矩方向选择合适的解
        // 正向扭矩选择较大的解，负向扭矩选择较小的解
        float temp = pid->out > 0 ? (-b + sqrt_d) / (2 * A) : (-b - sqrt_d) / (2 * A);

        /* 扭矩限幅，确保在允许范围内 */
        temp = CLAMP(temp, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        pid->out = temp;  // 更新PID输出为计算后的扭矩值
    }
}