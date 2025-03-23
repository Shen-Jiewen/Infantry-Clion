/**
******************************************************************************
  * @file    chassis_power_control.h
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
  *    - 调用前需初始化各电机CAN接口和PID控制器
  *    - 配置power_buffer_pid参数（通常为PI控制器）
  *    - 设置正确的能量阈值和功率加成参数
  *    - 确保裁判系统数据接收正常，能够获取实时功率数据
  *
  * 2. 接口说明：
  *    - chassis_power_control()为主入口函数，需在主控制循环中周期性调用（推荐1kHz）
  *    - 需实现get_chassis_power_and_buffer()获取当前功率和缓冲能量状态
  *    - 需对接电容控制模块CAP_CAN_DataSend()进行电容充放电管理
  *    - 使用KEY_SPEED_UP按键可触发暴走模式，提供更高功率输出
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

#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

/* 依赖头文件 */
#include "chassis.h"

/* 宏定义 */
/**
 * @brief 暴走模式触发按键宏定义
 */
#define KEY_SPEED_UP                 KEY_PRESSED_OFFSET_SHIFT

/**
 * @brief 超级电容使能标志
 */
#define ENABLE_CAP                   0

/* 函数声明 */
/**
 * @brief 底盘功率控制主函数
 * @param chassis_power_control 底盘控制结构体指针
 */
void chassis_power_control(chassis_control_t *chassis_power_control);

#endif // CHASSIS_POWER_CONTROL_H