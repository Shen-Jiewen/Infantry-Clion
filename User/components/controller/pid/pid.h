/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

enum PID_MODE
{
	PID_POSITION = 0,
	PID_DELTA
};

typedef struct
{
	uint8_t mode;
	//PID 三参数
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;

	fp32 max_out;  //最大输出
	fp32 max_iout; //最大积分输出

	fp32 set;
	fp32 fdb;

	fp32 out;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
	fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

/**
  * @brief          云台PID计算函数
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据（云台角度反馈）
  * @param[in]      set: 设定角度值
  * @param[in]      error_delta: 外部计算的误差变化率（微分项）
  * @retval         pid输出
  *
  * @note           该函数内部会对角度误差进行归一化（调用 rad_format），
  *                 以适应角度数据的环绕特性，同时保证接口与其它PID函数统一。
  */
extern fp32 Gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 error_delta);

/**
  * @brief          云台PID结构初始化函数（去掉模式参数）
  * @param[out]     pid: PID结构数据指针
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  *
  * @note           该函数默认将模式设置为PID_POSITION，适用于云台角度控制。
  */
extern void Gimbal_PID_init(pid_type_def *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);


#endif
