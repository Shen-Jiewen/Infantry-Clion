#include "cmsis_os.h"
#include "vofa.h"
#include "dt7.h"
#include "imu.h"
#include "feedforward.h"

/**
 * @brief USB数据传输任务
 */
_Noreturn void usb_task(__attribute__((unused)) void *argument) {
	/* 初始启动延时，等待其他模块初始化 */
	osDelay(1000);

	/* 获取遥控器和IMU数据指针 */
	const RC_ctrl_t *usb_dt7_ctrl = get_dt7_point();
	const imu_control_t *imu_control = get_imu_control_point();

	/* 初始化VOFA功能 */
	vofa_init();

	/* 主循环，持续发送数据 */
	while (1) {
		/* 添加IMU姿态角数据 */
		vofa_append_data(imu_control->angle[0]);
		vofa_append_data(imu_control->angle[1]);
		vofa_append_data(imu_control->angle[2]);

		/* 添加遥控器通道数据 */
		vofa_append_data(usb_dt7_ctrl->rc.ch[0]);
		vofa_append_data(usb_dt7_ctrl->rc.ch[1]);
		vofa_append_data(usb_dt7_ctrl->rc.ch[2]);
		vofa_append_data(usb_dt7_ctrl->rc.ch[3]);
		vofa_append_data(usb_dt7_ctrl->rc.ch[4]);

		/* 添加遥控器开关数据 */
		vofa_append_data(usb_dt7_ctrl->rc.s[0]);
		vofa_append_data(usb_dt7_ctrl->rc.s[1]);

		/* 添加IMU温度和云台电流数据 */
		vofa_append_data(imu_control->temperature);
		vofa_append_data(get_gimbal_pitch_current());
		vofa_append_data(get_gimbal_yaw_current());

		/* 发送一帧数据 */
		vofa_send_frame();

		/* 任务周期延时 */
		osDelay(10);
	}
}
