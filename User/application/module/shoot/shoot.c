//
// Created by Rick on 2024/12/20.
//

#include "shoot.h"

#include "fdcan.h"
#include "gimbal_behaviour.h"
#include "referee.h"

static shoot_control_t shoot_control;

static void trigger_motor_turn_back(shoot_control_t *shoot_back);

/**
  * @brief          返回射击控制结构体指针
  * @retval         void
  */
shoot_control_t *get_shoot_control_point(void) {
	return &shoot_control;
}

/**
 * @brief          射击任务初始化函数
 * @param shoot_init 射击控制的结构体指针
 */
void shoot_init(shoot_control_t *shoot_init) {
	if (shoot_init == NULL) {
		return;
	}

	//定义各个PID参数
	static const fp32 Trigger_speed_pid[3] =
			{TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	static const fp32 Shoot_L_speed_pid[3] =
			{SHOOT_L_SPEED_PID_KP, SHOOT_L_SPEED_PID_KI, SHOOT_L_SPEED_PID_KD};
	static const fp32 Shoot_R_speed_pid[3] =
			{SHOOT_R_SPEED_PID_KP, SHOOT_R_SPEED_PID_KI, SHOOT_R_SPEED_PID_KD};

	//获取遥控器指针
	shoot_init->shoot_rc_ctrl = get_dt7_point();

	//初始化拨盘电机类型和测量数据
	shoot_init->trigger_motor.motor_2006_measure = get_motor_2006_measure_point(2);

	//初始化摩擦轮电机的测量数据
	for (uint8_t i = 0; i < 2; i++) {
		shoot_init->friction_motor[i].motor_3508_measure = get_motor_3508_measure_point(1, i);
	}

	//初始状态机为发射停止
	shoot_init->shoot_mode = SHOOT_STOP;

	//初始化PID
	PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT,
	         TRIGGER_BULLET_PID_MAX_IOUT);

	PID_init(&shoot_control.friction_speed_pid[0], PID_POSITION, Shoot_L_speed_pid, SHOOT_L_SPEED_PID_MAX_OUT,
	         SHOOT_L_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.friction_speed_pid[1], PID_POSITION, Shoot_R_speed_pid, SHOOT_R_SPEED_PID_MAX_OUT,
	         SHOOT_R_SPEED_PID_MAX_IOUT);

	//数据更新
	shoot_feedback_update(shoot_init);

	//初始化两个摩擦轮电机的速度和目标速度
	for (uint8_t i = 0; i < 2; i++) {
		shoot_init->friction_motor[i].speed = 0.0f;
		shoot_init->friction_motor[i].speed_set = 0.0f;
	}

	//初始化拨盘电机
	shoot_init->trigger_motor.give_current = 0;
	shoot_init->trigger_motor.speed = 0.f;
	shoot_init->trigger_motor.speed_set = 0.f;
	shoot_init->friction_motor[0].speed = 0.f;
	shoot_init->friction_motor[0].speed_set = 0.f;
	shoot_init->friction_motor[0].give_current = 0;
	shoot_init->friction_motor[1].speed = 0.f;
	shoot_init->friction_motor[1].speed_set = 0.f;
	shoot_init->friction_motor[1].give_current = 0;
	shoot_init->last_press_l = shoot_init->last_press_r = 0;
	shoot_init->press_l_time = shoot_init->press_r_time = 0;

	//获取当前弹速限制以及热量限制
	get_shoot_speed_limit(&shoot_init->last_shoot_speed_limit);
	get_shoot_speed_limit(&shoot_init->shoot_speed_limit);
	get_shoot_heat0_limit_and_heat0(&shoot_init->heat_limit, &shoot_init->heat);
}

/**
  * @brief          射击数据更新
  * @param			shoot_feedback
  * @retval         void
  */
void shoot_feedback_update(shoot_control_t *shoot_feedback) {
	if (shoot_feedback == NULL) {
		return;
	}

	//更新摩擦轮电机速度
	for (uint8_t i = 0; i < 2; i++) {
		shoot_feedback->friction_motor[i].speed =
				(float) shoot_feedback->friction_motor[i].motor_3508_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	}

	//记录上次的键盘状态
	shoot_feedback->last_press_l = shoot_feedback->press_l;
	shoot_feedback->last_press_r = shoot_feedback->press_r;

	//判断鼠标状态
	shoot_feedback->press_l = shoot_feedback->shoot_rc_ctrl->mouse.press_l;
	shoot_feedback->press_r = shoot_feedback->shoot_rc_ctrl->mouse.press_r;

	//鼠标状态检测
	if (!shoot_feedback->last_press_l && shoot_feedback->press_l) //上次未按下，这次按下--短按
	{
		shoot_feedback->mouse_state = MOUSE_SHORT_PRESS;
	} else if (shoot_feedback->last_press_l && shoot_feedback->press_l) //上次按下，这次按下，时间自加
	{
		shoot_feedback->press_l_time++;
		if (shoot_feedback->press_l_time > PRESS_LONG_TIME) //时间大于阈值，标记为长按状态
		{
			shoot_feedback->mouse_state = MOUSE_LONG_PRESS;
		}
	} else if (shoot_feedback->last_press_l) //上次按下，这次未按下
	{
		shoot_feedback->press_l_time = 0; //清空计时，标记为没有按下
		shoot_feedback->mouse_state = MOUSE_NO_PRESS;
	}

	// 新增：定义长度为20的弹速历史数组及管理变量
	static fp32 bullet_speed_history[20] = {
		24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f,
		24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f, 24.0f
	};
	static uint8_t history_index = 0;
	static fp32 last_bullet_speed = 0.0f;
	fp32 current_bullet_speed = 0.0f;
	fp32 average_speed = 0.0f;

	// 获取当前弹速
	get_current_bullet_speed(&current_bullet_speed);

	// 当裁判系统读取到的速度与上一次不一致时，更新数组
	if (current_bullet_speed != last_bullet_speed && current_bullet_speed > 0.1f) {
		// 更新历史数组
		bullet_speed_history[history_index] = current_bullet_speed;

		// 更新索引，循环使用数组
		history_index = (history_index + 1) % 20;

		// 更新上一次速度记录
		last_bullet_speed = current_bullet_speed;
	}

	// 计算数组平均值
	for (uint8_t i = 0; i < 20; i++) {
		average_speed += bullet_speed_history[i];
	}
	average_speed /= 20.0f;

	// 设置弹速：2*22-数组的均值
	shoot_feedback->friction_motor[0].speed_set = 2 * 21.5f - average_speed;
	shoot_feedback->friction_motor[1].speed_set = 2 * 21.5f - average_speed;
}

/**
  * @brief 射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
    *	实际上只用到了三个状态机模式
    *	SHOOT_STOP，初始停止模式，摩擦轮和拨弹电机都不转动
    *	SHOOT_READY_FRIC，摩擦轮开启模式，前后四个摩擦轮转动
    *	SHOOT_READY_BULLET，准备射击模式，摩擦轮转动稳定会跳到下一个模式
    *	SHOOT_BULLET，射击模式，摩擦轮和拨弹电机转动，弹丸被发射出去
  * @param			set_mode
  * @retval         void
  */
void shoot_set_mode(shoot_control_t *set_mode) {
	static int8_t last_s = RC_SW_UP;

	// 检测上位拨杆是否发生上升沿(从下/中拨到上拨)
	if (switch_is_up(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s)) {
		// 切换射击模式：如果当前是停止状态，则开启摩擦轮；否则，停止射击。
		if (set_mode->shoot_mode == SHOOT_STOP) {
			set_mode->shoot_mode = OPEN_FRIC; // 开启摩擦轮。
		} else {
			set_mode->shoot_mode = SHOOT_STOP; // 停止射击。
		}
	}

	// 检测中位拨杆是否处于中间档位。
	else if (switch_is_mid(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])) {
		// 如果按下“射击开启”的键盘按键，并且当前射击模式是停止状态，则开启摩擦轮。
		if ((set_mode->shoot_rc_ctrl->key.v & SHOOT_ON_KEYBOARD) && set_mode->shoot_mode == SHOOT_STOP) {
			set_mode->shoot_mode = OPEN_FRIC;
		}
		// 如果按下“射击关闭”的键盘按键，并且当前射击模式不是停止状态，则停止射击。
		else if ((set_mode->shoot_rc_ctrl->key.v & SHOOT_OFF_KEYBOARD) && set_mode->shoot_mode != SHOOT_STOP) {
			set_mode->shoot_mode = SHOOT_STOP;
		}
	}

	// 获取射击热量限制和当前热量
	get_shoot_heat1_limit_and_heat1(&set_mode->heat_limit, &set_mode->heat);

	// 判断摩擦轮的实际转速是否达到设定的目标转速。
	if (set_mode->shoot_mode >= OPEN_FRIC &&
	    fabsf(set_mode->friction_motor[0].speed - set_mode->friction_motor[0].speed_set) < 0.1f &&
	    fabsf(set_mode->friction_motor[1].speed - set_mode->friction_motor[1].speed_set) < 0.1f) {
		uint8_t can_shoot = 0; // 标记是否允许射击（基于裁判系统和热量）
#if REFEREE
       // 如果裁判系统正常工作，并且剩余热量足够，则允许射击。
       if (!toe_is_error(REFEREE_TOE) && (set_mode->heat + SHOOT_HEAT_REMAIN_VALUE_MAX <= set_mode->heat_limit))
       {
          can_shoot = 1;
       }
       else {
          can_shoot = 0;
       }
#else
		// 如果裁判系统未启用，则始终允许射击
		can_shoot = 1;
#endif

		// 如果允许射击，并且下位拨杆处于下拨状态或者鼠标左键被按下。
		if ((switch_is_down(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL]) || set_mode->press_l) &&
		    can_shoot) {
			// TODO:添加自瞄标志位
			// 如果云台处于自动模式且控制状态为2（可能表示火控模式），则打开发射机构.
			if ((get_gimbal_behaviour() == GIMBAL_AUTO)) {
				set_mode->shoot_mode = OPEN_TRIGGER;
			}
			// 如果云台不处于自动模式，则打开发射机构。
			else if (get_gimbal_behaviour() != GIMBAL_AUTO) {
				set_mode->shoot_mode = OPEN_TRIGGER;
			}
			// 如果处于自动瞄准模式但未锁定目标,则保持摩擦轮开启
			else {
				set_mode->shoot_mode = OPEN_FRIC;
			}
		}
		// 如果不满足打开发射机构的条件，则保持摩擦轮开启状态。
		else {
			set_mode->shoot_mode = OPEN_FRIC;
		}
	}
}

/**
 * @brief 拨弹电机防堵转函数
 *
 */
static void trigger_motor_turn_back(shoot_control_t *shoot_back) {
	// 判断阻塞时间是否大于设定的最大阻塞时间
	if (shoot_back->block_time > BLOCK_TIME) {
		shoot_back->trigger_motor.speed_set = -TRIGGER_SPEED;
	}

	// 判断当前拨弹电机的速度是否小于设定的最小速度,并且阻塞时间小于最大的阻塞时间
	if (fabsf(shoot_back->trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_back->block_time < BLOCK_TIME) {
		// 阻塞时间增加
		shoot_back->block_time++;
		// 反转时间清空
		shoot_back->reverse_time = 0;
	} else if (shoot_back->block_time == BLOCK_TIME && shoot_back->reverse_time < REVERSE_TIME) {
		// 阻塞时间已满,反转时间未结束
		shoot_back->reverse_time++;
	} else {
		// 以上条件都不满足,表示一个新的控制周期开始,阻塞时间清零
		shoot_back->block_time = 0;
	}
}

/**
 * @brief 射击控制循环函数，根据不同的状态机实现输出函数中输入数据的控制
 * @param control_loop
 */
void shoot_control_loop(shoot_control_t *control_loop) {
	if (control_loop == NULL) {
		return;
	}

	if (control_loop->shoot_mode == SHOOT_STOP) {
		// 设置拨弹轮的速度
		control_loop->trigger_motor.speed_set = 0.0f;
		control_loop->friction_motor[0].speed_set = 0.0f;
		control_loop->friction_motor[1].speed_set = 0.0f;
	} else if (control_loop->shoot_mode == OPEN_FRIC) {
		control_loop->trigger_motor.speed_set = 0.0f;
	} else if (control_loop->shoot_mode == OPEN_TRIGGER) {
		if (get_gimbal_behaviour() == GIMBAL_AUTO) {
			control_loop->trigger_motor.speed_set = AUTO_TRIGGER_SPEED;
		} else {
			if (control_loop->mouse_state == MOUSE_SHORT_PRESS || switch_is_down(
				    control_loop->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])) {
				control_loop->trigger_motor.speed_set = TRIGGER_SPEED;
			} else if (control_loop->mouse_state == MOUSE_LONG_PRESS) {
				control_loop->trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
			}
		}
		trigger_motor_turn_back(control_loop);
	}

	// 由旋转切换到停止状态
	if (control_loop->shoot_mode == SHOOT_STOP) {
		control_loop->trigger_motor.give_current = 0;

		if (fabsf(control_loop->friction_motor[0].speed) < 1.0f && fabsf(control_loop->friction_motor[1].speed) <
		    1.0f) {
			control_loop->friction_motor[0].give_current = 0;
			control_loop->friction_motor[1].give_current = 0;
		} else {
			control_loop->friction_motor[0].give_current = (int16_t) PID_calc(
				&control_loop->friction_speed_pid[0], control_loop->friction_motor[0].speed,
				control_loop->friction_motor[0].speed_set);
			control_loop->friction_motor[1].give_current = (int16_t) PID_calc(
				&control_loop->friction_speed_pid[1], control_loop->friction_motor[1].speed,
				control_loop->friction_motor[1].speed_set);
		}
	} else {
		// 拨弹电机闭环控制
		PID_calc(&control_loop->trigger_motor_pid, control_loop->trigger_motor.speed,
		         control_loop->trigger_motor.speed_set);
		control_loop->trigger_motor.give_current = (int16_t)control_loop->trigger_motor_pid.out;

		//旋转方向由电机屁股看向输出轴 电流为正对应逆时针旋转
#if FRIC_L_TURN
		shoot_control.friction_motor[0].speed_set = -shoot_control.friction_motor[0].speed_set;
#else
		shoot_control.friction_motor[1].speed_set = -shoot_control.friction_motor[1].speed_set;
#endif
		control_loop->friction_motor[0].give_current = (int16_t)PID_calc(&control_loop->friction_speed_pid[0], control_loop->friction_motor[0].speed, control_loop->friction_motor[0].speed_set);
		control_loop->friction_motor[0].give_current = (int16_t)PID_calc(&control_loop->friction_speed_pid[1], control_loop->friction_motor[1].speed, control_loop->friction_motor[1].speed_set);
	}
}

/**
 * @brief 发射机构的CAN发送函数
 * @param motor1
 * @param motor2
 * @param motor3
 * @param motor4
 */
static void FDCAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8]; // 数据缓存

	// 配置 FDCAN 消息头
	txHeader.Identifier = CAN_SHOOT_ALL_ID; // 示例标准 ID (根据实际需求修改)
	txHeader.IdType = FDCAN_STANDARD_ID; // 标准帧 (11位 ID)
	txHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
	txHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度为 8 字节
	txHeader.BitRateSwitch = FDCAN_BRS_OFF; // 无速率切换（经典 CAN）
	txHeader.FDFormat = FDCAN_FD_CAN; // 使用经典 CAN 帧格式
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 无错误
	txHeader.MessageMarker = 0;

	// 填充发送数据
	txData[0] = motor1 >> 8; // Motor1 高位
	txData[1] = motor1 & 0xFF; // Motor1 低位
	txData[2] = motor2 >> 8; // Motor2 高位
	txData[3] = motor2 & 0xFF; // Motor2 低位
	txData[4] = motor3 >> 8; // Motor3 高位
	txData[5] = motor3 & 0xFF; // Motor3 低位
	txData[6] = motor4 >> 8; // Motor4 高位
	txData[7] = motor4 & 0xFF; // Motor4 低位

	// 将消息添加到发送 FIFO 队列
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK) {
	}
}

static shoot_control_t shoot_control = {
	.CAN_cmd_shoot = FDCAN_cmd_shoot,
};
