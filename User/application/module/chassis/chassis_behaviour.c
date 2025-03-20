//
// Created by Rick on 2024/12/4.
//

#include "chassis_behaviour.h"

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，设定值直接发送到can总线上，所有速度设定为0
  * @param[out]     vx_can_set: 纵向速度，设定值将直接发送到can总线上，设定为0
  * @param[out]     vy_can_set: 横向速度，设定值将直接发送到can总线上，设定为0
  * @param[out]     wz_can_set: 旋转速度，设定值将直接发送到can总线上，设定为0
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_zero_force_control(fp32* vx_can_set,
	fp32* vy_can_set,
	fp32* wz_can_set,
	__attribute__((unused)) chassis_control_t* chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，所有速度设定为0
  * @param[out]     vx_set: 纵向速度，设定值为0
  * @param[out]     vy_set: 横向速度，设定值为0
  * @param[out]     wz_set: 旋转速度，设定值为0
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_no_move_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	__attribute__((unused)) chassis_control_t* chassis_move_rc_to_vector);

/**
  * @brief          控制底盘的纵向、横向速度以及角度（取消摇摆功能）
  *                 该函数不再计算摇摆角度，直接设置角度为0
  * @param[out]     vx_set: 纵向速度指针，计算后的纵向速度值
  * @param[out]     vy_set: 横向速度指针，计算后的横向速度值
  * @param[out]     angle_set: 角度指针，设置为0
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_follow_gimbal_yaw_control(fp32* vx_set,
	fp32* vy_set,
	fp32* angle_set,
	chassis_control_t* chassis_move_rc_to_vector);

/**
  * @brief          控制底盘的纵向、横向速度以及旋转速度，但不跟随角度
  *                 旋转速度由固定参数设定
  * @param[out]     vx_set: 纵向速度指针，计算后的纵向速度值
  * @param[out]     vy_set: 横向速度指针，计算后的横向速度值
  * @param[out]     wz_set: 旋转速度指针，计算后的旋转速度值
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_no_follow_yaw_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	chassis_control_t* chassis_move_rc_to_vector);

/**
  * @brief          控制底盘的纵向、横向速度以及旋转速度，适用于开放式控制模式
  * @param[out]     vx_set: 纵向速度指针，计算后的纵向速度值
  * @param[out]     vy_set: 横向速度指针，计算后的横向速度值
  * @param[out]     wz_set: 旋转速度指针，计算后的旋转速度值
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_open_set_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	chassis_control_t* chassis_move_rc_to_vector);

static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
static chassis_behaviour_e last_chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
extern ext_game_robot_state_t robot_state;									//比赛机器人状态

/**
  * @brief          根据遥控器控制信号设置底盘行为模式
  *                 此函数根据遥控器开关和按键状态，选择底盘的行为模式并更新底盘控制模式。
  *                 同时在云台模式下，底盘可能不执行某些操作。
  *
  * @param[in,out]  chassis_move_mode: 底盘控制模式数据结构指针，输入时包含当前底盘控制信息，输出时更新为新的控制模式
  *
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_control_t* chassis_move_mode)
{
	// 检查输入指针是否为空
	if (chassis_move_mode == NULL)
	{
		return; // 如果指针为空，返回不做任何操作
	}

	// 根据遥控器模式开关设定底盘行为模式，遥控值通道0打上
	if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
        if(chassis_move_mode->auto_gyro_select)
        {
            chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;     //自瞄
        }else
        {
            chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;	              //小陀螺
        }
	}
	else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //遥控值通道0打下
	{
		// 如果开关向下，设置为无力，直接开环设置电流为，而不是闭环不动
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
        //清空标志位
        chassis_move_mode->gyro_flag = 0;
        chassis_move_mode->auto_flag = 0;
	}
	else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //遥控值通道0打中，可以键盘控制
	{
        //判断键盘按下来置位小陀螺标志位
        if((chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)&& !chassis_move_mode->gyro_flag)
        {
            chassis_move_mode->gyro_flag = 1;//小陀螺标志位
        }
        else if((chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) && chassis_move_mode->gyro_flag)
        {
            chassis_move_mode->gyro_flag = 0;
        }
        //通过鼠标开启自瞄
        if(chassis_move_mode->chassis_RC->mouse.press_r )
        {
            chassis_move_mode->auto_flag = 1;    //自瞄标志位
        }
        else if(! chassis_move_mode->chassis_RC->mouse.press_r )
        {
            chassis_move_mode->auto_flag = 0;
        }
        //血量为零-阵亡清空标志位，防止复活疯车，因为C板有NUC虚拟串口不会断电
        if(robot_state.remain_HP == 0)    //阵亡清空标志位，防止复活后状态混乱
        {
            chassis_move_mode->gyro_flag = 0;
            chassis_move_mode->auto_flag = 0;
        }
        if(chassis_move_mode->gyro_flag){
            chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
        }
        else{
            chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        }
	}

	// 当云台处于某些模式时，底盘不移动
	if (gimbal_cmd_to_chassis_stop()) {
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;   //某些模式下底盘无力，如初始化，校准等
	}
	// 更新上一次的底盘行为模式
	last_chassis_behaviour_mode = chassis_behaviour_mode;

	// 根据当前的底盘行为模式设置底盘控制模式
	if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		// 零力矩模式，底盘处于原始控制模式，底盘按照原始控制信号进行控制
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
	}
	else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		// 底盘不移动模式，底盘锁死不允许移动 | 小陀螺模式，底盘旋转不跟随云台
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
	{
		// 底盘跟随云台模式，底盘与云台同步
		chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
	}
}

void chassis_behaviour_control_set(fp32* vx_set,
	fp32* vy_set,
	fp32* angle_set,
	chassis_control_t* chassis_move_rc_to_vector)
{
	// 检查输入指针是否为空
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	// 根据底盘行为模式设置底盘控制
	if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
	{
		chassis_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_zero_force_control(fp32* vx_can_set,
	fp32* vy_can_set,
	fp32* wz_can_set,
	__attribute__((unused)) chassis_control_t* chassis_move_rc_to_vector)
{
	*vx_can_set = 0.0f;
	*wz_can_set = 0.0f;
	*vy_can_set = 0.0f;
}

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	__attribute__((unused)) chassis_control_t* chassis_move_rc_to_vector)
{
	*vx_set = 0.0f;
	*vy_set = 0.0f;
	*wz_set = 0.0f;
}

/**
  * @brief          控制底盘的纵向、横向速度以及角度（取消摇摆功能）
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     angle_set: 角度指针（目前不再用于摇摆功能）
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据指针
  * @retval         none
  */
static void chassis_follow_gimbal_yaw_control(fp32* vx_set,
	fp32* vy_set,
	fp32* angle_set,
	chassis_control_t* chassis_move_rc_to_vector)
{
	// 获取底盘的纵向和横向速度
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	// 与云台的相对角度设置为0
	*angle_set = 0.0f;
}

/**
  * @brief          设置底盘速度并控制其旋转速度，不跟随角度模式下，底盘的旋转速度由固定参数设定
  *
  * @param[in]      vx_set: 纵向速度，正值为前进，负值为后退
  * @param[in]      vy_set: 横向速度，正值为左移，负值为右移
  * @param[out]     wz_set: 底盘旋转速度，正值为逆时针旋转，负值为顺时针旋转
  * @param[in]      chassis_move_rc_to_vector: 底盘控制数据结构指针，包含遥控器的输入
  * @retval         none
  */
static void chassis_no_follow_yaw_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	chassis_control_t* chassis_move_rc_to_vector)
{
	// 计算纵向和横向速度
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	// 设置固定的旋转速度，逆时针旋转为正，顺时针旋转为负，为了方便对抗赛过检录设置正反小陀螺
    if(switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        *wz_set = CHASSIS_GYRO_WZ_SPEED;
    }else if(switch_is_down(chassis_move_rc_to_vector->chassis_RC->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        *wz_set = -CHASSIS_GYRO_WZ_SPEED;
    }
}
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */
static void chassis_open_set_control(fp32* vx_set,
	fp32* vy_set,
	fp32* wz_set,
	chassis_control_t* chassis_move_rc_to_vector)
{
	*vx_set = (fp32)chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
	*vy_set = (fp32)-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
	*wz_set = (fp32)-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
}



