//
// Created by Rick on 2024/12/20.
//

#ifndef SHOOT_H_
#define SHOOT_H_

#include "main.h"
#include "gimbal_behaviour.h"
#include "struct_typedef.h"
#include "dji_3508.h"
#include "dji_2006.h"
#include "dt7.h"
#include "pid.h"

//摩擦轮电机速度转化m/s，相关系数---add by qiyin 2025.1.3
#define FRICTION_RADIUS    0.03f   //摩擦轮半径值
#ifndef PI
#define PI   3.14159265358979323846f
#endif
//速度转化公式：v = PI*radius*n/30    线速度 = PI * 半径*n /30
#define FRICTION_COEFFICIENT   (PI*FRICTION_RADIUS/30.0f)     //速度转化为m/s总的系数
#define FRICTION_ALL_COEFFICIENT   0.00314159265358979323846264338328f        //速度系数结果，减少计算

#define FRIC_L_TURN 1

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f  //100

//射击摩擦轮打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
//射击提速
#define SHOOT_UP_KEYBOARD         	KEY_PRESSED_OFFSET_R
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             500     //500
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000    //2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//拨弹速度
#define READY_BULLET_SPEED          3.0f
#define TRIGGER_SPEED               -5.0f     //-5
#define CONTINUE_TRIGGER_SPEED      -10.0f
#define AUTO_TRIGGER_SPEED          -20.0f	  //20.0
//摩擦轮转速动态调整系数KP
#define BULLET_KP                 0.2

#define	SHOOT_OUT_SPEED_BUFFER			2.0f
#define SHOOT_OUT_HEAT_SPEED			0.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         2.0f
#define BLOCK_TIME                  1000   //1000
#define REVERSE_TIME                800    //500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_FIVE 					0.628318530717958647692528676655f
#define PI_TEN                      0.314f

//拨弹电机角度PID
#define TRIGGER_ANGLE_PID_KP        15.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        0.3f
#define TRIGGER_ANGLE_PID_MAX_OUT   9.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT  0.0f

//拨弹轮电机PID
#define TRIGGER_SPEED_PID_KP        800.0f   //1000.0f
#define TRIGGER_SPEED_PID_KI        5.0f
#define TRIGGER_SPEED_PID_KD        0.0f
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 4000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE_MAX     40
#define SHOOT_HEAT_REMAIN_VALUE_MIN     5


//射击电机速度环PID，单位：m/s
#define SHOOT_L_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_L_SPEED_PID_KI 5.0f      //10
#define SHOOT_L_SPEED_PID_KD 0.0f
#define SHOOT_L_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_L_SPEED_PID_MAX_IOUT 1000.0f

#define SHOOT_R_SPEED_PID_KP 8000.0f   //15000
#define SHOOT_R_SPEED_PID_KI 5.0f      //10
#define SHOOT_R_SPEED_PID_KD 0.0f
#define SHOOT_R_SPEED_PID_MAX_OUT  16000.0f
#define SHOOT_R_SPEED_PID_MAX_IOUT 1000.0f

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f //3508转换为速度

typedef enum {
	SHOOT_STOP  = 0,  //停止射击，待开
	OPEN_FRIC,        //打开摩擦轮
	OPEN_TRIGGER,     //打开拨弹电机
} shoot_mode_e;

typedef enum
{
	MOUSE_NO_PRESS = 0,    //未按下
	MOUSE_SHORT_PRESS = 1, //短按
	MOUSE_LONG_PRESS = 2,  //长按
}mouse_state_e;

typedef struct {
	const RC_ctrl_t *shoot_rc_ctrl;		//指向发射机构遥控控制输入的常量指针
	motor_2006_t trigger_motor;		    //拨盘电机结构体
	motor_3508_t friction_motor[2];		//两个摩擦轮电机结构体
	shoot_mode_e shoot_mode;			//发射的状态机枚举
	mouse_state_e mouse_state;			//鼠标按键状态机

	pid_type_def friction_speed_pid[2]; //两个摩擦轮电机的PID
	pid_type_def trigger_motor_pid;		//拨弹电机PID

	bool_t press_l;						//鼠标左键状态
	bool_t press_r;						//鼠标右键状态
	bool_t last_press_l;				//上一次鼠标左键状态
	bool_t last_press_r;				//上一次鼠标右键状态
	uint16_t press_l_time;				//鼠标左键长按计时
	uint16_t press_r_time;				//鼠标右键长按计时

	uint16_t block_time;				// 阻塞时间
	uint16_t reverse_time;				// 反转时间

	uint16_t shoot_speed_limit;			//射速限制
	uint16_t last_shoot_speed_limit;	//上一次射速限制

	uint16_t heat_limit;				//热量限制
	uint16_t heat;						//热量

	void (*CAN_cmd_shoot)(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //CAN发送函数指针
} shoot_control_t;

shoot_control_t *get_shoot_control_point(void);

void shoot_init(shoot_control_t *shoot_init);

void shoot_feedback_update(shoot_control_t *shoot_feedback);

void shoot_control_loop(shoot_control_t *control_loop);

void shoot_set_mode(shoot_control_t *set_mode);

#endif //SHOOT_H_
