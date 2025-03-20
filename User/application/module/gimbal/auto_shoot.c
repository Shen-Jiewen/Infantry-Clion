#include "auto_shoot.h"
#include "referee.h"
#include "arm_math.h"
#include "user_lib.h"
#include "imu.h"
#include <string.h>
#include "usbd_cdc_if.h"

// 单例模式，静态全局变量
static auto_shoot_t auto_shoot;

static void solver_track_init(solver_track_t *solver_track);

// CRC16校验表
static const uint16_t s_crc16_table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief  获取自瞄实例
 * @retval 自瞄实例指针
 */
auto_shoot_t *auto_shoot_get_instance(void) {
    return &auto_shoot;
}

/**
 * @brief  自瞄模块初始化
 * @retval 无
 */
void auto_shoot_init(auto_shoot_t *auto_init) {
    // 初始化自瞄FIFO
    fifo_s_init(&auto_init->auto_shoot_fifo, auto_init->auto_shoot_fifo_buf, AUTO_SHOOT_FIFO_BUF_LENGTH);
    // 弹道解算参数初始化
    solver_track_init(&auto_shoot.solver_track);

    // 清空数据结构体
    memset(&auto_init->received_packed, 0, sizeof(received_packed_t));
    memset(&auto_init->send_packed, 0, sizeof(send_packed_t));
    memset(&auto_init->unpack_data, 0, sizeof(unpack_autoshoot_data_t));

    // 初始化解包数据长度
    auto_init->unpack_data.data_len = sizeof(received_packed_t);
}

/**
 *
 * @param solver_track 弹道解算数据结构体指针
 */
static void solver_track_init(solver_track_t *solver_track) {
    solver_track->bullet_type = BULLET_17; // 弹丸类型
    solver_track->current_v = BULLET_SPEED; // 弹丸初速度
    solver_track->bias_time = TRIGGER_DELAY; // 发单延迟时间
    solver_track->s_bias = BULLET_FORWARD_DISTANCE;
    solver_track->z_bias = CAMERA_TO_YAW_AXIS_HEIGHT;
}


/**
 * @brief  CRC16校验和计算
 * @param  pchMessage: 需要校验的数据
 * @param  dwLength: 数据长度
 * @param  wCRC: 初始校验值
 * @retval 计算得到的校验和
 */
uint16_t auto_shoot_get_crc16(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    if (pchMessage == NULL) {
        return 0xFFFF;
    }

    while (dwLength--) {
        const uint8_t chData = *pchMessage++;
        wCRC = (wCRC >> 8) ^ s_crc16_table[(wCRC ^ (uint16_t) chData) & 0x00ff];
    }

    return wCRC;
}

/**
 * @brief  自瞄FIFO数据解包
 * @retval 无
 */
void auto_shoot_unpack_fifo_data(unpack_autoshoot_data_t *auto_shoot_unpack, fifo_s_t *auto_shoot_fifo,
                                 received_packed_t *received_packed) {
    uint16_t check_sum = 0;
    uint8_t byte = 0;
    unpack_autoshoot_data_t *p_obj = auto_shoot_unpack; // 直接使用传入的指针

    while (fifo_s_used(auto_shoot_fifo)) {
        byte = fifo_s_get(auto_shoot_fifo);

        switch (p_obj->unpack_step) {
            // 包头检验
            case 0: {
                if (byte == HEADER_SOF) {
                    p_obj->unpack_step = 1;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            }
            break;

            // 数据接收
            case 1: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index >= sizeof(received_packed_t)) {
                    // 使用 sizeof(received_packed_t)
                    p_obj->unpack_step = 0;
                    p_obj->index = 0;

                    // 获取校验和
                    check_sum = (p_obj->protocol_packet[sizeof(received_packed_t) - 1] << 8) |
                                p_obj->protocol_packet[sizeof(received_packed_t) - 2];

                    // 验证校验和
                    if (auto_shoot_get_crc16(p_obj->protocol_packet,
                                             sizeof(received_packed_t) - sizeof(check_sum),
                                             CRC_INIT_AUTO) == check_sum) {
                        // 复制至结构体中
                        memcpy(received_packed, p_obj->protocol_packet, sizeof(received_packed_t));
                        // 复制到传入的 received_packed
                    }
                }
            }
            break;

            // 默认情况，重置状态
            default: {
                p_obj->unpack_step = 0;
                p_obj->index = 0;
            }
            break;
        }
    }
}

/**
 * @brief  更新下位机自瞄选板数据，从视觉接收数据结构体搬运
 * @retval 无
 * @note   从坐标轴正向看向原点，逆时针方向为正
 */
void extract_vision_data_to_solver(solver_track_t *solver_track, received_packed_t *received_packed) {
    // 更新目标ID和装甲板数量
    solver_track->armor_id = (armor_id_e) received_packed->id;
    solver_track->armor_num = (armor_num_e) received_packed->armors_num;

    // 更新目标位置信息
    solver_track->xw = received_packed->x;
    solver_track->yw = received_packed->y;
    solver_track->zw = received_packed->z;
    solver_track->tar_yaw = received_packed->yaw;

    // 更新目标速度信息
    solver_track->vxw = received_packed->vx;
    solver_track->vyw = received_packed->vy;
    solver_track->vzw = received_packed->vz;
    solver_track->v_yaw = received_packed->v_yaw;

    // 更新目标装甲板尺寸信息
    solver_track->r1 = received_packed->r1;
    solver_track->r2 = received_packed->r2;
    solver_track->dz = received_packed->dz;
}

/**
 * @brief  根据最优决策得出被击打装甲板 自动解算弹道
 * @param  solver_track: 目标跟踪数据结构体
 * @param  send_packed: 发送数据包结构体，用于存储计算结果
 * @retval 无
 */
void auto_shoot_solve_trajectory(solver_track_t *solver_track, send_packed_t *send_packed) {
    // 从solver_track结构体中获取参数
    float xc = solver_track->xw;
    float yc = solver_track->yw;
    float z = solver_track->zw;
    float armor_yaw = solver_track->tar_yaw;
    float v_yaw = solver_track->v_yaw;
    float r1 = solver_track->r1;
    float r2 = solver_track->r2;

    // 计算装甲板坐标
    float xa = xc - r1 * cosf(armor_yaw);
    float ya = yc - r1 * sinf(armor_yaw);

    // 计算飞行时间
    float distance = sqrtf(xa * xa + ya * ya);
    float horizontal_speed = solver_track->current_v * cosf(solver_track->current_pitch);
    float s_fly_time = distance / horizontal_speed + solver_track->bias_time / 1000.0f;

    // 计算飞行时间后的装甲板位置
    float s_pre_xc = xc + solver_track->vxw * s_fly_time;
    float s_pre_yc = yc + solver_track->vyw * s_fly_time;
    float s_pre_yaw = armor_yaw + solver_track->v_yaw * s_fly_time;

    // 初始化目标坐标变量
    float aim_x = 0.0f;
    float aim_y = 0.0f;
    float aim_z = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    solver_track->fire_step = 0;

    // 处理低角速度情况
    if (fabsf(solver_track->v_yaw) < 1.5f) {
        aim_x = s_pre_xc - r1 * cosf(s_pre_yaw);
        aim_y = s_pre_yc - r1 * sinf(s_pre_yaw);
        aim_z = z;
        solver_track->fire_step = 2;
    } else {
        // 高角速度情况下选择最优装甲板
        uint8_t armor_num = solver_track->armor_num;
        bool_t is_current_pair = 1;
        float r = 0;
        float center_yaw = atan2f(s_pre_yc, s_pre_xc);

        for (uint8_t i = 0; i < armor_num; i++) {
            float tmp_yaw = s_pre_yaw + (fp32) i * (2 * PI / (fp32) armor_num);
            float yaw_diff = loop_fp32_constrain(center_yaw - tmp_yaw, -PI, PI);
            float yaw_diff_offset = signbit(v_yaw) ? -ARMOR_YAW_LIMIT_OFFSET : ARMOR_YAW_LIMIT_OFFSET;

            if (-ARMOR_YAW_LIMIT + yaw_diff_offset < yaw_diff &&
                yaw_diff < ARMOR_YAW_LIMIT + yaw_diff_offset) {
                // 根据装甲板数量确定半径和高度
                if (armor_num == 4) {
                    r = is_current_pair ? r1 : r2;
                    aim_z = z + (is_current_pair ? 0 : solver_track->dz);
                } else {
                    r = r1;
                    aim_z = z;
                }

                // 检查装甲板到坐标原点的距离是否小于机器人中心到原点的距离
                float armor_x = s_pre_xc - r * cosf(tmp_yaw);
                float armor_y = s_pre_yc - r * sinf(tmp_yaw);
                float armor_dist_squared = armor_x * armor_x + armor_y * armor_y;
                float center_dist_squared = s_pre_xc * s_pre_xc + s_pre_yc * s_pre_yc;

                if (armor_dist_squared < center_dist_squared) {
                    aim_x = armor_x;
                    aim_y = armor_y;
                    solver_track->fire_step = 1;
                    break;
                }
            }
            is_current_pair = !is_current_pair;
        }
    }

    // 处理瞄准和发射控制
    if (solver_track->fire_step != 0) {
        float temp_pitch = 0.0f;

        // 解算弹道，注释掉的代码
        // solver(solver_track->current_v, solver_track->k,
        //        sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - solver_track->s_bias,
        //        aim_z, &temp_pitch);

        if (temp_pitch) {
            pitch = -temp_pitch;
        }

        if (aim_x != 0.0f || aim_y != 0.0f) {
            yaw = atan2f(aim_y, aim_x);
        }

        float s_allow_error_distance;
        // 发射控制逻辑
        if (solver_track->fire_step == 1) {
            // 根据装甲板类型确定误差容忍度
            if (solver_track->armor_num == 2 || solver_track->armor_id == ARMOR_HERO) {
                s_allow_error_distance = 0.04f;
            } else {
                s_allow_error_distance = 0.01f;
            }

            if (solver_track->armor_id == ARMOR_OUTPOST) {
                s_allow_error_distance = 0.01f;
            }

            float allow_error_angle = s_allow_error_distance / distance;

            if (fabsf(loop_fp32_constrain(solver_track->current_yaw - yaw, -PI, PI)) < allow_error_angle) {
                solver_track->fire_step = 2;
            }
        }
    }

    solver_track->target_pitch = pitch;
    solver_track->tar_yaw = yaw;
    send_packed->aim_x = aim_x;
    send_packed->aim_y = aim_y;
    send_packed->aim_z = aim_z;
}

/**
 * @brief  向上位机发送数据
 * @retval 无
 */
void autoshoot_prepare_send_data(const received_packed_t *received_packed, send_packed_t *send_packed,
                                 solver_track_t *solver_track) {
    // 设置帧头
    send_packed->header = AUTO_SHOOT_HEADER_SOF;

    // 获取机器人ID
    uint8_t robot_id = get_robot_id();

    // 获取当前机器人颜色
    if (robot_id <= RED_SENTRY) {
        //红色
        send_packed->detect_color = ROBOT_TEAM_BLUE;
    } else if (robot_id >= BLUE_HERO) {
        send_packed->detect_color = ROBOT_TEAM_RED;
    }

    // 获取云台角度数据
    send_packed->roll = get_INS_angle_point()[0];
    send_packed->pitch = get_INS_angle_point()[1];
    send_packed->yaw = get_INS_angle_point()[2];

    // 如果接收到追踪数据
    if (received_packed->tracking == 1) {
        // 根据最优决策得出被击打装甲板 自动解算弹道
        auto_shoot_solve_trajectory(solver_track, send_packed);
    }

    //获取校验码
    send_packed->checksum = auto_shoot_get_crc16(
        (uint8_t *) send_packed,
        sizeof(send_packed_t) - 2, CRC_INIT_AUTO);
}

uint8_t CDC_send_message(uint8_t *Buf, uint16_t Len) {
    return CDC_Transmit_HS(Buf, Len);
}

static auto_shoot_t auto_shoot = {
    .send_message = CDC_send_message,
};
