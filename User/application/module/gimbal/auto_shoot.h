#ifndef AUTO_SHOOT_H
#define AUTO_SHOOT_H

#include "main.h"
#include "fifo.h"
#include "struct_typedef.h"


// 常量定义
#define AUTO_SHOOT_FIFO_BUF_LENGTH 1024    // 最大接收的数据长度
#define CRC_INIT_AUTO              0xFFFF  // CRC校验初始值
#define AUTO_SHOOT_HEADER_SOF      0x5A    // 帧头标识
#define ARMOR_YAW_LIMIT            0.6f    // 装甲板偏航角限制
#define ARMOR_YAW_LIMIT_OFFSET     0.22f   // 装甲板偏航角限制偏移量
#define BULLET_SPEED               23.5f   // 弹丸初速度
#define TRIGGER_DELAY              100.f   // 发弹延迟时间
#define BULLET_FORWARD_DISTANCE    0.0f    // 枪口前推距离
#define CAMERA_TO_YAW_AXIS_HEIGHT  0.0f    // 相机距离转轴高度


// 装甲板ID枚举
typedef enum {
    ARMOR_OUTPOST = 0, // 前哨站
    ARMOR_HERO = 1, // 英雄
    ARMOR_ENGINEER = 2, // 工程
    ARMOR_INFANTRY3 = 3, // 步兵3
    ARMOR_INFANTRY4 = 4, // 步兵4
    ARMOR_INFANTRY5 = 5, // 步兵5
    ARMOR_GUARD = 6, // 哨兵
    ARMOR_BASE = 7 // 基地
} armor_id_e;

// 装甲板数量枚举
typedef enum {
    ARMOR_NUM_BALANCE = 2, // 平衡步兵
    ARMOR_NUM_OUTPOST = 3, // 前哨站
    ARMOR_NUM_NORMAL = 4 // 常规
} armor_num_e;

// 子弹类型枚举
typedef enum {
    BULLET_17 = 0, // 17mm弹丸
    BULLET_42 = 1 // 42mm弹丸
} bullet_type_e;

// 机器人队伍枚举
typedef enum {
    ROBOT_TEAM_RED = 0, // 红队
    ROBOT_TEAM_BLUE = 1 // 蓝队
} robot_team_e;

// 解算跟踪结构体
typedef struct {
    // 自身参数
    bullet_type_e bullet_type; // 子弹类型: 0-17mm, 1-42mm
    float current_v; // 当前弹速
    float current_pitch; // 当前俯仰角
    float current_yaw; // 当前偏航角

    // 目标参数
    float xw; // 世界坐标系下的x坐标
    float yw; // 世界坐标系下的y坐标
    float zw; // 世界坐标系下的z坐标
    float tar_yaw; // 目标偏航角

    float vxw; // 世界坐标系下的x方向速度
    float vyw; // 世界坐标系下的y方向速度
    float vzw; // 世界坐标系下的z方向速度
    float v_yaw; // 目标偏航角速度

    float r1; // 目标中心到前后装甲板的距离
    float r2; // 目标中心到左右装甲板的距离
    float dz; // 另一对装甲板相对于被跟踪装甲板的高度差

    float bias_time; // 偏置时间
    float s_bias; // 枪口前推的距离
    float z_bias; // yaw轴电机到枪口水平面的垂直距离

    // 开火控制
    uint8_t fire_step;

    armor_id_e armor_id; // 装甲板类型
    armor_num_e armor_num; // 装甲板数量
} solver_track_t;

// 发送数据包结构体
typedef struct __attribute__((packed)) {
    uint8_t header; // 帧头 0xA5
    robot_team_e detect_color: 1; // 检测颜色: 0-红色, 1-蓝色
    bool_t reset_tracker: 1; // 是否重置跟踪器
    uint8_t reserved: 6; // 保留位

    float roll; // 横滚角
    float pitch; // 俯仰角
    float yaw; // 偏航角

    float aim_x; // 目标x坐标
    float aim_y; // 目标y坐标
    float aim_z; // 目标z坐标

    uint16_t checksum; // CRC校验和
} send_packed_t;

// 接收数据包结构体
typedef struct __attribute__((packed)) {
    uint8_t header; // 帧头 0xA5
    bool_t tracking: 1; // 是否正在追踪目标
    uint8_t id: 3; // 目标ID
    bool_t armors_num: 3; // 装甲板数量
    uint8_t reserved: 1; // 保留位

    float x; // x坐标
    float y; // y坐标
    float z; // z坐标
    float yaw; // 偏航角

    float vx; // x方向速度
    float vy; // y方向速度
    float vz; // z方向速度
    float v_yaw; // 偏航角速度

    float r1; // 目标中心到前后装甲板的距离
    float r2; // 目标中心到左右装甲板的距离
    float dz; // 高度差

    uint16_t checksum; // CRC校验和
} received_packed_t;

// 自瞄数据解包结构体
typedef struct {
    uint16_t data_len; // 数据长度
    uint8_t protocol_packet[sizeof(received_packed_t)]; // 协议数据包
    uint8_t unpack_step; // 解包步骤
    uint16_t index; // 索引
} unpack_autoshoot_data_t;

// 自瞄主结构体
typedef struct {
    solver_track_t solver_track; // 解算跟踪结构体

    fifo_s_t auto_shoot_fifo; // FIFO结构体
    uint8_t auto_shoot_fifo_buf[AUTO_SHOOT_FIFO_BUF_LENGTH]; // FIFO缓冲区
    unpack_autoshoot_data_t unpack_data; // 解包数据结构体

    send_packed_t send_packed; // 发送数据结构体
    received_packed_t received_packed; // 接收数据结构体

    // 通信接口函数指针
    uint8_t (*send_message)(uint8_t *Buf, uint16_t Len);
} auto_shoot_t;

// 外部函数声明
void auto_shoot_init(auto_shoot_t *auto_init);

void auto_shoot_unpack_fifo_data(unpack_autoshoot_data_t *auto_shoot_unpack, fifo_s_t *auto_shoot_fifo,
                                 received_packed_t *received_packed);

void extract_vision_data_to_solver(solver_track_t *solver_track, received_packed_t *received_packed);

uint16_t auto_shoot_get_crc16(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

void auto_shoot_solve_trajectory(solver_track_t *solver_track, send_packed_t *send_packed);

void autoshoot_prepare_send_data(const received_packed_t *received_packed, send_packed_t *send_packed,
                                 solver_track_t *solver_track);

// 获取自瞄实例
auto_shoot_t *auto_shoot_get_instance(void);

#endif // AUTO_SHOOT_H
