#ifndef UI_DATA_H
#define UI_DATA_H

#include "main.h"

typedef struct {
    uint8_t fric_flag; //摩擦轮标志
    uint8_t cap_remain_energy; //电容电量
    uint8_t gyro_flag; //小陀螺标志
    float auto_pitch; //自瞄Pitch与yaw轴数据
    float auto_yaw;
    float pitch;
    float yaw;
    float center_pitch;
    float center_yaw;
    float distance; //视觉读回的距离
    uint8_t auto_mode;
    uint16_t remain_bullet; //允许发弹量
    uint8_t tracking_ID; //追踪到的车的ID
    uint8_t track_flag; //追踪标志
    uint8_t last_track_flag;
    uint8_t UI_trackID_bit;
    float UI_pitch;
} UI_data_t;

#endif //UI_DATA_H
