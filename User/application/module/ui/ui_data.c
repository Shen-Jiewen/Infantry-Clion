#include "ui_data.h"
#include "referee.h"
#include "supercap.h"
#include "shoot.h"
#include "auto_shoot.h"
#include "math.h"

static UI_data_t UI_data;

void UI_data_init(UI_data_t *UI_data) {
    UI_data->cap_remain_energy = get_CAPower_measure_point()->cap_energy;
    UI_data->fric_flag = get_shoot_control_point()->shoot_mode >= OPEN_FRIC ? 1 : 0;
    // UI_data.gyro_flag = *gyro_data;

    UI_data->distance = 0;
    UI_data->remain_bullet = get_remain_bullet_num(); //读取允许发弹量
    UI_data->track_flag = auto_shoot_get_instance()->received_packed.tracking; //读取追踪标志
    UI_data->tracking_ID = auto_shoot_get_instance()->received_packed.id; //获取识别到的车的ID
    UI_data->UI_trackID_bit = 1;
}

void UI_data_update(UI_data_t *UI_data) {
    UI_data->remain_bullet = get_remain_bullet_num(); //更新允许发弹量
    UI_data->track_flag = auto_shoot_get_instance()->received_packed.tracking; //读取追踪标志
    UI_data->tracking_ID = auto_shoot_get_instance()->received_packed.id; //获取识别到的车的ID
    UI_data->distance = sqrtf(
        powf(auto_shoot_get_instance()->received_packed.x, 2) + powf(auto_shoot_get_instance()->received_packed.y, 2));

    UI_data->cap_remain_energy = get_CAPower_measure_point()->cap_energy; //读取电容电量
    UI_data->fric_flag = get_shoot_control_point()->shoot_mode >= OPEN_FRIC ? 1 : 0;
    // UI_data->gyro_flag = *gyro_data;
}

UI_data_t *get_UI_data_point(void) {
    return &UI_data;
}
