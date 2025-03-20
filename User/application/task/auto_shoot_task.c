//
// Created by Rick on 2025/3/20.
//


#include "main.h"
#include "auto_shoot.h"
#include "cmsis_os2.h"

auto_shoot_t *auto_shoot;

_Noreturn void auto_shoot_task(__attribute__((unused)) void *argument) {
    osDelay(1000);
    auto_shoot = auto_shoot_get_instance();
    auto_shoot_init(auto_shoot);

    while (1) {
        // 将FIFO中的数据解包到接收数据结构体中
        auto_shoot_unpack_fifo_data(&auto_shoot->unpack_data, &auto_shoot->auto_shoot_fifo,
                                    &auto_shoot->received_packed);
        // 更新下位机自瞄选板数据，从视觉接收数据结构体搬运
        extract_vision_data_to_solver(&auto_shoot->solver_track, &auto_shoot->received_packed);
        // 计算向上位机发送的预测数据
        autoshoot_prepare_send_data(&auto_shoot->received_packed, &auto_shoot->send_packed,
                                    &auto_shoot->solver_track);
    }
}
