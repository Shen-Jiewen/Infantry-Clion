//
// Created by Rick on 2024/12/30.
//

#ifndef TICK_H_
#define TICK_H_

#include "main.h"
#include "bsp_tim.h"

// 用户配置定时器的频率
#define MODULE_TICK_RATE_HZ 100000

void tick_init(void);

#endif //TICK_H_
