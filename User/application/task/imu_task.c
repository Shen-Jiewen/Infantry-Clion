#include "FreeRTOS.h"
#include "semphr.h"
#include "imu.h"
#include "ist8310driver.h"
#include "dm_imu_l1.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "bsp_tim.h"
#include "tim.h"

static void imu_callback();

imu_control_t *imu_control;

_Noreturn void imu_task(__attribute__((unused)) void *argument) {
	imu_control = get_imu_control_point();
	// imu_control_init(imu_control);
	// imu_hardware_init();

	// 溢出时间为10微秒
	BSP_TIM_InitTypeDef tim_config = {
		.htim = &htim17,
		.period = 300,
		.callback = imu_callback
	};
	BSP_TIM_Init(&tim_config);

	while (1) {
		// if (xSemaphoreTake(imu_control->xAccSemaphore, portMAX_DELAY) == pdTRUE &&
		// 	xSemaphoreTake(imu_control->xGyroSemaphore, portMAX_DELAY) == pdTRUE) {
		// 	// imu_data_update(imu_control);
		// 	imu_statistics_update(imu_control);
		// 	imu_temperature_control(imu_control);
		// }
		dm_imu_data_update(imu_control, get_dm_imu_point(0));
		osDelay(1);
	}
}

void imu_callback(void) {
	static uint32_t dm_imu_get_step = 0;
	switch (dm_imu_get_step) {
		case 0:
			DM_imu_l1_get_angle(&hfdcan3, 0x01);
			break;
		case 1:
			DM_imu_l1_get_gyro(&hfdcan3, 0x01);
			break;
		case 2:
			DM_imu_l1_get_accel(&hfdcan3, 0x01);
			break;
		default:
			break;
	}
	dm_imu_get_step = (dm_imu_get_step + 1) % 3;
}

