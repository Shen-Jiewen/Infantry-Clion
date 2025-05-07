#include "cmsis_os.h"

_Noreturn void watch_task(__attribute__((unused)) void* argument){

	// 喂狗任务,间隔20ms
	while (1){
		// 延时
		osDelay(200);
	}
}