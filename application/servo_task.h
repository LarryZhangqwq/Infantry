#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "struct_typedef.h"


extern void servo_task(void const *argument);

uint8_t get_cover_state();

#endif
