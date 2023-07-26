/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "gimbal_task.h"
typedef enum
{
	CLASSIC =0,
	WIND = 1,
}vision_mode_e;
typedef __packed struct
{
	uint8_t SOF;
	uint8_t target_found;
	fp32 pitch_angle;
	fp32 yaw_angle;
	uint8_t checksum;
}vision_rx_t;
typedef  struct
{
	uint8_t SOF;
	const fp32 *INS_quat_vision;
	vision_mode_e vision_mode;
	gimbal_control_t *vision_gimbal;
	uint8_t shoot_remote;
	uint8_t enery_color;
	ext_game_robot_state_t *robot_state_v;
	fp32 bullet_speed;
}vision_t;
union refree_4_byte_t
{
		float f;
		unsigned char buf[4];
};
extern void usb_task(void const * argument);

#endif
