/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usbï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï?1?7
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"
#include "remote_control.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "crc8_crc16.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "vision.h"

extern vision_control_t vision_control;

static void usb_printf(const char *fmt, ...);
static void get_INS_quat_data(uint8_t array);
uint8_t usb_txbuf[49];
vision_t vision_data;
extern USBD_HandleTypeDef hUsbDeviceFS;
int32_t usb_count=0;
static void get_buller_speed(uint8_t array);
static void get_HP_data(uint8_t array);
void vision_init();
static void vision_data_update();
static void get_vision_data(void);
static void get_yaw_data(uint8_t array);
static void get_pitch_data(uint8_t array);
vision_rx_t vision_rx;
extern RC_ctrl_t rc_ctrl;
void usb_task(void const *argument)
{
    
    MX_USB_DEVICE_Init();
		vision_init();
    osDelay(1000);
    while (1)
    {
			vision_data_update();
			get_vision_data();
        vTaskDelay(10);
    }
}

uint32_t system_time;
uint16_t a = 20;
static void get_vision_data(void)
{
    usb_txbuf[0] = vision_data.SOF;
		get_INS_quat_data(1);
	  usb_txbuf[17] = vision_control.vision_mode;
		usb_txbuf[18] = 0xFF;
		if(vision_data.robot_state_v->robot_id>100)
		{
			usb_txbuf[19] = 0;
		}
		else
			usb_txbuf[19] = 1;
		get_buller_speed(20);
		get_HP_data(24);
		uint32_t system_time = xTaskGetTickCount();
    usb_txbuf[39] = system_time >> 24;
    usb_txbuf[38] = system_time >> 16;
    usb_txbuf[37] = system_time >> 8;
    usb_txbuf[36] = system_time;
		get_yaw_data(40);
		get_pitch_data(44);
		append_CRC8_check_sum(usb_txbuf,sizeof(usb_txbuf));
    CDC_Transmit_FS(usb_txbuf, sizeof(usb_txbuf));
}
void vision_init()
{
	vision_data.INS_quat_vision = get_INS_quat_point();
	vision_data.robot_state_v = get_robot_status_point();
	vision_data.vision_gimbal = get_gimbal_point();
	vision_data.SOF = 0XA5;
	vision_data.vision_mode = CLASSIC;
}
static void get_INS_quat_data(uint8_t array)
{
    union refree_4_byte_t INS_data[4];
    INS_data[0].f = *vision_data.INS_quat_vision;
    INS_data[1].f = *(vision_data.INS_quat_vision + 1);
    INS_data[2].f = *(vision_data.INS_quat_vision + 2);
		INS_data[3].f = *(vision_data.INS_quat_vision + 3);
	
    usb_txbuf[array] = INS_data[0].buf[0];
    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8] = INS_data[2].buf[0];
    usb_txbuf[array + 9] = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];

		usb_txbuf[array + 12] = INS_data[3].buf[0];
    usb_txbuf[array + 13] = INS_data[3].buf[1];
    usb_txbuf[array + 14] = INS_data[3].buf[2];
    usb_txbuf[array + 15] = INS_data[3].buf[3];
}

static void get_pitch_data(uint8_t array)
{
    union refree_4_byte_t pitch_data;
    pitch_data.f = vision_data.vision_gimbal->gimbal_pitch_motor.absolute_angle;
	
    usb_txbuf[array] = pitch_data.buf[0];
    usb_txbuf[array + 1] = pitch_data.buf[1];
    usb_txbuf[array + 2] = pitch_data.buf[2];
    usb_txbuf[array + 3] = pitch_data.buf[3];
}

static void get_yaw_data(uint8_t array)
{
    union refree_4_byte_t yaw_data;
    yaw_data.f = vision_data.vision_gimbal->gimbal_yaw_motor.absolute_angle;
	
    usb_txbuf[array] = yaw_data.buf[0];
    usb_txbuf[array + 1] = yaw_data.buf[1];
    usb_txbuf[array + 2] = yaw_data.buf[2];
    usb_txbuf[array + 3] = yaw_data.buf[3];
}
static void vision_data_update()
{
	vision_data.bullet_speed = get_bullet_speed();
}
static void get_HP_data(uint8_t array)
{
	for(int i = 0;i<=11;i++)
	{
		usb_txbuf[array+1] = 0;
	}
}
static void get_buller_speed(uint8_t array)
{
	  union refree_4_byte_t INS_data;
    INS_data.f = vision_data.bullet_speed;
	
    usb_txbuf[array] = INS_data.buf[0];
    usb_txbuf[array + 1] = INS_data.buf[1];
    usb_txbuf[array + 2] = INS_data.buf[2];
    usb_txbuf[array + 3] = INS_data.buf[3];
}