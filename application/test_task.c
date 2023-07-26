/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.��������������
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

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;



/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        error = 0;

        //find error
        //���ִ���
        for(error_num = 0; error_num < VISION - 1; error_num++)
        {
            if(error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        //no error, stop buzzer
        //û�д���, ֹͣ������
				
				/*
				if(error_list_test_local[TRIGGER_MOTOR_TOE].error_exist)
				{
						error_num = 2;
					  error = 1;
				}
				if(error_list_test_local[FRIC_RIGHT_MOTOR_TOE].error_exist)
				{
						error_num = 3;
						error = 1;
				}
				if(error_list_test_local[FRIC_LEFT_MOTOR_TOE].error_exist) 
				{
						error_num = 4;
						error = 1;
				}
        if(error == 0 && last_error != 0)
        {
            buzzer_off();
        }
				if(error_list_test_local[PITCH_GIMBAL_MOTOR_TOE].error_exist)
				{
						error_num = 1;
						error = 1;
				}
				if(error_list_test_local[YAW_GIMBAL_MOTOR_TOE].error_exist)
				{
						error_num = 0;
						error = 1;
				}
				if(error_list_test_local[REFEREE_TOE].error_exist)
				{
						error_num = 5;
						error = 1;
				}
				*/
				
        //have error
        //�д���
				// error = 0;
        if(error)
        {
            buzzer_warn_error(error_num+1);
        }

        last_error = error;
        osDelay(10);
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          ʹ�÷�������
  * @param[in]      num:��������
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
		static uint8_t level_flag=0;
    if(show_num == 0 && stop_num == 0)
    {
				if(num<7)
				{
					show_num = num;
					stop_num = 100;
					level_flag=0;
				}
				else
				{
					show_num = num-6;
					stop_num = 100;
					level_flag=1;
				}
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
						if(!level_flag)
						{
							buzzer_on(1, 20000);
						}
						else
						{
							buzzer_on(2, 20000);
						}
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}


