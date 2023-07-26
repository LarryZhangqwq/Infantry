/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
extern ext_game_robot_state_t robot_state;
#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
#include "vision.h"
static void shoot_bullet_control(void);
extern vision_control_t vision_control;
shoot_control_t shoot_control; //射击数据
fp32 trigger_speed = 0;
uint16_t stop_time=299;
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid[3] = {FRIC_ANGLE_PID_KP, FRIC_ANGLE_PID_KI, FRIC_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    shoot_control.shoot_state = get_robot_status_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric_motor_measure[0] = get_fric_motor_measure_point(1);
    shoot_control.fric_motor_measure[1] = get_fric_motor_measure_point(2);
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_pid[0], PID_POSITION, Fric_speed_pid, 16000, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_pid[1], PID_POSITION, Fric_speed_pid, 16000, FRIC_PID_MAX_IOUT);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_15, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_15, FRIC_OFF);
    shoot_control.fric_can1 = FRIC_OFF;
    shoot_control.fric_can2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current[0] = 0;
    shoot_control.given_current[1] = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.dual_mode=RIGHT;
    shoot_control.trigger_mode = LOW_SHOOT;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t *shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
    if(shoot_control.trigger_mode == LOW_SHOOT)
    {
        trigger_speed = 5;
    }
    else
    {
        trigger_speed = 15;
    }
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = trigger_speed;
        trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current[0] = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        shoot_control.fric1_ramp.out = 0;
        shoot_control.fric2_ramp.out = 0;
    }
    else
    {
        shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current[0] = (int16_t)(shoot_control.trigger_motor_pid.out);
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_CAN_ADD_VALUE);
    }
    if(shoot_control.dual_mode==LEFT)
    {
        shoot_control.given_current[1]=2100-(stop_time-300)*5.5;
    }
    else if(shoot_control.dual_mode==RIGHT)
    {
        shoot_control.given_current[1]=-2100+(stop_time-300)*5.5;
    }
    shoot_control.fric_can1 = (int16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_can2 = (int16_t)(shoot_control.fric2_ramp.out);
    PID_calc(&shoot_control.fric_motor_pid[0], shoot_control.fric_motor_measure[0]->speed_rpm, -shoot_control.fric_can1);
    PID_calc(&shoot_control.fric_motor_pid[1], shoot_control.fric_motor_measure[1]->speed_rpm, shoot_control.fric_can1);
    CAN_CMD_FRIC((int16_t)shoot_control.fric_motor_pid[0].out, (int16_t)shoot_control.fric_motor_pid[1].out);
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{

    static int8_t last_s = RC_SW_UP;
    static uint8_t fric_state = 0;
    static uint16_t press_time = 0;
    uint8_t threshthold = 20;
    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD && !(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL)) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
        //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD && (shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_CTRL)) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    if ((switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_mid(last_s) && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
		{
			press_time = 0;
		}
    if (shoot_control.shoot_mode == SHOOT_READY)
    {
				if(switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s))
				{
					shoot_control.shoot_mode = SHOOT_BULLET;
				}
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])))
        {
						press_time++;
						if(press_time>1500)
						{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
							press_time = 200;
						}
        }

    }


    if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C)
    {
        stop_time=650;
    }
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_mode == SHOOT_READY||shoot_control.shoot_mode == SHOOT_BULLET))
    {

        if ((shoot_control.press_l && shoot_control.last_press_l == 0) )
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }

        if(shoot_control.press_l_time==PRESS_LONG_TIME)
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
    }
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        if (!shoot_control.press_l)
        {
            if (shoot_control.shoot_mode != SHOOT_BULLET)
            {
                shoot_control.shoot_mode = SHOOT_READY;
            }
        }
    }
    if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)
    {
        shoot_control.trigger_mode = HIGH_SHOOT;
    }

    if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_B)
    {
        shoot_control.trigger_mode = LOW_SHOOT;
    }
    get_shoot_heat1_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    get_shoot_heat2_limit_and_heat1(&shoot_control.heat_limit2,&shoot_control.heat2);
    if(stop_time)
    {
        stop_time--;
    }
    if (!toe_is_error(REFEREE_TOE))
    {
        if(robot_state.shooter_id2_17mm_cooling_rate == 0||robot_state.shooter_id2_17mm_cooling_rate == 10)
        {
            if ((shoot_control.heat + 40 >= shoot_control.heat_limit) && (!(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)))
            {
								if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET||shoot_control.shoot_mode == SHOOT_BULLET)
								{
									shoot_control.shoot_mode = SHOOT_READY;
								}
            }
        }
        else
        {
            if(shoot_control.trigger_mode==HIGH_SHOOT)
            {
                threshthold = 40;
            }
            else
            {
                threshthold = 20;
            }

            if(shoot_control.dual_mode==LEFT &&shoot_control.heat + threshthold>= shoot_control.heat_limit)
            {
                stop_time=650;
            }
            else if(shoot_control.dual_mode==RIGHT &&shoot_control.heat2 + threshthold >= shoot_control.heat_limit2)
            {
                stop_time=650;
            }
        }
    }
    if (stop_time <= 650 && stop_time >=350) shoot_control.shoot_mode = SHOOT_READY;
    if(stop_time == 600)
    {
        shoot_control.dual_mode = (shoot_control.dual_mode+1)%2;
    }
    static shoot_mode_e flag=SHOOT_STOP;
		static uint8_t stop_time = 0;
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
     {
				stop_time++;  
     }
		 else
		 {
			 stop_time=0;
		 }
		 
		 if(stop_time > 200)
		 {
			 stop_time = 201;
			 shoot_control.shoot_mode = SHOOT_STOP;
		 }
		 
		 
    if(stop_time&&(shoot_control.shoot_mode==SHOOT_CONTINUE_BULLET||shoot_control.shoot_mode==SHOOT_BULLET))
    {
        shoot_control.shoot_mode = SHOOT_READY;
        flag=SHOOT_CONTINUE_BULLET;
    }
    if(flag&&stop_time==0)
    {
        shoot_control.shoot_mode = flag;
        flag = 0;
    }
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
fp32 fric_30 = FRIC_30;
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
		if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.shoot_motor_measure->ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    if (!toe_is_error(REFEREE_TOE))
    {
        if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 15)
        {
            shoot_control.fric1_ramp.max_value = FRIC_15;
            shoot_control.fric2_ramp.max_value = FRIC_15;
        }
        else if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 18)
        {
            shoot_control.fric1_ramp.max_value = FRIC_18;
            shoot_control.fric2_ramp.max_value = FRIC_18;
        }
        else if (shoot_control.shoot_state->shooter_id1_17mm_speed_limit == 30)
        {
            shoot_control.fric1_ramp.max_value = fric_30;
            shoot_control.fric2_ramp.max_value = fric_30;
        }
    }
    else
    {
        shoot_control.fric1_ramp.max_value = FRIC_15;
        shoot_control.fric2_ramp.max_value = FRIC_15;
    }
}

static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.set_angle + 2 * PI_EIGHT);
        shoot_control.move_flag = 1;
    }
    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.04)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = 12;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY;
    }
}
uint8_t get_trigger_state()
{
    if(toe_is_error(TRIGGER_MOTOR_TOE))
    {
        return 3; //离线
    }
    if(shoot_control.speed<3)
    {
        return 0;//OFF
    }
    else
    {
        return 1;//ON
    }
}
uint8_t get_fric_state()
{
    if(toe_is_error(FRIC_RIGHT_MOTOR_TOE)||toe_is_error(FRIC_LEFT_MOTOR_TOE))
    {
        return 3; //离线
    }
    if(shoot_control.fric_motor_measure[0]->speed_rpm<-1000&&shoot_control.fric_motor_measure[1]->speed_rpm>1000)
    {
        return 1;//ON
    }
    else
    {
        return 0;//ON
    }
}
shoot_control_t *get_shoot_point(void)
{
    return &shoot_control;
}

uint8_t get_shooter_mode() {
    return shoot_control.dual_mode;
}
