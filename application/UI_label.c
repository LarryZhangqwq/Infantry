//Miao 
/*使用说明：
 * 标签控件用于显示文字类信息。
 * 使用 UI_label_static 初始化
 * 初始化生成文字基本格式
 * 使用 UI_label_change 更新状态
 * 状态来源为“视觉”数据输入 对数据进行判断后 将储存于结构体 UI_label_data
*/

#include "UI_label.h"
#include "User_Task.h"
#include "main.h"
#include "math.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"
#include "cmsis_os.h"
#include "string.h"
#include <stdio.h>
#include "detect_task.h"
#include "RM_Cilent_UI.h"
#include "usbd_cdc_if.h"
#include "gimbal_task.h"
#include "vision.h"
#include "servo_task.h"
#include "usb_task.h"
#include "remote_control.h"

UI_show_t ui;

Graph_Data line_30;
Graph_Data line_31;
Graph_Data line_32;
Graph_Data line_33;

String_Data image_1, image_2, image_3, image_4, image_5, image_6; // 这是信息表的提示栏
String_Data vision_0, vision_1, vision_4;  // 这是信息表的数据栏

uint8_t ui_trigger_state = UINT8_MAX;
uint8_t ui_spin_state = UINT8_MAX;
uint8_t ui_fric_state = UINT8_MAX;
uint8_t ui_cover_state = UINT8_MAX;
uint8_t ui_vision_state = UINT8_MAX;
fp32 ui_pitch_state = UINT8_MAX;

extern ext_game_robot_state_t robot_state;

void label_draw(uint8_t optional) {

    //更新数据
    //ui.ui_vision = get_vision_data();
    ui.ui_chassis_move = get_chassis_point();

    ui.ui_gimbal_control = get_gimbal_point();
    ui.ui_shoot_control = get_shoot_point();
    ui.ui_robot_hurt = get_hurt_point();
    ui.ui_robot_status = get_robot_status_point();

    uint8_t temp;
    
    //  !Form: SUPERCAP WARNING
    if (! switch_is_down(get_remote_control_point()->rc.s[0]) && robot_state.remain_HP == 0) {
        String_Data vision_9;
        memset(&vision_9, 0, sizeof(vision_9));
        Char_Draw(&vision_9, "207", optional, 1, UI_Color_Black, 64, 17, 16, 448, 808, "PLEASE ZERO FORCE");
        Char_ReFresh(vision_9);
    } else {
        String_Data vision_9;
        memset(&vision_9, 0, sizeof(vision_9));
        Char_Draw(&vision_9, "207", UI_Graph_Del, 1, UI_Color_Black, 64, 0, 16, 448, 808, "");
        Char_ReFresh(vision_9);
    }
    

    //  !Form: 拨弹轮
    String_Data vision_5;
    temp = get_trigger_state();
    if (temp != ui_trigger_state) {
        ui_trigger_state = temp;
        switch (ui_trigger_state) {
            case 0:  // OFF
                memset(&vision_5, 0, sizeof(vision_5));
                Char_Draw(&vision_5, "203", optional, 1, UI_Color_Yellow, 15, 3, 4, 220, 860, "OFF");
                Char_ReFresh(vision_5);
                break;
            case 1:  // ON
                memset(&vision_5, 0, sizeof(vision_5));
                Char_Draw(&vision_5, "203", optional, 1, UI_Color_Green, 15, 2, 4, 220, 860, "ON");
                Char_ReFresh(vision_5);
                break;
            case 3:  // 离线
                memset(&vision_5, 0, sizeof(vision_5));
                Char_Draw(&vision_5, "203", optional, 1, UI_Color_Yellow, 15, 8, 4, 220, 860, "off-line");
                Char_ReFresh(vision_5);
                break;
            default:  // 理论上不会遇到
                break;
        }
    }

    // !Form: 摩擦轮
    String_Data vision_6;
    temp = get_fric_state();
    if (temp != ui_fric_state) {
        ui_fric_state = temp;
        switch (ui_fric_state) {
            case 0:  // OFF
                memset(&vision_6, 0, sizeof(vision_6));
                Char_Draw(&vision_6, "204", optional, 1, UI_Color_Yellow, 15, 3, 4, 220, 830, "OFF");
                Char_ReFresh(vision_6);
                break;
            case 1:  // ON
                memset(&vision_6, 0, sizeof(vision_6));
                Char_Draw(&vision_6, "204", optional, 1, UI_Color_Green, 15, 2, 4, 220, 830, "ON");
                Char_ReFresh(vision_6);
                break;
            case 3:  // 离线
                memset(&vision_6, 0, sizeof(vision_6));
                Char_Draw(&vision_6, "204", optional, 1, UI_Color_Yellow, 15, 8, 4, 220, 830, "off-line");
                Char_ReFresh(vision_6);
                break;
            default:  // 理论上不会遇到
                break;
        }
    }

    //  !Form: Pitch轴数据
    fp32 pitch_angle = ui.ui_gimbal_control->gimbal_pitch_motor.absolute_angle;
    if (!toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) {
        if (fabsf(pitch_angle - ui_pitch_state) >= 0.01) {
            ui_pitch_state = pitch_angle;
            char pitch_angle_value[12];
            String_Data CH_PITCH_DATA;
            memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
            sprintf(pitch_angle_value, "%.2f", pitch_angle);
            Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Yellow, 15, 6, 4, 220, 800, &pitch_angle_value[0]);
            Char_ReFresh(CH_PITCH_DATA);
        }
    } else {
        String_Data CH_PITCH_DATA;
        memset(&CH_PITCH_DATA, 0, sizeof(CH_PITCH_DATA));
        Char_Draw(&CH_PITCH_DATA, "022", optional, 8, UI_Color_Yellow, 15, 5, 4, 220, 800, "ERROR");
        Char_ReFresh(CH_PITCH_DATA);
    }

    // !Form: 小陀螺状态
    // FIXME: 只有键盘事件才会响应，待更新
    temp = ui.ui_chassis_move->swing_flag;
    if (temp != ui_spin_state) {
        ui_spin_state = temp;
        if (ui_spin_state == 1) {  // 小陀螺启动
            String_Data vision_2;
            memset(&vision_2, 0, sizeof(vision_2));
            Char_Draw(&vision_2, "201", optional, 1, UI_Color_Green, 15, 2, 4, 220, 770, "ON");
            Char_ReFresh(vision_2);
        } else {
            String_Data vision_3;
            memset(&vision_3, 0, sizeof(vision_3));
            Char_Draw(&vision_3, "201", optional, 1, UI_Color_Yellow, 15, 3, 4, 220, 770, "OFF");
            Char_ReFresh(vision_3);
        }
    }

    //  !Form:  弹舱盖数据判断
    temp = get_cover_state();
    if (temp != ui_cover_state) {
        ui_cover_state = temp;
        if (ui_cover_state == 1) {  // 弹仓关
            memset(&vision_4, 0, sizeof(vision_4));
            Char_Draw(&vision_4, "202", optional, 1, UI_Color_Yellow, 15, 3, 4, 220, 740, "OFF");
            Char_ReFresh(vision_4);
        } else {  // 弹舱开
            memset(&vision_4, 0, sizeof(vision_4));
            Char_Draw(&vision_4, "202", optional, 1, UI_Color_Green, 15, 4, 4, 220, 740, "ON");
            Char_ReFresh(vision_4);
        }
    }

    //  !Form: 视觉数据判断
    temp = get_vision_state();
    if (temp != ui_vision_state) {
        ui_vision_state = temp;
        if (ui_vision_state == 0) {  // 掉线  
            memset(&vision_0, 0, sizeof(vision_0));
            Char_Draw(&vision_0, "200", optional, 1, UI_Color_Pink, 15, 7, 4, 220, 710, "Human");
            Char_ReFresh(vision_0);
					
					  memset(&line_30, 0, sizeof(line_30));
						Line_Draw(&line_30, "902", optional, 3, UI_Color_Pink, 2, 640, 720, 640, 360);

						memset(&line_31, 0, sizeof(line_31));
						Line_Draw(&line_31, "903", optional, 3, UI_Color_Pink, 2, 640, 360, 1280, 360);

						memset(&line_32, 0, sizeof(line_32));
						Line_Draw(&line_32, "904", optional, 3, UI_Color_Pink, 2, 1280, 360, 1280, 720);

						memset(&line_33, 0, sizeof(line_33));
						Line_Draw(&line_33, "905", optional, 3, UI_Color_Pink, 2, 1280, 720, 640, 720);
					
					  UI_ReFresh(1, line_30, line_31);
				  	UI_ReFresh(1, line_32, line_33);
				}else if (ui_vision_state == 3){  //未识别到
						memset(&vision_0, 0, sizeof(vision_0));
            Char_Draw(&vision_0, "200", optional, 1, UI_Color_Yellow, 15, 8, 4, 220, 710, "NoTarget");
            Char_ReFresh(vision_0);
					
					  					  memset(&line_30, 0, sizeof(line_30));
						Line_Draw(&line_30, "902", optional, 3, UI_Color_Yellow, 2, 640, 720, 640, 360);

						memset(&line_31, 0, sizeof(line_31));
						Line_Draw(&line_31, "903", optional, 3, UI_Color_Yellow, 2, 640, 360, 1280, 360);

						memset(&line_32, 0, sizeof(line_32));
						Line_Draw(&line_32, "904", optional, 3, UI_Color_Yellow, 2, 1280, 360, 1280, 720);

						memset(&line_33, 0, sizeof(line_33));
						Line_Draw(&line_33, "905", optional, 3, UI_Color_Yellow, 2, 1280, 720, 640, 720);
					
					  UI_ReFresh(1, line_30, line_31);
				  	UI_ReFresh(1, line_32, line_33);
        } else if (ui_vision_state == 1) {  // 识别到
            memset(&vision_1, 0, sizeof(vision_1));
            Char_Draw(&vision_1, "200", optional, 1, UI_Color_Cyan, 15, 5, 4, 220, 710, "Found");
            Char_ReFresh(vision_1);
					
					  					  memset(&line_30, 0, sizeof(line_30));
						Line_Draw(&line_30, "902", optional, 3, UI_Color_Cyan, 2, 640, 720, 640, 360);

						memset(&line_31, 0, sizeof(line_31));
						Line_Draw(&line_31, "903", optional, 3, UI_Color_Cyan, 2, 640, 360, 1280, 360);

						memset(&line_32, 0, sizeof(line_32));
						Line_Draw(&line_32, "904", optional, 3, UI_Color_Cyan, 2, 1280, 360, 1280, 720);

						memset(&line_33, 0, sizeof(line_33));
						Line_Draw(&line_33, "905", optional, 3, UI_Color_Cyan, 2, 1280, 720, 640, 720);
					
					  UI_ReFresh(1, line_30, line_31);
				  	UI_ReFresh(1, line_32, line_33);
        } else {  // ui_vision_state == 2, 识别到并在跟踪
            memset(&vision_4, 0, sizeof(vision_4));
            Char_Draw(&vision_4, "200", optional, 1, UI_Color_Green, 15, 8, 4, 220, 710, "Followed");
            Char_ReFresh(vision_4);
					
					  memset(&line_30, 0, sizeof(line_30));
						Line_Draw(&line_30, "902", optional, 3, UI_Color_Green, 2, 640, 720, 640, 360);

						memset(&line_31, 0, sizeof(line_31));
						Line_Draw(&line_31, "903", optional, 3, UI_Color_Green, 2, 640, 360, 1280, 360);

						memset(&line_32, 0, sizeof(line_32));
						Line_Draw(&line_32, "904", optional, 3, UI_Color_Green, 2, 1280, 360, 1280, 720);

						memset(&line_33, 0, sizeof(line_33));
						Line_Draw(&line_33, "905", optional, 3, UI_Color_Green, 2, 1280, 720, 640, 720);
					
					  UI_ReFresh(1, line_30, line_31);
				  	UI_ReFresh(1, line_32, line_33);
        }
    }
}


//初始化
void UI_label_static() {
    memset(&image_3, 0, sizeof(image_3));
    Char_Draw(&image_3, "103", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 860, "TRIGGER");  // 拨弹轮
    Char_ReFresh(image_3);

    memset(&image_4, 0, sizeof(image_4));
    Char_Draw(&image_4, "104", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 830, "FIRC");  // 摩擦轮
    Char_ReFresh(image_4);

    memset(&image_5, 0, sizeof(image_5));
    Char_Draw(&image_5, "105", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 800, "PITCH");
    Char_ReFresh(image_5);

    memset(&image_6, 0, sizeof(image_6));
    Char_Draw(&image_6, "106", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 4, 4, 60, 770, "SPIN");  // 小陀螺
    Char_ReFresh(image_6);

    memset(&image_1, 0, sizeof(image_1));
    Char_Draw(&image_1, "101", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 710, "VISION");
    Char_ReFresh(image_1);

    memset(&image_2, 0, sizeof(image_2));
    Char_Draw(&image_2, "102", UI_Graph_ADD, 1, UI_Color_Cyan, 15, 8, 4, 60, 740, "COVER");
    Char_ReFresh(image_2);

    UI_label_reset_cache();

    label_draw(UI_Graph_ADD);
}


void UI_label_reset_cache() {
    ui_trigger_state = UINT8_MAX;
    ui_spin_state = UINT8_MAX;
    ui_fric_state = UINT8_MAX;
    ui_cover_state = UINT8_MAX;
    ui_vision_state = UINT8_MAX;
    ui_pitch_state = UINT8_MAX;

}


//状态更新
void UI_label_change() {
    label_draw(UI_Graph_Change);
}
