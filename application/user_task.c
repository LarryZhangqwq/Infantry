/**
  ****************************辽宁科技大学COD****************************
  * @file       user_task.c/h-雨落长安
  * fork order:       GMaster 电控组-UI (Juntong, Miao, Yanxue)
  * @brief      一个普通程序，请大家友善对待，谢谢大家，写的demo，未来得及封装，凑合看吧
  *             我们封装了，注释不多，凑合着看吧
  ==============================================================================
  @endverbatim
  ****************************辽宁科技大学COD****************************
  ****************************G-Master 电控组-UI************************
  */

#include "User_Task.h"
#include "gimbal_task.h"
#include "referee.h"
#include "cmsis_os.h"
#include "string.h"
#include "RM_Cilent_UI.h"
#include "UI_ProgressBar.h"
#include "UI_car.h"
#include "UI_label.h"



extern vision_control_t vision_control;

#include "vision.h"
#include "CAN_receive.h" // 加载摩擦轮发射闭环真实转速

#define PI 3.1415936
#define cbc car.basic_config

#define set_name(d, s1, s2, s3) do{\
                                    (d)[0] = (uint8_t) (s1); \
                                    (d)[1] = (uint8_t) (s2); \
                                    (d)[2] = (uint8_t) (s3); \
                                } while(0)


void UI_send_init();

void UI_car_init();

void UI_car_change();

extern uint16_t Robot_number;
extern uint16_t Robot_cline_number;
extern UI_show_t ui;

Graph_Data line_2;
Graph_Data line_3;
Graph_Data line_4;
Graph_Data line_5;
Graph_Data line_6;
Graph_Data line_7;
Graph_Data line_8;
Graph_Data line_9;
Graph_Data line_10;
Graph_Data line_11;
Graph_Data line_12;
Graph_Data line_20;
String_Data time_data;
uint8_t ui_shooter_speed = 0;

int second = 0;

car_handle car;

progress_bar_data bar;

id_data_t id_data;

void robot_id_data_init(void);

void robot_id_select(void);

void UI_aimline(void);

void UI_car_static(void);



void UserTask(void const *pvParameters) {
    static uint16_t time = 0;
    int16_t batter_percentage = 0;

    memset(&bar, 0, sizeof(bar));
    memset(&car, 0, sizeof(car));

    UI_send_init();
    UI_label_static();
    UI_car_init();
    UI_ProgressBar_static(&bar);
    UI_car_static();

    while (1) {
        batter_percentage = get_cap_percent();
        bar.progress_bar_data_change = batter_percentage;

        // 刷新
        time = (time + 1) % 200;  // 由于刷新延迟函数被删除，重置速度满一些
        if (time == 0) {
            UI_label_static();  // 重新加载数据表格
            UI_ProgressBar_static(&bar);  // 重新加载超级电容显示
            UI_car_static();
        } else if (time % 30 == 0) {
            UI_label_reset_cache();
        }
        if (time % 10 == 0) {
            UI_label_change();
            UI_ProgressBar_change(&bar);
            second += 1;
            ui_shooter_speed = get_shooter_speed_limit(); //获得弹速上限
            UI_aimline();  // 重新绘制瞄准线
        }
        UI_car_change();

        robot_id_select(); // 保证热插拔，每次任务都选择一次ID
        vTaskDelay(1);
    }
}

void UI_car_init() {
    /*** 填充小车数据 ***/
/*中心坐标*/
    cbc.central_x = 1700;
    cbc.central_y = 700;
/*显示配置*/
    cbc.full_radius = 110;
    cbc.body_half_length = 70;
    cbc.body_half_width = 60;
    cbc.rear_half_width = 30;
    cbc.head_radius = 35;

    cbc.drawing_width = 2;
    cbc.normal_colour_code = UI_Color_Yellow;
    cbc.attacked_colour_code = UI_Color_Pink;
    cbc.head_layer = 1;
    cbc.body_layer = 2;

    set_name(cbc.body_name_front, '3', '0', '0');
    set_name(cbc.body_name_back, '3', '0', '1');
    set_name(cbc.body_name_left, '3', '0', '2');
    set_name(cbc.body_name_right, '3', '0', '3');
    set_name(cbc.head_name_line, '3', '0', '4');
    set_name(cbc.head_name_circle, '3', '0', '5');
    set_name(cbc.rear_name_back, '3', '0', '6');
    set_name(cbc.rear_name_left, '3', '0', '7');
    set_name(cbc.rear_name_right, '3', '0', '8');

    car.body_rad = PI;
    car.head_degree = 180;
    car.front_armor_showing_attacked = 0;  // 初始状态是不被击打
    car.back_armor_showing_attacked = 0;
    car.left_armor_showing_attacked = 0;
    car.right_armor_showing_attacked = 0;
}

void UI_car_static() {
    car_init_by_handle(&car);
}

void UI_car_change() {
    car_rotate_body(&car, get_body_rad());  // 经测试是+180
    // 现在前装甲板OK了
    // 前装甲板显示正常
    // 后装甲板显示正常
    // 左装甲板是右装甲板
    // 受击打状态实际与名称对应
//    car_front_armor_showing_attacked(&car, get_front_amour_attacked());
//    car_back_armor_showing_attacked(&car, get_back_amour_attacked());
//    car_left_armor_showing_attacked(&car, get_right_amour_attacked());
//    car_right_armor_showing_attacked(&car, get_left_amour_attacked());
}

void UI_send_init() {

/*** 数据赋值  ***/
    robot_id_data_init();
    robot_id_select();

}

void ui_reset_car_front_armor_attacked_timer() {
    car_reset_front_armor_timer(&car);
}

void ui_reset_car_back_armor_attacked_timer() {
    car_reset_back_armor_timer(&car);
}

void ui_reset_car_left_armor_attacked_timer() {
    car_reset_right_armor_timer(&car);  // 绘制时左右颠倒了
}

void ui_reset_car_right_armor_attacked_timer() {
    car_reset_left_armor_timer(&car);  // 为什么重置了左计时器呢，见“ui_reset_car_front_armor_attacked_timer”的注释
}

void UI_aimline() {
    if (ui_shooter_speed == 15) {
        UI_Delete(UI_Graph_Del, 3);

//        memset(&line_1, 0, sizeof(line_1));
//        Line_Draw(&line_1, "901", UI_Graph_ADD, 3, UI_Color_Yellow, 2, 930, 450, 990, 450);

        memset(&line_2, 0, sizeof(line_2));
        Line_Draw(&line_2, "902", UI_Graph_Change, 3, UI_Color_Yellow, 2, 930, 454, 990, 454);

        memset(&line_5, 0, sizeof(line_5));
        Line_Draw(&line_5, "903", UI_Graph_Change, 3, UI_Color_Pink, 2, 900, 421, 1020, 421);

        memset(&line_8, 0, sizeof(line_8));
        Line_Draw(&line_8, "904", UI_Graph_Change, 3, UI_Color_Pink, 2, 870, 405, 1050, 405);

        memset(&line_11, 0, sizeof(line_11));
        Line_Draw(&line_11, "905", UI_Graph_Del, 3, UI_Color_Green, 2, 840, 462, 1080, 462);

        memset(&line_12, 0, sizeof(line_12));
        Line_Draw(&line_12, "906", UI_Graph_Del, 3, UI_Color_Green, 2, 840, 442, 1080, 442);
			
				memset(&line_20, 0, sizeof(line_20));
        Line_Draw(&line_20, "905", UI_Graph_Del, 3, UI_Color_Yellow, 2, 960, 300, 960, 600);

        UI_ReFresh(5, line_2, line_5, line_8, line_11, line_12);
				UI_ReFresh(1, line_20);
    } else if (ui_shooter_speed == 18) {
        UI_Delete(UI_Graph_Del, 3);

//						memset(&line_1, 0, sizeof(line_1));
//						Line_Draw(&line_1, "901", UI_Graph_ADD, 3, UI_Color_Yellow, 2, 930, 450, 990, 450);

        memset(&line_3, 0, sizeof(line_3));
        Line_Draw(&line_3, "902", UI_Graph_Del, 3, UI_Color_Orange, 2, 930, 467, 990, 467);

        memset(&line_6, 0, sizeof(line_6));
        Line_Draw(&line_6, "903", UI_Graph_Change, 3, UI_Color_Pink, 2, 900, 448, 1020, 448);

        memset(&line_9, 0, sizeof(line_9));
        Line_Draw(&line_9, "904", UI_Graph_Change, 3, UI_Color_Pink, 2, 870, 432, 1050, 432);

        memset(&line_11, 0, sizeof(line_11));
        Line_Draw(&line_11, "905", UI_Graph_Del, 3, UI_Color_Green, 2, 840, 462, 1080, 462);

        memset(&line_12, 0, sizeof(line_12));
        Line_Draw(&line_12, "906", UI_Graph_Del, 3, UI_Color_Green, 2, 840, 442, 1080, 442);
			
			  memset(&line_20, 0, sizeof(line_20));
        Line_Draw(&line_20, "905", UI_Graph_Del, 3, UI_Color_Yellow, 2, 960, 300, 960, 600);

        UI_ReFresh(5, line_3, line_6, line_9, line_11, line_12);
				UI_ReFresh(1, line_20);
    } else {
        UI_Delete(UI_Graph_Del, 3);

//			      memset(&line_1, 0, sizeof(line_1));
//						Line_Draw(&line_1, "901", UI_Graph_Del, 3, UI_Color_Yellow, 2, 930, 450, 990, 450);

        memset(&line_4, 0, sizeof(line_4));
        Line_Draw(&line_4, "902", UI_Graph_ADD, 3, UI_Color_Yellow, 2, 930, 473, 990, 473);

        memset(&line_7, 0, sizeof(line_7));
        Line_Draw(&line_7, "903", UI_Graph_ADD, 3, UI_Color_Pink, 2, 900, 483, 1020, 483);

        memset(&line_10, 0, sizeof(line_10));
        Line_Draw(&line_10, "904", UI_Graph_ADD, 3, UI_Color_Pink, 2, 870, 473, 1050, 473);

        memset(&line_11, 0, sizeof(line_11));
        Line_Draw(&line_11, "905", UI_Graph_ADD, 3, UI_Color_Green, 2, 840, 462, 1080, 462);

        memset(&line_12, 0, sizeof(line_12));
        Line_Draw(&line_12, "906", UI_Graph_ADD, 3, UI_Color_Green, 2, 840, 442, 1080, 442);

        UI_ReFresh(5, line_4, line_7, line_10, line_11, line_12);
    }

    //刹车辅助线
    Graph_Data line_16;
    Graph_Data line_17;
    memset(&line_16, 0, sizeof(line_16));
    memset(&line_17, 0, sizeof(line_17));
    Line_Draw(&line_16, "916", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 490, 100, 840, 400);
    Line_Draw(&line_17, "917", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 1410, 100, 1060, 400);

    UI_ReFresh(2, line_16, line_17);
}

void robot_id_data_init(void) {  //无空中、哨兵、雷达站的id
    id_data.ID[0] = 1;
    id_data.ID[1] = 2;
    id_data.ID[2] = 3;
    id_data.ID[3] = 4;
    id_data.ID[4] = 5;


    id_data.ID[5] = 101;
    id_data.ID[6] = 102;
    id_data.ID[7] = 103;
    id_data.ID[8] = 104;
    id_data.ID[9] = 105;


    id_data.sender_ID[0] = 1;
    id_data.sender_ID[1] = 2;
    id_data.sender_ID[2] = 3;
    id_data.sender_ID[3] = 4;
    id_data.sender_ID[4] = 5;

    id_data.sender_ID[5] = 101;
    id_data.sender_ID[6] = 102;
    id_data.sender_ID[7] = 103;
    id_data.sender_ID[8] = 104;
    id_data.sender_ID[9] = 105;

    id_data.receiver_ID[0] = 0x101;
    id_data.receiver_ID[1] = 0x102;
    id_data.receiver_ID[2] = 0x103;
    id_data.receiver_ID[3] = 0x104;
    id_data.receiver_ID[4] = 0x105;

    id_data.receiver_ID[5] = 0x165;
    id_data.receiver_ID[6] = 0x166;
    id_data.receiver_ID[7] = 0x167;
    id_data.receiver_ID[8] = 0x168;
    id_data.receiver_ID[9] = 0x169;
}


void robot_id_select(void) {
    Uint8_t i = 0;
    Robot_number = get_robot_id();
    for (i = 0; i <= 9; i++) {
        if (Robot_number == id_data.ID[i]) {
            Robot_cline_number = id_data.receiver_ID[i];
        }
    }
}

