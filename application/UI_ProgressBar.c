
#include <stdio.h>
#include <math.h>
#include "UI_ProgressBar.h"
#include "RM_Cilent_UI.h"
#include "cmsis_os.h"
#include "shoot.h"
#include "string.h"
#include "chassis_task.h"


/* **说明  条形进度条**
使用 UI_ProgressBar_static 初始化
初始化定义为100%的进度
调用 UI_ProgressBar_change 更新进度条
需提前以 progress_bar_data 建立结构体输入数据
*/



void UI_ProgressBar_static(progress_bar_data *bar)//初始化和静态量
{
//    Graph_Data label_1;
//    memset(&label_1, 0, sizeof(label_1));
//    Rectangle_Draw(&label_1, "111", UI_Graph_ADD, 1, UI_Color_White, 2, border_start_x, border_start_y,
//                   border_end_x, border_end_y);

//    Graph_Data label_2;
//    memset(&label_2, 0, sizeof(label_2));
//    Rectangle_Draw(&label_2, "112", UI_Graph_ADD, 1, UI_Color_White, 2,
//                   border_start_x + border_line_width + ProgressFilling_line_width / 2,
//                   border_start_y + border_line_width + ProgressFilling_line_width / 2,
//                   border_start_x + 5 * bar->progress_bar_data_change  - border_line_width,
//                   border_start_y + 5 * bar->progress_bar_data_change  - border_line_width);

    String_Data label_3;
    char string[12];
    memset(&label_3, 0, sizeof(label_3));
    memset(string, 0, sizeof(char) * 8);
    sprintf(string, "%d%%", bar->progress_bar_data_change);
    Char_Draw(&label_3, "113", UI_Graph_ADD, 1, UI_Color_Cyan, 28, 4, 4, 910, 200, string);
//    UI_ReFresh(2, label_1, label_2);
    Char_ReFresh(label_3);

    memset(&label_3, 0, sizeof(label_3));
    Char_Draw(&label_3, "114", UI_Graph_ADD, 1, UI_Color_Cyan, 28, 1, 4, 930, 245,
              get_shooter_mode() ? "L" : "R");
    Char_ReFresh(label_3);

}

void UI_ProgressBar_change(progress_bar_data *bar)  //动态量
{
//    Graph_Data label_4;
//    memset(&label_4, 0, sizeof(label_4));
//    Rectangle_Draw(&label_4, "112", UI_Graph_Change, 1, UI_Color_White, 12,
//                   border_start_x + border_line_width + ProgressFilling_line_width / 2,
//                   border_start_y + border_line_width + ProgressFilling_line_width / 2,
//                   border_start_x + 5 * bar->progress_bar_data_change  - border_line_width,
//                   border_start_y + 5 * bar->progress_bar_data_change  - border_line_width);
//    UI_ReFresh(1, label_4);

    String_Data label_5;
    char string[12];
    memset(&label_5, 0, sizeof(label_5));
    memset(string, 0, sizeof(char) * 8);
    sprintf(string, "%d%%", bar->progress_bar_data_change);
    uint8_t colour;
    if (bar->progress_bar_data_change < 0) {
        // 电容异常
        colour = UI_Color_Purplish_red;
    } else if (get_cap_state() == 1) {
        // 电容正常且正在使用
        colour = UI_Color_Cyan;
    } else {  // bar->progress_bar_data_change >= 0
        // 电容正常且未使用
        colour = UI_Color_Yellow;
    }
    Char_Draw(&label_5, "113", UI_Graph_Change, 1, colour, 28, 4, 4, 910, 200, string);
    Char_ReFresh(label_5);

    memset(&label_5, 0, sizeof(label_5));
    Char_Draw(&label_5, "114", UI_Graph_Change, 1, UI_Color_Cyan, 28, 1, 4, 930, 245,
              get_shooter_mode() ? "L" : "R");
    Char_ReFresh(label_5);

}//希望以10hz频率更新数据  @Juntong :这是谁在什么时候写的？ @Miao :这是gyx在写进度条控件的时候写的
