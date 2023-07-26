//
// Created by Jessi on 2022/4/13.
//
#include "UI_SavingLabel.h"
#include "RM_Cilent_UI.h"


sl_handle *sl = NULL;

int sl_update(sl_handle *self, int length, char *string) {
    static char buffer[30];  // 30 是最长字符串长度
    for (int i = 0; i < length; i++) {
        buffer[i] = *(string + i);
    }
//    return write_a_char(self->name_array, length, buffer, self->x, self->y, self->font_size, self->colour_code,
//                        self->width, self->layer);
}

