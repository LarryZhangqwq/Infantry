//
// Created by Jessi on 2022/4/13.
//

#ifndef UIGROUP_UI_SAVINGLABEL_H
#define UIGROUP_UI_SAVINGLABEL_H

#ifndef uint8_t

#include <stdint.h>

#endif  // uint8_t


typedef struct sl
{
    uint16_t x;
    uint16_t y;

    uint8_t width;
    uint8_t font_size;
    uint8_t colour_code;

    uint8_t layer;

    char booster_state_data[30];
} sl_handle;

extern sl_handle *sl;

int sl_update(sl_handle *, int, char *);
int sl_delete(sl_handle *);

#endif //UIGROUP_UI_SAVINGLABEL_H
