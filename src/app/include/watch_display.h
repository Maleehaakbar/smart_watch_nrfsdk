#ifndef WATCH_DISPLAY_H
#define WATCH_DISPLAY_H

#include <lvgl.h>

extern lv_obj_t *screen1;    /*extern screens to use in main.c*/
extern lv_obj_t *screen2;

extern lv_obj_t *rtc_label;
extern lv_obj_t *direction_label;

int watch_display_init();
void create_screen1();
void create_screen2();
void switch_screens();

#endif