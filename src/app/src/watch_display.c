#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(watch_display, CONFIG_ZSW_APP_LOG_LEVEL);
static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* decalare screens*/
lv_obj_t *screen1;   /* screen object have labels,text, and others*/
lv_obj_t *screen2; 

/*screen1 labels*/
lv_obj_t *rtc_label;

/*screen2 label*/
lv_obj_t *direction_label;

uint8_t change_screen = 1;

int watch_display_init()
{   
    int ret;
    /*check if oled display is ready*/
    if (!device_is_ready(display)) {
        LOG_ERR("Device %s is not ready\n", display->name);
		return -ENODEV;
    }

    k_sleep(K_MSEC(100));
    
    /*turn off the display*/
    ret = display_blanking_off(display);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_ERR("Failed to turn blanking off (error %d)", ret);
		return -ENODEV;
	}

    return 0;
}

void create_screen1()
{
    screen1 = lv_obj_create(NULL);   /*create screen*/
    rtc_label = lv_label_create(screen1);   /*screen1 is a parent of rtc label,Create a dynamic label widget*/

    lv_obj_align(rtc_label, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_text_color(rtc_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(screen1, lv_color_black(), 0);
}

void create_screen2()
{
    screen2 = lv_obj_create(NULL);   /*create screen*/
    direction_label = lv_label_create(screen2);   

    lv_obj_align(direction_label, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_text_color(direction_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(screen2, lv_color_black(), 0);
}

void switch_screens()
{
    if(change_screen)
    {
        lv_scr_load(screen2);
        change_screen = 0;
    }

    else 
    {
        lv_scr_load(screen1);
        change_screen = 1;
    }
}
