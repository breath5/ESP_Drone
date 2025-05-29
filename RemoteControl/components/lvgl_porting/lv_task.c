
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./driver/gpio.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
//#include "lv_demos.h"
#include "lv_task.h"
#include "lv_demos.h"
#include "lvgl_helpers.h"
#include "xpt2046.h"
#include "tp_spi.h"
#include "touch_driver.h"

#include <stdio.h>//兼容
#include <stdbool.h>//兼容
#include <unistd.h>//兼容
#include "Data_declaration.h"
#include "esp_task_wdt.h"

LV_FONT_DECLARE(Chinese_character_library);//声明中文字库
LV_IMG_DECLARE(uav1); //声明姿态仪图片
LV_IMG_DECLARE(ns); //声明图片
LV_FONT_DECLARE(lv_font_montserrat_14);//声明使用的字体大小

char buf[20]; //文本更换的中间变量

 lv_obj_t * thrust_bar = NULL;
 lv_obj_t * yaw_bar = NULL;
 lv_obj_t * roll_bar = NULL;
 lv_obj_t * pitch_bar = NULL;

 lv_obj_t* thrust_val = NULL;
 lv_obj_t* roll_val = NULL;
 lv_obj_t* pitch_val = NULL;
 lv_obj_t* yaw_val = NULL;
 lv_obj_t* uav_vbat = NULL;

 lv_obj_t* drone_roll = NULL;
 lv_obj_t* drone_yaw = NULL;
 lv_obj_t* drone_pitch = NULL;
 lv_obj_t* drone_voltage = NULL;

static  lv_obj_t * meter;
static  lv_meter_indicator_t * indicator;

/*用定时器给LVGL提供时钟*/
static void lv_tick_task(void *arg)
{
	(void)arg;
	lv_tick_inc(10);

}

//刷新指针的值
static void vol_event_cb(lv_event_t * e)
{
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if(dsc->part != LV_PART_INDICATOR) return;

    lv_obj_t * obj = lv_event_get_target(e);

    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.font = LV_FONT_DEFAULT;

    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d", (int)lv_bar_get_value(obj));

    lv_point_t txt_size;
    lv_txt_get_size(&txt_size, buf, label_dsc.font, label_dsc.letter_space, label_dsc.line_space, LV_COORD_MAX,
                    label_dsc.flag);

    lv_area_t txt_area;
    /*If the indicator is long enough put the text inside on the right*/
    if(lv_area_get_width(dsc->draw_area) > txt_size.x + 20) {
        txt_area.x2 = dsc->draw_area->x2 - 5;
        txt_area.x1 = txt_area.x2 - txt_size.x + 1;
        label_dsc.color = lv_color_white();
    }
        /*If the indicator is still short put the text out of it on the right*/
    else {
        txt_area.x1 = dsc->draw_area->x2 + 5;
        txt_area.x2 = txt_area.x1 + txt_size.x - 1;
        label_dsc.color = lv_color_black();
    }

    txt_area.y1 = dsc->draw_area->y1 + (lv_area_get_height(dsc->draw_area) - txt_size.y) / 2;
    txt_area.y2 = txt_area.y1 + txt_size.y - 1;

    lv_draw_label(dsc->draw_ctx, &label_dsc, &txt_area, buf, NULL);
}
void lv_meter_custom(void)
{
    // 创建仪表盘对象
    meter = lv_meter_create(lv_scr_act());
    lv_obj_center(meter);
    //lv_obj_set_pos(meter, 150, 50);
    lv_obj_set_size(meter, 215, 215);

    lv_obj_remove_style(meter, NULL, LV_PART_INDICATOR);
    // 添加刻度
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 101, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 10, 4, 15, lv_color_black(), 15);

    // 设置刻度的范围为 0 ~ 1000
    lv_meter_set_scale_range(meter, scale, 0, 1000, 320, 135-25);

    // 添加绿色弧线 (0~400)
    lv_meter_indicator_t * indic;
    indic = lv_meter_add_arc(meter, scale, 4, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 300);

    // 添加黄色弧线 (400~700)
    indic = lv_meter_add_arc(meter, scale, 4, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_meter_set_indicator_start_value(meter, indic, 300);
    lv_meter_set_indicator_end_value(meter, indic, 700);

    // 添加红色弧线 (700~1000)
    indic = lv_meter_add_arc(meter, scale, 4, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 700);
    lv_meter_set_indicator_end_value(meter, indic, 1000);

    // 添加指针
    indicator = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_BLUE), -10);

    // 使用定时器定期更新 thrust_value
    //lv_timer_t * timer = lv_timer_create(update_thrust_value, 500, indic);
}
//处理lvgl事件
void lv_task(void *arg)
{
    TickType_t adp = xTaskGetTickCount();
    const TickType_t adg = 10;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行

    while(1){
        vTaskDelayUntil(&adp,adg);
        lv_task_handler();
        esp_task_wdt_delete(NULL); // 删除当前任务

        lv_bar_set_value(thrust_bar, (int)setpoint.thrust, LV_ANIM_OFF);
        lv_bar_set_value(yaw_bar, (int)setpoint.attitude.yaw, LV_ANIM_OFF);
        lv_bar_set_value(roll_bar, (int)setpoint.attitude.roll, LV_ANIM_OFF);
        lv_bar_set_value(pitch_bar, (int)setpoint.attitude.pitch, LV_ANIM_OFF);
        lv_bar_set_value(drone_voltage, (int)(((float)UAV_VBAT/470)*100), LV_ANIM_OFF);
        lv_meter_set_indicator_value(meter, indicator, (int32_t)setpoint.thrust);  //刷新指针的值



        sprintf(buf, "Thrust %0.2f  ",setpoint.thrust);
        vTaskDelay(pdMS_TO_TICKS(1));
        lv_label_set_text(thrust_val, buf);
        sprintf(buf, "Yaw: %d ",(int)setpoint.attitude.yaw);
        vTaskDelay(pdMS_TO_TICKS(1));
        lv_label_set_text(yaw_val, buf);
        sprintf(buf, "Roll: %d ",(int)setpoint.attitude.roll);
        vTaskDelay(pdMS_TO_TICKS(1));
        lv_label_set_text(roll_val, buf);
        sprintf(buf, "Pitch: %d ",(int)setpoint.attitude.pitch);
        vTaskDelay(pdMS_TO_TICKS(1));
        lv_label_set_text(pitch_val, buf);


			sprintf(buf, "roll %0.2f ",state.attitude.roll);
			lv_label_set_text(drone_roll, buf);
			sprintf(buf, "pitch %0.2f",state.attitude.pitch);
			lv_label_set_text(drone_pitch, buf);
			sprintf(buf, "yaw %0.2f ",state.attitude.yaw);
            lv_label_set_text(drone_yaw, buf);
//			sprintf(buf, "alt %d ",alt);
//			lv_label_set_text(uav_alt, buf);
			sprintf(buf, "uav_vbat %d ",UAV_VBAT);
			lv_label_set_text(uav_vbat, buf);

    }
}



void LVGL_picture_establish(void){//创建画面

    //lv_obj_t* scr = lv_obj_create(NULL); //创建屏幕
    //lv_scr_load(scr); // 加载屏幕
    lv_obj_t* scr = lv_scr_act();

    static lv_style_t style_indic;         //创建垂直进度条模板
    lv_style_init(&style_indic);           //清空
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);       //设置为竖直方向

    thrust_bar = lv_bar_create(scr);       //创建油门进度条
    lv_obj_add_style(thrust_bar, &style_indic, LV_PART_SCROLLBAR); //应用模板
    lv_bar_set_range(thrust_bar, 0, 1000); //设置值的范围0-1000
    lv_obj_set_size(thrust_bar, 20, 100);  //设置大小，宽10长100
    lv_obj_set_pos(thrust_bar, 0, 200);    //设置位置

    yaw_bar = lv_bar_create(scr); 		//创建油门进度条
    lv_obj_set_pos(yaw_bar, 0, 300);    //设置位置
    lv_obj_set_size(yaw_bar, 100, 20);  //设置大小，宽10长100
    lv_bar_set_range(yaw_bar, 0, 3000); //设置值的范围0-1000

    pitch_bar = lv_bar_create(scr);       //创建油门进度条
    lv_obj_add_style(pitch_bar, &style_indic, LV_PART_SCROLLBAR);  //应用模板
    lv_bar_set_range(pitch_bar, 0, 3000); //设置值的范围0-1000
    lv_obj_set_size(pitch_bar, 20, 100);  //设置大小，宽10长100
    lv_obj_set_pos(pitch_bar, 460, 200);  //设置位置

    roll_bar = lv_bar_create(scr); 		 //创建油门进度条
    lv_obj_set_pos(roll_bar, 380, 300);  //设置位置
    lv_obj_set_size(roll_bar, 100, 20);  //设置大小，宽10长100
    lv_bar_set_range(roll_bar, 0, 3000); //设置值的范围0-1000

    drone_voltage = lv_bar_create(scr);
    lv_obj_add_event_cb(drone_voltage, vol_event_cb, LV_EVENT_DRAW_PART_END, NULL);//电压进度条
    lv_obj_set_pos(drone_voltage, 190, 300);
    lv_obj_set_size(drone_voltage, 100, 20);


    thrust_val = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(thrust_val, 10, 10);  		// 设置标签位置
    lv_label_set_text(thrust_val, "----");	    // 设置标签内容

    yaw_val = lv_label_create(scr);        	    // 创建一个文本
    lv_obj_set_pos(yaw_val, 10, 30);			// 设置标签位置
    lv_label_set_text(yaw_val, "----");			// 设置标签内容


    roll_val = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(roll_val, 10, 50);			// 设置标签位置
    lv_label_set_text(roll_val, "----");		// 设置标签内容

    pitch_val = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(pitch_val, 10, 70);			// 设置标签位置
    lv_label_set_text(pitch_val, "----");		// 设置标签内容


    drone_roll = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(drone_roll, 360, 10);			// 设置标签位置
    lv_label_set_text(drone_roll, "----");		// 设置标签内容


    drone_pitch = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(drone_pitch, 360, 30);			// 设置标签位置
    lv_label_set_text(drone_pitch, "----");		// 设置标签内容

    drone_yaw = lv_label_create(scr);        		// 创建一个文本
    lv_obj_set_pos(drone_yaw, 360, 50);			// 设置标签位置
    lv_label_set_text(drone_yaw, "----");			// 设置标签内容
//
//    uav_alt = lv_label_create(scr);        		// 创建一个文本
//    lv_obj_set_pos(uav_alt, 360, 70);			// 设置标签位置
//    lv_label_set_text(uav_alt, "----");			// 设置标签内容
//
    uav_vbat = lv_label_create(scr);        	// 创建一个文本
    lv_obj_set_pos(uav_vbat, 360, 90);			// 设置标签位置
    lv_label_set_text(uav_vbat, "----");		// 设置标签内容

}



void lv_task_init(void)
{
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    lv_group_t *group = lv_group_create();
    lv_indev_set_group(indev_keypad, group);

    /*  esp_register_freertos_tick_hook(lv_tick_task);
    Create and start a periodic timer interrupt to call lv_tick_inc 
    */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));

    LVGL_picture_establish();//创建画面
    lv_meter_custom();//油表
    //lv_demo_stress();

// xTaskCreate(lv_task, "lv_task",1024*8,NULL,5,NULL);//创建LVGL处理任务
   xTaskCreatePinnedToCore(lv_task, "lv_task", 1024 * 8, NULL, 5, NULL, 1); // Create LVGL processing task on CPU1

}