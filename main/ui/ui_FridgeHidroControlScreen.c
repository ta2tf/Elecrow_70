// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.3.6
// Project name: MiniCombine2

#include "ui.h"

void ui_FridgeHidroControlScreen_screen_init(void)
{
ui_FridgeHidroControlScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_FridgeHidroControlScreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_FridgeHidroControlScreen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_FridgeHidroControlScreen, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_FridgeOnOffImage = lv_img_create(ui_FridgeHidroControlScreen);
lv_img_set_src(ui_FridgeOnOffImage, &ui_img_1521626150);
lv_obj_set_width( ui_FridgeOnOffImage, 100);
lv_obj_set_height( ui_FridgeOnOffImage, LV_SIZE_CONTENT);   /// 100
lv_obj_set_x( ui_FridgeOnOffImage, -70 );
lv_obj_set_y( ui_FridgeOnOffImage, -24 );
lv_obj_set_align( ui_FridgeOnOffImage, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_FridgeOnOffImage, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_FridgeOnOffImage, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_FridgeOnOffIcon = lv_img_create(ui_FridgeOnOffImage);
lv_img_set_src(ui_FridgeOnOffIcon, &ui_img_fridge_48_w_png);
lv_obj_set_width( ui_FridgeOnOffIcon, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_FridgeOnOffIcon, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_FridgeOnOffIcon, 2 );
lv_obj_set_y( ui_FridgeOnOffIcon, 4 );
lv_obj_set_align( ui_FridgeOnOffIcon, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_FridgeOnOffIcon, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_FridgeOnOffIcon, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_HidroforOnOffImage = lv_img_create(ui_FridgeHidroControlScreen);
lv_img_set_src(ui_HidroforOnOffImage, &ui_img_439896971);
lv_obj_set_width( ui_HidroforOnOffImage, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_HidroforOnOffImage, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_HidroforOnOffImage, 56 );
lv_obj_set_y( ui_HidroforOnOffImage, -23 );
lv_obj_set_align( ui_HidroforOnOffImage, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_HidroforOnOffImage, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_HidroforOnOffImage, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_HidroforOnOffIcon = lv_img_create(ui_HidroforOnOffImage);
lv_img_set_src(ui_HidroforOnOffIcon, &ui_img_hidrofor_48_w_png);
lv_obj_set_width( ui_HidroforOnOffIcon, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_HidroforOnOffIcon, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_HidroforOnOffIcon, 3 );
lv_obj_set_y( ui_HidroforOnOffIcon, 1 );
lv_obj_set_align( ui_HidroforOnOffIcon, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_HidroforOnOffIcon, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_HidroforOnOffIcon, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_img_recolor(ui_HidroforOnOffIcon, lv_color_hex(0xFF0000), LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_img_recolor_opa(ui_HidroforOnOffIcon, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_HeadContainer2 = lv_obj_create(ui_FridgeHidroControlScreen);
lv_obj_set_width( ui_HeadContainer2, 320);
lv_obj_set_height( ui_HeadContainer2, 28);
lv_obj_set_x( ui_HeadContainer2, 1 );
lv_obj_set_y( ui_HeadContainer2, -103 );
lv_obj_set_align( ui_HeadContainer2, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_HeadContainer2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_HeadContainer2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_HeadContainer2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_HeadContainer2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_HeadContainer2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_OctoopiLogoImage2 = lv_img_create(ui_HeadContainer2);
lv_img_set_src(ui_OctoopiLogoImage2, &ui_img_octoopilogo_png);
lv_obj_set_width( ui_OctoopiLogoImage2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_OctoopiLogoImage2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_OctoopiLogoImage2, -147 );
lv_obj_set_y( ui_OctoopiLogoImage2, -1 );
lv_obj_set_align( ui_OctoopiLogoImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_OctoopiLogoImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_OctoopiLogoImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_img_set_zoom(ui_OctoopiLogoImage2,230);

ui_SetupImage2 = lv_img_create(ui_HeadContainer2);
lv_img_set_src(ui_SetupImage2, &ui_img_setup_png);
lv_obj_set_width( ui_SetupImage2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SetupImage2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SetupImage2, 146 );
lv_obj_set_y( ui_SetupImage2, -1 );
lv_obj_set_align( ui_SetupImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_SetupImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_SetupImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_InMonitorContainer2 = lv_obj_create(ui_HeadContainer2);
lv_obj_set_width( ui_InMonitorContainer2, 212);
lv_obj_set_height( ui_InMonitorContainer2, 28);
lv_obj_set_x( ui_InMonitorContainer2, 5 );
lv_obj_set_y( ui_InMonitorContainer2, 0 );
lv_obj_set_align( ui_InMonitorContainer2, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_InMonitorContainer2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_InMonitorContainer2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_InMonitorContainer2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_InMonitorContainer2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_InMonitorContainer2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MonTrunkImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonTrunkImage2, &ui_img_trunk_open_png);
lv_obj_set_width( ui_MonTrunkImage2, 24);
lv_obj_set_height( ui_MonTrunkImage2, 24);
lv_obj_set_x( ui_MonTrunkImage2, -87 );
lv_obj_set_y( ui_MonTrunkImage2, -1 );
lv_obj_set_align( ui_MonTrunkImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonTrunkImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonTrunkImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MonDoorImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonDoorImage2, &ui_img_door_lock_close_png);
lv_obj_set_width( ui_MonDoorImage2, 24);
lv_obj_set_height( ui_MonDoorImage2, 24);
lv_obj_set_x( ui_MonDoorImage2, -52 );
lv_obj_set_y( ui_MonDoorImage2, -1 );
lv_obj_set_align( ui_MonDoorImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonDoorImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonDoorImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MonPlugImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonPlugImage2, &ui_img_plug_yes_png);
lv_obj_set_width( ui_MonPlugImage2, 24);
lv_obj_set_height( ui_MonPlugImage2, 24);
lv_obj_set_x( ui_MonPlugImage2, -15 );
lv_obj_set_y( ui_MonPlugImage2, -1 );
lv_obj_set_align( ui_MonPlugImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonPlugImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonPlugImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MonLampImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonLampImage2, &ui_img_outdoor_lamp_on_png);
lv_obj_set_width( ui_MonLampImage2, 24);
lv_obj_set_height( ui_MonLampImage2, 24);
lv_obj_set_x( ui_MonLampImage2, 19 );
lv_obj_set_y( ui_MonLampImage2, -1 );
lv_obj_set_align( ui_MonLampImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonLampImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonLampImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MonBlueImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonBlueImage2, &ui_img_bluetooth_yes_png);
lv_obj_set_width( ui_MonBlueImage2, 24);
lv_obj_set_height( ui_MonBlueImage2, 24);
lv_obj_set_x( ui_MonBlueImage2, 55 );
lv_obj_set_y( ui_MonBlueImage2, -1 );
lv_obj_set_align( ui_MonBlueImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonBlueImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonBlueImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_MonWifiImage2 = lv_img_create(ui_InMonitorContainer2);
lv_img_set_src(ui_MonWifiImage2, &ui_img_wifi_yes_png);
lv_obj_set_width( ui_MonWifiImage2, 24);
lv_obj_set_height( ui_MonWifiImage2, 24);
lv_obj_set_x( ui_MonWifiImage2, 89 );
lv_obj_set_y( ui_MonWifiImage2, -1 );
lv_obj_set_align( ui_MonWifiImage2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_MonWifiImage2, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_MonWifiImage2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

}