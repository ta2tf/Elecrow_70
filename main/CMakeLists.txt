 idf_component_register(
    SRCS rgb_lcd_example_main.c
         lvgl_demo_ui.c


ui/ui_Screen_2_Move.c
ui/ui.c
ui/ui_comp_hook.c
ui/ui_helpers.c
ui/ui_events.c
ui/ui_img_bg_png.c
ui/ui_img_btn_pad_bottom_png.c
ui/ui_img_btn_pad_top_png.c
ui/ui_img_arrow_up_png.c
ui/ui_img_arrow_down_png.c
ui/ui_img_arrow_left_png.c
ui/ui_img_arrow_right_png.c
 
ui/ui_img_s1_png.c
ui/ui_img_s2_png.c
ui/ui_img_s3_png.c
ui/ui_img_s4_png.c
ui/ui_img_s5_png.c
ui/ui_img_s6_png.c
ui/ui_img_n4_png.c 

ui/ui_img_e1_png.c
ui/ui_img_e2_png.c
ui/ui_img_e3_png.c
ui/ui_img_e4_png.c
ui/ui_img_e5_png.c
ui/ui_img_e6_png.c
ui/ui_img_e7_png.c
ui/ui_img_e8_png.c
ui/ui_img_e9_png.c
ui/ui_img_e10_png.c
 

 
    )
set(COMPONENT_ADD_INCLUDEDIRS "." "ui")
 target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")