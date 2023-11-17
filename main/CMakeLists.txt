 idf_component_register(
    SRCS rgb_lcd_example_main.c
         lvgl_demo_ui.c
 
ui/ui_Screen1.c
ui/ui.c
ui/ui_comp_hook.c
ui/ui_helpers.c


 
    )
set(COMPONENT_ADD_INCLUDEDIRS "." "ui")
 target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")