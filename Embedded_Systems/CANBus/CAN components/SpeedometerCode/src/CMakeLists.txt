SET(SOURCES ui_Screen1.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_white_writing3_png.c
    ui_img_warning_unlit_png.c
    ui_img_warning_lit_png.c
    ui_img_fullbeam_unlit_png.c
    ui_img_fullbeam_lit_png.c
    ui_img_indicator_right_unlit_png.c
    ui_img_indicator_right_partial_png.c
    ui_img_indicator_right_partial2_png.c
    ui_img_indicator_right_lit_png.c
    ui_img_status_batt_ok_png.c
    ui_img_status_batt_warn_png.c
    ui_img_status_temp_ok_png.c
    ui_img_status_temp_warn_png.c
    ui_img_status_comms_ok_png.c
    ui_img_status_comms_warn_png.c
    ui_img_status_solar_ok_png.c
    ui_img_status_solar_warn_png.c
    ui_img_status_ctrl_ok_png.c
    ui_img_status_ctrl_warn_png.c
    ui_img_status_motor_ok_png.c
    ui_img_status_motor_warn_png.c
    fonts/ui_font_InterMedium18.c
    fonts/ui_font_InterMedium20.c
    fonts/ui_font_InterSemiBold22.c
    fonts/ui_font_RajdhaniBold40.c
    fonts/ui_font_RajdhaniMedium32.c
    fonts/ui_font_RajdhaniSemiBold30.c
    fonts/ui_font_RajdhaniSemiBold96.c)

add_library(ui ${SOURCES})
