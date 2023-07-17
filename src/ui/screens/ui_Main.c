// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: nevermore

#include "../ui.h"

void ui_Main_screen_init(void)
{
    ui_Main = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Main, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Main, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Main, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel0 = lv_obj_create(ui_Main);
    lv_obj_set_width(ui_Panel0, lv_pct(45));
    lv_obj_set_height(ui_Panel0, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Panel0, lv_pct(0));
    lv_obj_set_y(ui_Panel0, lv_pct(6));
    lv_obj_set_align(ui_Panel0, LV_ALIGN_TOP_MID);
    lv_obj_set_flex_flow(ui_Panel0, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Panel0, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Panel0, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel0, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Panel0, LV_BORDER_SIDE_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PressureLabels = lv_obj_create(ui_Panel0);
    lv_obj_set_width(ui_PressureLabels, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_PressureLabels, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_PressureLabels, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_PressureLabels, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_PressureLabels, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_PressureLabels, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_PressureLabels, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_PressureLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PressureIn = lv_label_create(ui_PressureLabels);
    lv_obj_set_width(ui_PressureIn, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PressureIn, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_PressureIn, LV_ALIGN_CENTER);
    lv_label_set_text(ui_PressureIn, "??? kPa");
    lv_obj_set_style_text_color(ui_PressureIn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_PressureIn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_PressureIn, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_PressureIn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_PressureIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_PressureIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_PressureIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PressureOut = lv_label_create(ui_PressureLabels);
    lv_obj_set_width(ui_PressureOut, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PressureOut, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_PressureOut, LV_ALIGN_CENTER);
    lv_label_set_text(ui_PressureOut, "??? kPa");
    lv_obj_set_style_text_color(ui_PressureOut, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_PressureOut, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_PressureOut, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_PressureOut, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_PressureOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_PressureOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_PressureOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumidityLabels = lv_obj_create(ui_Panel0);
    lv_obj_set_width(ui_HumidityLabels, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_HumidityLabels, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_HumidityLabels, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_HumidityLabels, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_HumidityLabels, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_HumidityLabels, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_HumidityLabels, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_HumidityLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumidityIn = lv_label_create(ui_HumidityLabels);
    lv_obj_set_width(ui_HumidityIn, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HumidityIn, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_HumidityIn, LV_ALIGN_CENTER);
    lv_label_set_text(ui_HumidityIn, "??.?%");
    lv_obj_set_style_text_color(ui_HumidityIn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_HumidityIn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_HumidityIn, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_HumidityIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_HumidityIn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_HumidityIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_HumidityIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumidityOut = lv_label_create(ui_HumidityLabels);
    lv_obj_set_width(ui_HumidityOut, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HumidityOut, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_HumidityOut, LV_ALIGN_CENTER);
    lv_label_set_text(ui_HumidityOut, "??.?%");
    lv_obj_set_style_text_color(ui_HumidityOut, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_HumidityOut, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_HumidityOut, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_HumidityOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_HumidityOut, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_HumidityOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_HumidityOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ChartBox = lv_obj_create(ui_Main);
    lv_obj_set_width(ui_ChartBox, lv_pct(80));
    lv_obj_set_height(ui_ChartBox, lv_pct(65));
    lv_obj_set_x(ui_ChartBox, 0);
    lv_obj_set_y(ui_ChartBox, lv_pct(5));
    lv_obj_set_align(ui_ChartBox, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_ChartBox, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_ChartBox, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_ChartBox, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ChartBox, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_ChartBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ChartOverlay = lv_obj_create(ui_ChartBox);
    lv_obj_set_width(ui_ChartOverlay, lv_pct(100));
    lv_obj_set_flex_grow(ui_ChartOverlay, 1);
    lv_obj_set_align(ui_ChartOverlay, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_ChartOverlay, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ChartOverlay, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_ChartOverlay, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Chart = lv_chart_create(ui_ChartOverlay);
    lv_obj_set_width(ui_Chart, lv_pct(100));
    lv_obj_set_height(ui_Chart, lv_pct(100));
    lv_obj_set_align(ui_Chart, LV_ALIGN_CENTER);
    lv_chart_set_type(ui_Chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0, 500);
    lv_chart_set_range(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0, 90);
    lv_chart_set_div_line_count(ui_Chart, 10, 10);
    lv_chart_set_axis_tick(ui_Chart, LV_CHART_AXIS_PRIMARY_X, 8, 5, 10, 2, false, 35);
    lv_chart_set_axis_tick(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 5, 3, 0, 2, false, 50);
    lv_chart_set_axis_tick(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 5, 3, 0, 2, false, 25);
    lv_obj_set_style_radius(ui_Chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Chart, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Chart, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Chart, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Chart, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_line_color(ui_Chart, lv_color_hex(0xFFFFFF), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui_Chart, 255, LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Chart, lv_color_hex(0xFFFFFF), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Chart, 255, LV_PART_TICKS | LV_STATE_DEFAULT);

    ui_ChartLabels = lv_obj_create(ui_ChartOverlay);
    lv_obj_set_width(ui_ChartLabels, LV_SIZE_CONTENT);   /// 154
    lv_obj_set_height(ui_ChartLabels, LV_SIZE_CONTENT);    /// 109
    lv_obj_set_flex_flow(ui_ChartLabels, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_ChartLabels, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_ChartLabels, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ChartLabels, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ChartLabels, 128, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_ChartLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_VocLabels = lv_obj_create(ui_ChartLabels);
    lv_obj_set_width(ui_VocLabels, LV_SIZE_CONTENT);   /// 154
    lv_obj_set_height(ui_VocLabels, LV_SIZE_CONTENT);    /// 109
    lv_obj_set_flex_flow(ui_VocLabels, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_VocLabels, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_VocLabels, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_VocLabels, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_VocLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_VocIn = lv_label_create(ui_VocLabels);
    lv_obj_set_width(ui_VocIn, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_VocIn, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_VocIn, LV_ALIGN_CENTER);
    lv_label_set_text(ui_VocIn, "???");
    lv_obj_set_style_text_color(ui_VocIn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_VocIn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_VocIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_VocIn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_VocIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_VocIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_VocOut = lv_label_create(ui_VocLabels);
    lv_obj_set_width(ui_VocOut, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_VocOut, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_VocOut, LV_ALIGN_CENTER);
    lv_label_set_text(ui_VocOut, "???");
    lv_obj_set_style_text_color(ui_VocOut, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_VocOut, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_VocOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_VocOut, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_VocOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_VocOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TempLabels = lv_obj_create(ui_ChartLabels);
    lv_obj_set_width(ui_TempLabels, LV_SIZE_CONTENT);   /// 154
    lv_obj_set_height(ui_TempLabels, LV_SIZE_CONTENT);    /// 109
    lv_obj_set_align(ui_TempLabels, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_set_flex_flow(ui_TempLabels, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_TempLabels, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_TempLabels, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_TempLabels, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_TempLabels, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TempIn = lv_label_create(ui_TempLabels);
    lv_obj_set_width(ui_TempIn, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TempIn, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_TempIn, LV_ALIGN_CENTER);
    lv_label_set_text(ui_TempIn, "?.?c");
    lv_obj_set_style_text_color(ui_TempIn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_TempIn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_TempIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_TempIn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_TempIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_TempIn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TempOut = lv_label_create(ui_TempLabels);
    lv_obj_set_width(ui_TempOut, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TempOut, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_TempOut, LV_ALIGN_CENTER);
    lv_label_set_text(ui_TempOut, "?.?c");
    lv_obj_set_style_text_color(ui_TempOut, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_TempOut, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_TempOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_TempOut, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_TempOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_TempOut, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ChartMax = lv_label_create(ui_ChartOverlay);
    lv_obj_set_width(ui_ChartMax, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ChartMax, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ChartMax, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_ChartMax, "");
    lv_obj_set_style_text_color(ui_ChartMax, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_ChartMax, 220, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_ChartMax, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_ChartMax, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_ChartMax, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_ChartMax, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_ChartMax, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_ChartMax, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_XAxisScale = lv_label_create(ui_ChartBox);
    lv_obj_set_width(ui_XAxisScale, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_XAxisScale, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_XAxisScale, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_XAxisScale, "1h");
    lv_obj_set_style_text_color(ui_XAxisScale, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_XAxisScale, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_XAxisScale, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_XAxisScale, 128, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FanBox = lv_obj_create(ui_Main);
    lv_obj_set_width(ui_FanBox, lv_pct(30));
    lv_obj_set_height(ui_FanBox, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(ui_FanBox, 0);
    lv_obj_set_y(ui_FanBox, lv_pct(-5));
    lv_obj_set_align(ui_FanBox, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_flex_flow(ui_FanBox, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_FanBox, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_FanBox, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_FanBox, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_FanBox, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FanPowerText = lv_label_create(ui_FanBox);
    lv_obj_set_width(ui_FanPowerText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_FanPowerText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_FanPowerText, LV_ALIGN_CENTER);
    lv_label_set_text(ui_FanPowerText, "Fan:");
    lv_obj_set_style_text_color(ui_FanPowerText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_FanPowerText, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_FanPowerText, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_FanPowerText, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_FanPowerText, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_FanPowerText, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FanPower = lv_label_create(ui_FanBox);
    lv_obj_set_width(ui_FanPower, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_FanPower, LV_SIZE_CONTENT);    /// 1
    lv_label_set_text(ui_FanPower, "0%");
    lv_obj_set_style_text_color(ui_FanPower, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_FanPower, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_FanPower, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_FanPower, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_FanPower, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_FanPower, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FanPowerArc = lv_arc_create(ui_Main);
    lv_obj_set_width(ui_FanPowerArc, lv_pct(100));
    lv_obj_set_height(ui_FanPowerArc, lv_pct(100));
    lv_obj_set_align(ui_FanPowerArc, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_FanPowerArc, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_set_scroll_dir(ui_FanPowerArc, LV_DIR_TOP);
    lv_arc_set_range(ui_FanPowerArc, 0, 200);
    lv_arc_set_bg_angles(ui_FanPowerArc, 115, 65);
    lv_obj_set_style_arc_width(ui_FanPowerArc, 4, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_FanPowerArc, lv_color_hex(0x00FFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_FanPowerArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_FanPowerArc, 4, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(ui_FanPowerArc, lv_color_hex(0xFFFF00), LV_PART_INDICATOR | LV_STATE_PRESSED);
    lv_obj_set_style_arc_opa(ui_FanPowerArc, 255, LV_PART_INDICATOR | LV_STATE_PRESSED);

    lv_obj_set_style_bg_color(ui_FanPowerArc, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_FanPowerArc, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_FanPowerArc, lv_color_hex(0xFFFF00), LV_PART_KNOB | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_FanPowerArc, 255, LV_PART_KNOB | LV_STATE_PRESSED);

}
