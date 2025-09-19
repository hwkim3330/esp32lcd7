/**
 * Tesla-Style Dashboard with Touch Controls
 * ESP32-S3 7" Waveshare Touch LCD
 * Inspired by VelocityDRIVE CT & Tesla Model S/3
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

Board *board = NULL;

// UI Objects - Main Dashboard
static lv_obj_t *scr;
static lv_obj_t *speed_gauge;
static lv_obj_t *speed_label;
static lv_obj_t *speed_unit_label;
static lv_obj_t *power_meter;
static lv_obj_t *battery_container;
static lv_obj_t *battery_bar;
static lv_obj_t *battery_label;
static lv_obj_t *range_label;
static lv_obj_t *gear_selector;
static lv_obj_t *drive_mode_label;

// Touch Control Panels
static lv_obj_t *left_panel;
static lv_obj_t *center_panel;
static lv_obj_t *right_panel;
static lv_obj_t *bottom_panel;

// Touch Controls
static lv_obj_t *climate_btn;
static lv_obj_t *media_btn;
static lv_obj_t *nav_btn;
static lv_obj_t *phone_btn;
static lv_obj_t *settings_btn;
static lv_obj_t *autopilot_btn;
static lv_obj_t *park_btn;
static lv_obj_t *reverse_btn;
static lv_obj_t *neutral_btn;
static lv_obj_t *drive_btn;
static lv_obj_t *temp_slider;
static lv_obj_t *volume_slider;
static lv_obj_t *brightness_slider;

// Status Indicators
static lv_obj_t *left_turn_signal;
static lv_obj_t *right_turn_signal;
static lv_obj_t *headlight_indicator;
static lv_obj_t *parking_brake_indicator;
static lv_obj_t *door_status[4];
static lv_obj_t *tire_pressure[4];
static lv_obj_t *temp_label;
static lv_obj_t *time_label;

// Charts and Visualizations
static lv_obj_t *power_chart;
static lv_obj_t *efficiency_chart;
static lv_obj_t *nav_map;
static lv_chart_series_t *power_series;
static lv_chart_series_t *regen_series;
static lv_chart_series_t *efficiency_series;

// Animation timers
static lv_timer_t *update_timer;
static lv_timer_t *animation_timer;

// Vehicle State
typedef struct {
    int speed;
    int target_speed;
    int power;
    int battery_percent;
    int range_km;
    char gear; // P, R, N, D
    bool autopilot_enabled;
    bool left_signal;
    bool right_signal;
    int temp_inside;
    int temp_outside;
    bool doors_locked;
    bool lights_on;
    float tire_pressure_psi[4];
    int volume;
    int brightness;
    int climate_temp;
} VehicleState;

static VehicleState vehicle = {
    .speed = 0,
    .target_speed = 0,
    .power = 0,
    .battery_percent = 85,
    .range_km = 380,
    .gear = 'P',
    .autopilot_enabled = false,
    .left_signal = false,
    .right_signal = false,
    .temp_inside = 22,
    .temp_outside = 18,
    .doors_locked = true,
    .lights_on = false,
    .tire_pressure_psi = {32.5, 32.5, 32.0, 32.0},
    .volume = 50,
    .brightness = 70,
    .climate_temp = 22
};

// Color scheme - Tesla inspired
static lv_color_t color_tesla_red = LV_COLOR_MAKE(0xE3, 0x1C, 0x1C);
static lv_color_t color_tesla_blue = LV_COLOR_MAKE(0x3D, 0x6C, 0xCE);
static lv_color_t color_white = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
static lv_color_t color_gray = LV_COLOR_MAKE(0x4A, 0x4A, 0x4A);
static lv_color_t color_dark = LV_COLOR_MAKE(0x1A, 0x1A, 0x1A);
static lv_color_t color_green = LV_COLOR_MAKE(0x4A, 0xD3, 0x4A);
static lv_color_t color_orange = LV_COLOR_MAKE(0xFF, 0xA5, 0x00);

// Touch event handlers
static void gear_select_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    const char *txt = lv_label_get_text(lv_obj_get_child(btn, 0));
    vehicle.gear = txt[0];

    // Update visual feedback
    lv_obj_set_style_bg_color(park_btn, vehicle.gear == 'P' ? color_tesla_blue : color_gray, 0);
    lv_obj_set_style_bg_color(reverse_btn, vehicle.gear == 'R' ? color_tesla_blue : color_gray, 0);
    lv_obj_set_style_bg_color(neutral_btn, vehicle.gear == 'N' ? color_tesla_blue : color_gray, 0);
    lv_obj_set_style_bg_color(drive_btn, vehicle.gear == 'D' ? color_tesla_blue : color_gray, 0);

    lv_label_set_text_fmt(gear_selector, "%c", vehicle.gear);
}

static void autopilot_event_cb(lv_event_t *e) {
    vehicle.autopilot_enabled = !vehicle.autopilot_enabled;
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn,
        vehicle.autopilot_enabled ? color_tesla_blue : color_gray, 0);
    lv_label_set_text(lv_obj_get_child(btn, 0),
        vehicle.autopilot_enabled ? "AUTOPILOT ON" : "AUTOPILOT OFF");
}

static void climate_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    vehicle.climate_temp = lv_slider_get_value(slider);
    lv_label_set_text_fmt(temp_label, "%d°C", vehicle.climate_temp);
}

static void volume_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    vehicle.volume = lv_slider_get_value(slider);
}

static void brightness_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    vehicle.brightness = lv_slider_get_value(slider);
    // In real implementation, this would control display brightness
}

static void turn_signal_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    if (btn == left_turn_signal) {
        vehicle.left_signal = !vehicle.left_signal;
        vehicle.right_signal = false;
    } else {
        vehicle.right_signal = !vehicle.right_signal;
        vehicle.left_signal = false;
    }
}

// Update dashboard display
static void update_dashboard(lv_timer_t *timer) {
    static int counter = 0;
    counter++;

    // Simulate speed changes
    if (vehicle.gear == 'D') {
        if (vehicle.speed < 120) vehicle.speed += random(0, 5);
    } else if (vehicle.gear == 'R') {
        if (vehicle.speed > -20) vehicle.speed -= 1;
    } else {
        if (vehicle.speed > 0) vehicle.speed -= 2;
        if (vehicle.speed < 0) vehicle.speed += 1;
    }

    // Update speed display
    lv_label_set_text_fmt(speed_label, "%d", abs(vehicle.speed));

    // Update power meter (simulate based on speed change)
    vehicle.power = random(-50, 150);
    lv_bar_set_value(power_meter, 100 + vehicle.power, LV_ANIM_ON);

    // Update battery (slow drain simulation)
    if (counter % 100 == 0 && vehicle.battery_percent > 20) {
        vehicle.battery_percent--;
        vehicle.range_km = vehicle.battery_percent * 4.5;
    }

    lv_bar_set_value(battery_bar, vehicle.battery_percent, LV_ANIM_ON);
    lv_label_set_text_fmt(battery_label, "%d%%", vehicle.battery_percent);
    lv_label_set_text_fmt(range_label, "%d km", vehicle.range_km);

    // Update charts
    lv_chart_set_next_value(power_chart, power_series, vehicle.power);
    lv_chart_set_next_value(power_chart, regen_series, vehicle.power < 0 ? -vehicle.power : 0);

    // Turn signal blinking
    if (counter % 10 == 0) {
        if (vehicle.left_signal) {
            static bool blink = false;
            blink = !blink;
            lv_obj_set_style_bg_opa(left_turn_signal, blink ? LV_OPA_100 : LV_OPA_30, 0);
        } else {
            lv_obj_set_style_bg_opa(left_turn_signal, LV_OPA_30, 0);
        }

        if (vehicle.right_signal) {
            static bool blink = false;
            blink = !blink;
            lv_obj_set_style_bg_opa(right_turn_signal, blink ? LV_OPA_100 : LV_OPA_30, 0);
        } else {
            lv_obj_set_style_bg_opa(right_turn_signal, LV_OPA_30, 0);
        }
    }

    // Update time
    unsigned long seconds = millis() / 1000;
    int hours = (seconds / 3600) % 24;
    int minutes = (seconds / 60) % 60;
    lv_label_set_text_fmt(time_label, "%02d:%02d", hours, minutes);
}

// Create Tesla-style dashboard UI
void create_tesla_dashboard() {
    scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, color_dark, 0);

    // Top status bar
    lv_obj_t *top_bar = lv_obj_create(scr);
    lv_obj_set_size(top_bar, 800, 40);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_bar, LV_COLOR_MAKE(0x10, 0x10, 0x10), 0);
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);

    // Tesla logo/brand
    lv_obj_t *brand = lv_label_create(top_bar);
    lv_label_set_text(brand, "TESLA");
    lv_obj_set_style_text_font(brand, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(brand, color_tesla_red, 0);
    lv_obj_align(brand, LV_ALIGN_LEFT_MID, 10, 0);

    // Time display
    time_label = lv_label_create(top_bar);
    lv_label_set_text(time_label, "00:00");
    lv_obj_set_style_text_color(time_label, color_white, 0);
    lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 0);

    // Temperature
    temp_label = lv_label_create(top_bar);
    lv_label_set_text_fmt(temp_label, "%d°C", vehicle.temp_outside);
    lv_obj_set_style_text_color(temp_label, color_white, 0);
    lv_obj_align(temp_label, LV_ALIGN_RIGHT_MID, -10, 0);

    // Main dashboard container
    lv_obj_t *main_container = lv_obj_create(scr);
    lv_obj_set_size(main_container, 800, 340);
    lv_obj_align(main_container, LV_ALIGN_TOP_MID, 0, 45);
    lv_obj_set_style_bg_color(main_container, color_dark, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

    // Center speed display
    center_panel = lv_obj_create(main_container);
    lv_obj_set_size(center_panel, 300, 280);
    lv_obj_align(center_panel, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_bg_color(center_panel, LV_COLOR_MAKE(0x15, 0x15, 0x15), 0);
    lv_obj_set_style_border_width(center_panel, 2, 0);
    lv_obj_set_style_border_color(center_panel, color_gray, 0);
    lv_obj_set_style_radius(center_panel, 15, 0);
    lv_obj_clear_flag(center_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Speed display
    speed_label = lv_label_create(center_panel);
    lv_label_set_text(speed_label, "0");
    lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(speed_label, color_white, 0);
    lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -20);

    speed_unit_label = lv_label_create(center_panel);
    lv_label_set_text(speed_unit_label, "km/h");
    lv_obj_set_style_text_color(speed_unit_label, color_gray, 0);
    lv_obj_align(speed_unit_label, LV_ALIGN_CENTER, 0, 20);

    // Gear selector display
    gear_selector = lv_label_create(center_panel);
    lv_label_set_text_fmt(gear_selector, "%c", vehicle.gear);
    lv_obj_set_style_text_font(gear_selector, &lv_font_montserrat_30, 0);
    lv_obj_set_style_text_color(gear_selector, color_tesla_blue, 0);
    lv_obj_align(gear_selector, LV_ALIGN_TOP_MID, 0, 10);

    // Autopilot indicator
    lv_obj_t *autopilot_status = lv_obj_create(center_panel);
    lv_obj_set_size(autopilot_status, 120, 30);
    lv_obj_align(autopilot_status, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(autopilot_status, color_gray, 0);
    lv_obj_set_style_radius(autopilot_status, 15, 0);
    lv_obj_clear_flag(autopilot_status, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ap_label = lv_label_create(autopilot_status);
    lv_label_set_text(ap_label, "AUTOPILOT");
    lv_obj_set_style_text_font(ap_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ap_label, color_white, 0);
    lv_obj_center(ap_label);

    // Left panel - Power meter
    left_panel = lv_obj_create(main_container);
    lv_obj_set_size(left_panel, 200, 280);
    lv_obj_align(left_panel, LV_ALIGN_LEFT_MID, 10, -20);
    lv_obj_set_style_bg_color(left_panel, LV_COLOR_MAKE(0x15, 0x15, 0x15), 0);
    lv_obj_set_style_border_width(left_panel, 2, 0);
    lv_obj_set_style_border_color(left_panel, color_gray, 0);
    lv_obj_set_style_radius(left_panel, 15, 0);
    lv_obj_clear_flag(left_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *power_title = lv_label_create(left_panel);
    lv_label_set_text(power_title, "POWER");
    lv_obj_set_style_text_color(power_title, color_white, 0);
    lv_obj_align(power_title, LV_ALIGN_TOP_MID, 0, 10);

    // Power meter bar
    power_meter = lv_bar_create(left_panel);
    lv_obj_set_size(power_meter, 160, 30);
    lv_obj_align(power_meter, LV_ALIGN_TOP_MID, 0, 40);
    lv_bar_set_range(power_meter, 0, 200);
    lv_bar_set_value(power_meter, 100, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(power_meter, color_gray, LV_PART_MAIN);
    lv_obj_set_style_bg_color(power_meter, color_green, LV_PART_INDICATOR);

    // Power chart
    power_chart = lv_chart_create(left_panel);
    lv_obj_set_size(power_chart, 180, 100);
    lv_obj_align(power_chart, LV_ALIGN_CENTER, 0, 20);
    lv_chart_set_type(power_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(power_chart, 50);
    lv_chart_set_range(power_chart, LV_CHART_AXIS_PRIMARY_Y, -100, 200);
    lv_obj_set_style_bg_opa(power_chart, LV_OPA_20, 0);

    power_series = lv_chart_add_series(power_chart, color_green, LV_CHART_AXIS_PRIMARY_Y);
    regen_series = lv_chart_add_series(power_chart, color_orange, LV_CHART_AXIS_PRIMARY_Y);

    // Turn signals
    left_turn_signal = lv_btn_create(left_panel);
    lv_obj_set_size(left_turn_signal, 40, 40);
    lv_obj_align(left_turn_signal, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(left_turn_signal, color_green, 0);
    lv_obj_add_event_cb(left_turn_signal, turn_signal_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *left_arrow = lv_label_create(left_turn_signal);
    lv_label_set_text(left_arrow, LV_SYMBOL_LEFT);
    lv_obj_center(left_arrow);

    // Right panel - Battery & Range
    right_panel = lv_obj_create(main_container);
    lv_obj_set_size(right_panel, 200, 280);
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, -10, -20);
    lv_obj_set_style_bg_color(right_panel, LV_COLOR_MAKE(0x15, 0x15, 0x15), 0);
    lv_obj_set_style_border_width(right_panel, 2, 0);
    lv_obj_set_style_border_color(right_panel, color_gray, 0);
    lv_obj_set_style_radius(right_panel, 15, 0);
    lv_obj_clear_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *battery_title = lv_label_create(right_panel);
    lv_label_set_text(battery_title, "BATTERY");
    lv_obj_set_style_text_color(battery_title, color_white, 0);
    lv_obj_align(battery_title, LV_ALIGN_TOP_MID, 0, 10);

    // Battery visualization
    battery_container = lv_obj_create(right_panel);
    lv_obj_set_size(battery_container, 160, 80);
    lv_obj_align(battery_container, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_bg_color(battery_container, color_gray, 0);
    lv_obj_set_style_radius(battery_container, 10, 0);
    lv_obj_clear_flag(battery_container, LV_OBJ_FLAG_SCROLLABLE);

    battery_bar = lv_bar_create(battery_container);
    lv_obj_set_size(battery_bar, 140, 60);
    lv_obj_center(battery_bar);
    lv_bar_set_range(battery_bar, 0, 100);
    lv_bar_set_value(battery_bar, vehicle.battery_percent, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(battery_bar, LV_COLOR_MAKE(0x30, 0x30, 0x30), LV_PART_MAIN);
    lv_obj_set_style_bg_color(battery_bar, color_green, LV_PART_INDICATOR);

    battery_label = lv_label_create(battery_container);
    lv_label_set_text_fmt(battery_label, "%d%%", vehicle.battery_percent);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(battery_label, color_white, 0);
    lv_obj_center(battery_label);

    // Range display
    lv_obj_t *range_title = lv_label_create(right_panel);
    lv_label_set_text(range_title, "EST. RANGE");
    lv_obj_set_style_text_color(range_title, color_gray, 0);
    lv_obj_align(range_title, LV_ALIGN_TOP_MID, 0, 130);

    range_label = lv_label_create(right_panel);
    lv_label_set_text_fmt(range_label, "%d km", vehicle.range_km);
    lv_obj_set_style_text_font(range_label, &lv_font_montserrat_30, 0);
    lv_obj_set_style_text_color(range_label, color_white, 0);
    lv_obj_align(range_label, LV_ALIGN_TOP_MID, 0, 150);

    // Turn signal right
    right_turn_signal = lv_btn_create(right_panel);
    lv_obj_set_size(right_turn_signal, 40, 40);
    lv_obj_align(right_turn_signal, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(right_turn_signal, color_green, 0);
    lv_obj_add_event_cb(right_turn_signal, turn_signal_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *right_arrow = lv_label_create(right_turn_signal);
    lv_label_set_text(right_arrow, LV_SYMBOL_RIGHT);
    lv_obj_center(right_arrow);

    // Bottom control panel with touch buttons
    bottom_panel = lv_obj_create(scr);
    lv_obj_set_size(bottom_panel, 800, 90);
    lv_obj_align(bottom_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bottom_panel, LV_COLOR_MAKE(0x20, 0x20, 0x20), 0);
    lv_obj_set_style_border_width(bottom_panel, 0, 0);
    lv_obj_clear_flag(bottom_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Gear selector buttons
    park_btn = lv_btn_create(bottom_panel);
    lv_obj_set_size(park_btn, 60, 60);
    lv_obj_align(park_btn, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_set_style_bg_color(park_btn, vehicle.gear == 'P' ? color_tesla_blue : color_gray, 0);
    lv_obj_add_event_cb(park_btn, gear_select_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *p_label = lv_label_create(park_btn);
    lv_label_set_text(p_label, "P");
    lv_obj_set_style_text_font(p_label, &lv_font_montserrat_20, 0);
    lv_obj_center(p_label);

    reverse_btn = lv_btn_create(bottom_panel);
    lv_obj_set_size(reverse_btn, 60, 60);
    lv_obj_align(reverse_btn, LV_ALIGN_LEFT_MID, 90, 0);
    lv_obj_set_style_bg_color(reverse_btn, color_gray, 0);
    lv_obj_add_event_cb(reverse_btn, gear_select_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *r_label = lv_label_create(reverse_btn);
    lv_label_set_text(r_label, "R");
    lv_obj_set_style_text_font(r_label, &lv_font_montserrat_20, 0);
    lv_obj_center(r_label);

    neutral_btn = lv_btn_create(bottom_panel);
    lv_obj_set_size(neutral_btn, 60, 60);
    lv_obj_align(neutral_btn, LV_ALIGN_LEFT_MID, 160, 0);
    lv_obj_set_style_bg_color(neutral_btn, color_gray, 0);
    lv_obj_add_event_cb(neutral_btn, gear_select_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *n_label = lv_label_create(neutral_btn);
    lv_label_set_text(n_label, "N");
    lv_obj_set_style_text_font(n_label, &lv_font_montserrat_20, 0);
    lv_obj_center(n_label);

    drive_btn = lv_btn_create(bottom_panel);
    lv_obj_set_size(drive_btn, 60, 60);
    lv_obj_align(drive_btn, LV_ALIGN_LEFT_MID, 230, 0);
    lv_obj_set_style_bg_color(drive_btn, color_gray, 0);
    lv_obj_add_event_cb(drive_btn, gear_select_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *d_label = lv_label_create(drive_btn);
    lv_label_set_text(d_label, "D");
    lv_obj_set_style_text_font(d_label, &lv_font_montserrat_20, 0);
    lv_obj_center(d_label);

    // Autopilot button
    autopilot_btn = lv_btn_create(bottom_panel);
    lv_obj_set_size(autopilot_btn, 140, 60);
    lv_obj_align(autopilot_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(autopilot_btn, color_gray, 0);
    lv_obj_add_event_cb(autopilot_btn, autopilot_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *ap_btn_label = lv_label_create(autopilot_btn);
    lv_label_set_text(ap_btn_label, "AUTOPILOT OFF");
    lv_obj_center(ap_btn_label);

    // Climate control slider
    temp_slider = lv_slider_create(bottom_panel);
    lv_obj_set_size(temp_slider, 120, 20);
    lv_obj_align(temp_slider, LV_ALIGN_RIGHT_MID, -150, -20);
    lv_slider_set_range(temp_slider, 16, 28);
    lv_slider_set_value(temp_slider, vehicle.climate_temp, LV_ANIM_OFF);
    lv_obj_add_event_cb(temp_slider, climate_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *climate_label = lv_label_create(bottom_panel);
    lv_label_set_text(climate_label, "CLIMATE");
    lv_obj_set_style_text_font(climate_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(climate_label, color_gray, 0);
    lv_obj_align(climate_label, LV_ALIGN_RIGHT_MID, -180, 10);

    // Volume slider
    volume_slider = lv_slider_create(bottom_panel);
    lv_obj_set_size(volume_slider, 100, 20);
    lv_obj_align(volume_slider, LV_ALIGN_RIGHT_MID, -20, -20);
    lv_slider_set_range(volume_slider, 0, 100);
    lv_slider_set_value(volume_slider, vehicle.volume, LV_ANIM_OFF);
    lv_obj_add_event_cb(volume_slider, volume_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *volume_label = lv_label_create(bottom_panel);
    lv_label_set_text(volume_label, "VOLUME");
    lv_obj_set_style_text_font(volume_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(volume_label, color_gray, 0);
    lv_obj_align(volume_label, LV_ALIGN_RIGHT_MID, -40, 10);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("Tesla-Style Dashboard with Touch Controls");
    Serial.println("ESP32-S3 7\" Waveshare Touch LCD");

    // Initialize board
    Serial.println("Initializing display board...");
    board = new Board();
    board->init();

    if (!board->begin()) {
        Serial.println("Board begin failed!");
    }

    // Initialize LVGL
    Serial.println("Initializing LVGL...");
    lvgl_port_init(board->getLCD(), board->getTouch());

    // Create dashboard UI
    Serial.println("Creating Tesla dashboard UI...");
    lvgl_port_lock(-1);
    create_tesla_dashboard();
    lvgl_port_unlock();

    // Create update timer
    update_timer = lv_timer_create(update_dashboard, 100, NULL);

    Serial.println("Dashboard ready! Touch controls enabled.");
}

void loop() {
    delay(5);
}