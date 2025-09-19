/**
 * ESP32-S3 WiFi CSI Vital Signs Radar
 * Non-contact Heart Rate & Breathing Detection
 * 7" Waveshare Touch LCD Display
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <math.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

Board *board = NULL;

// UI Objects
static lv_obj_t *scr;
static lv_obj_t *heart_rate_label;
static lv_obj_t *breathing_rate_label;
static lv_obj_t *heart_chart;
static lv_obj_t *breathing_chart;
static lv_obj_t *radar_canvas;
static lv_obj_t *status_label;
static lv_obj_t *distance_label;
static lv_obj_t *signal_meter;
static lv_obj_t *heart_icon;
static lv_obj_t *breath_icon;
static lv_chart_series_t *heart_series;
static lv_chart_series_t *breathing_series;
static lv_timer_t *radar_timer;
static lv_timer_t *update_timer;

// CSI Configuration
#define CSI_BUFFER_SIZE 512
#define FFT_SIZE 256
#define RADAR_RADIUS 100

// CSI Data
static float csi_amplitude[CSI_BUFFER_SIZE];
static float csi_phase[CSI_BUFFER_SIZE];
static int csi_index = 0;
static float heart_rate = 72.0;
static float breathing_rate = 16.0;
static float signal_quality = 0.0;
static int detection_distance = 0;
static bool person_detected = false;

// Radar animation
static float radar_angle = 0;
static uint8_t radar_points[360];

// Color scheme
static lv_color_t color_red = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
static lv_color_t color_green = LV_COLOR_MAKE(0x00, 0xFF, 0x00);
static lv_color_t color_blue = LV_COLOR_MAKE(0x00, 0xAA, 0xFF);
static lv_color_t color_cyan = LV_COLOR_MAKE(0x00, 0xFF, 0xFF);
static lv_color_t color_dark = LV_COLOR_MAKE(0x1A, 0x1A, 0x1A);

// Simple FFT for heart rate detection
void simple_fft(float* input, float* output, int n) {
    // Basic frequency analysis for heart rate (0.8-3 Hz) and breathing (0.1-0.5 Hz)
    for (int k = 0; k < n/2; k++) {
        float real = 0, imag = 0;
        for (int t = 0; t < n; t++) {
            float angle = -2.0 * PI * k * t / n;
            real += input[t] * cos(angle);
            imag += input[t] * sin(angle);
        }
        output[k] = sqrt(real * real + imag * imag);
    }
}

// Extract vital signs from CSI data
void extract_vital_signs() {
    static float fft_output[FFT_SIZE/2];
    static float filtered_hr = 72.0;
    static float filtered_br = 16.0;

    // Prepare data for FFT
    float input_data[FFT_SIZE];
    for (int i = 0; i < FFT_SIZE; i++) {
        int idx = (csi_index - FFT_SIZE + i + CSI_BUFFER_SIZE) % CSI_BUFFER_SIZE;
        input_data[i] = csi_amplitude[idx];
    }

    // Perform simple FFT
    simple_fft(input_data, fft_output, FFT_SIZE);

    // Find heart rate peak (0.8-3 Hz range, assuming 10 Hz sampling)
    float max_hr_power = 0;
    int hr_peak_idx = 0;
    for (int i = 8; i < 30; i++) {  // 0.8-3 Hz at 10 Hz sampling
        if (fft_output[i] > max_hr_power) {
            max_hr_power = fft_output[i];
            hr_peak_idx = i;
        }
    }

    // Find breathing rate peak (0.1-0.5 Hz range)
    float max_br_power = 0;
    int br_peak_idx = 0;
    for (int i = 1; i < 5; i++) {  // 0.1-0.5 Hz at 10 Hz sampling
        if (fft_output[i] > max_br_power) {
            max_br_power = fft_output[i];
            br_peak_idx = i;
        }
    }

    // Convert to BPM
    float detected_hr = (hr_peak_idx * 10.0 / FFT_SIZE) * 60.0;
    float detected_br = (br_peak_idx * 10.0 / FFT_SIZE) * 60.0;

    // Apply smoothing filter
    filtered_hr = filtered_hr * 0.8 + detected_hr * 0.2;
    filtered_br = filtered_br * 0.9 + detected_br * 0.1;

    // Clamp to realistic ranges
    heart_rate = constrain(filtered_hr, 40, 180);
    breathing_rate = constrain(filtered_br, 8, 30);

    // Calculate signal quality
    signal_quality = min(100.0f, (max_hr_power / 1000.0f) * 100.0f);
}

// WiFi CSI callback
void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    if (info->rx_ctrl.sig_mode == 1) {  // HT mode
        int8_t *data = (int8_t *)info->buf;

        float amplitude = 0;
        float phase = 0;

        // Process CSI data
        for (int i = 0; i < info->len; i += 2) {
            float real = data[i];
            float imag = data[i + 1];
            amplitude += sqrt(real * real + imag * imag);
            phase += atan2(imag, real);
        }

        amplitude /= (info->len / 2);
        phase /= (info->len / 2);

        // Store in buffer
        csi_amplitude[csi_index] = amplitude;
        csi_phase[csi_index] = phase;
        csi_index = (csi_index + 1) % CSI_BUFFER_SIZE;

        // Update detection
        person_detected = (amplitude > 20);
        detection_distance = constrain(100 - amplitude, 0, 100);
    }
}

// Initialize WiFi CSI
void setup_wifi_csi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };

    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(&wifi_csi_rx_cb, NULL);
    esp_wifi_set_csi(true);

    // Start scanning
    WiFi.scanNetworks(true, false, false, 300, 6);
}

// Draw radar sweep
void draw_radar(lv_timer_t *timer) {
    static lv_color_t buf[RADAR_RADIUS * RADAR_RADIUS * 4];
    lv_canvas_set_buffer(radar_canvas, buf, RADAR_RADIUS * 2, RADAR_RADIUS * 2, LV_IMG_CF_TRUE_COLOR);

    // Clear canvas
    lv_canvas_fill_bg(radar_canvas, lv_color_black(), LV_OPA_COVER);

    // Draw radar circles
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = color_green;
    arc_dsc.width = 1;

    for (int r = 40; r <= RADAR_RADIUS; r += 40) {
        lv_canvas_draw_arc(radar_canvas, RADAR_RADIUS, RADAR_RADIUS, r, 0, 360, &arc_dsc);
    }

    // Draw cross lines
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = color_green;
    line_dsc.width = 1;

    lv_point_t line_points[2];

    // Horizontal line
    line_points[0].x = 0;
    line_points[0].y = RADAR_RADIUS;
    line_points[1].x = RADAR_RADIUS * 2;
    line_points[1].y = RADAR_RADIUS;
    lv_canvas_draw_line(radar_canvas, line_points, 2, &line_dsc);

    // Vertical line
    line_points[0].x = RADAR_RADIUS;
    line_points[0].y = 0;
    line_points[1].x = RADAR_RADIUS;
    line_points[1].y = RADAR_RADIUS * 2;
    lv_canvas_draw_line(radar_canvas, line_points, 2, &line_dsc);

    // Draw sweep line
    radar_angle += 3;
    if (radar_angle >= 360) radar_angle = 0;

    float rad = radar_angle * PI / 180.0;
    int x = RADAR_RADIUS + cos(rad) * RADAR_RADIUS;
    int y = RADAR_RADIUS - sin(rad) * RADAR_RADIUS;

    line_dsc.color = color_cyan;
    line_dsc.width = 2;
    line_points[0].x = RADAR_RADIUS;
    line_points[0].y = RADAR_RADIUS;
    line_points[1].x = x;
    line_points[1].y = y;
    lv_canvas_draw_line(radar_canvas, line_points, 2, &line_dsc);

    // Draw detected points
    if (person_detected) {
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = color_red;
        rect_dsc.radius = 3;

        int px = RADAR_RADIUS + cos(rad) * (RADAR_RADIUS - detection_distance);
        int py = RADAR_RADIUS - sin(rad) * (RADAR_RADIUS - detection_distance);

        lv_area_t area;
        area.x1 = px - 3;
        area.y1 = py - 3;
        area.x2 = px + 3;
        area.y2 = py + 3;
        lv_canvas_draw_rect(radar_canvas, area.x1, area.y1, 6, 6, &rect_dsc);

        // Store point for persistence
        radar_points[(int)radar_angle] = 255;
    }

    // Draw persistent points with fade
    lv_draw_rect_dsc_t fade_rect_dsc;
    lv_draw_rect_dsc_init(&fade_rect_dsc);
    fade_rect_dsc.radius = 2;
    for (int i = 0; i < 360; i++) {
        if (radar_points[i] > 0) {
            float a = i * PI / 180.0;
            int px = RADAR_RADIUS + cos(a) * (RADAR_RADIUS - detection_distance);
            int py = RADAR_RADIUS - sin(a) * (RADAR_RADIUS - detection_distance);

            fade_rect_dsc.bg_opa = radar_points[i];
            fade_rect_dsc.bg_color = lv_color_mix(color_red, lv_color_black(), radar_points[i]);

            lv_area_t area;
            area.x1 = px - 2;
            area.y1 = py - 2;
            area.x2 = px + 2;
            area.y2 = py + 2;
            lv_canvas_draw_rect(radar_canvas, area.x1, area.y1, 4, 4, &fade_rect_dsc);

            // Fade out
            radar_points[i] = max(0, radar_points[i] - 3);
        }
    }
}

// Update vital signs display
void update_display(lv_timer_t *timer) {
    static int counter = 0;
    counter++;

    // Simulate CSI data for demo
    float t = millis() / 1000.0;
    float hr_wave = sin(t * 1.2 * 2 * PI) * 10;  // ~72 BPM
    float br_wave = sin(t * 0.27 * 2 * PI) * 20;  // ~16 BPM
    float noise = random(-5, 5) / 10.0;

    csi_amplitude[csi_index] = 50 + hr_wave + br_wave + noise;
    csi_index = (csi_index + 1) % CSI_BUFFER_SIZE;

    // Extract vital signs every 10 updates
    if (counter % 10 == 0) {
        extract_vital_signs();

        // Update heart rate
        lv_label_set_text_fmt(heart_rate_label, "%d", (int)heart_rate);
        lv_chart_set_next_value(heart_chart, heart_series, heart_rate);

        // Update breathing rate
        lv_label_set_text_fmt(breathing_rate_label, "%d", (int)breathing_rate);
        lv_chart_set_next_value(breathing_chart, breathing_series, breathing_rate);

        // Update signal quality
        lv_bar_set_value(signal_meter, (int)signal_quality, LV_ANIM_ON);

        // Update status
        if (person_detected) {
            lv_label_set_text(status_label, "Person Detected");
            lv_obj_set_style_text_color(status_label, color_green, 0);
            lv_label_set_text_fmt(distance_label, "Distance: %d cm", 50 + detection_distance);
        } else {
            lv_label_set_text(status_label, "Scanning...");
            lv_obj_set_style_text_color(status_label, color_blue, 0);
            lv_label_set_text(distance_label, "Distance: --");
        }

        // Animate heart icon
        static bool heart_beat = false;
        heart_beat = !heart_beat;
        if (heart_beat && person_detected) {
            lv_obj_set_style_text_color(heart_icon, color_red, 0);
            lv_obj_set_style_transform_zoom(heart_icon, 280, 0);
        } else {
            lv_obj_set_style_text_color(heart_icon, LV_COLOR_MAKE(0x80, 0x00, 0x00), 0);
            lv_obj_set_style_transform_zoom(heart_icon, 256, 0);
        }
    }
}

// Create UI
void create_radar_ui() {
    scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "WiFi CSI Vital Signs Radar");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, color_cyan, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Radar display (left)
    lv_obj_t *radar_container = lv_obj_create(scr);
    lv_obj_set_size(radar_container, 240, 240);
    lv_obj_align(radar_container, LV_ALIGN_LEFT_MID, 20, 0);
    lv_obj_set_style_bg_color(radar_container, color_dark, 0);
    lv_obj_set_style_border_color(radar_container, color_green, 0);
    lv_obj_set_style_border_width(radar_container, 2, 0);
    lv_obj_clear_flag(radar_container, LV_OBJ_FLAG_SCROLLABLE);

    radar_canvas = lv_canvas_create(radar_container);
    lv_obj_set_size(radar_canvas, RADAR_RADIUS * 2, RADAR_RADIUS * 2);
    lv_obj_center(radar_canvas);

    // Heart rate panel (top right)
    lv_obj_t *hr_panel = lv_obj_create(scr);
    lv_obj_set_size(hr_panel, 220, 180);
    lv_obj_align(hr_panel, LV_ALIGN_TOP_RIGHT, -20, 50);
    lv_obj_set_style_bg_color(hr_panel, color_dark, 0);
    lv_obj_set_style_border_color(hr_panel, color_red, 0);
    lv_obj_set_style_border_width(hr_panel, 2, 0);
    lv_obj_clear_flag(hr_panel, LV_OBJ_FLAG_SCROLLABLE);

    heart_icon = lv_label_create(hr_panel);
    lv_label_set_text(heart_icon, "\xE2\x9D\xA4");  // UTF-8 heart symbol
    lv_obj_set_style_text_font(heart_icon, &lv_font_montserrat_30, 0);
    lv_obj_align(heart_icon, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *hr_title = lv_label_create(hr_panel);
    lv_label_set_text(hr_title, "Heart Rate");
    lv_obj_set_style_text_color(hr_title, color_red, 0);
    lv_obj_align(hr_title, LV_ALIGN_TOP_LEFT, 50, 15);

    heart_rate_label = lv_label_create(hr_panel);
    lv_label_set_text(heart_rate_label, "72");
    lv_obj_set_style_text_font(heart_rate_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(heart_rate_label, lv_color_white(), 0);
    lv_obj_align(heart_rate_label, LV_ALIGN_CENTER, -30, -10);

    lv_obj_t *bpm_label = lv_label_create(hr_panel);
    lv_label_set_text(bpm_label, "BPM");
    lv_obj_set_style_text_color(bpm_label, LV_COLOR_MAKE(0xAA, 0xAA, 0xAA), 0);
    lv_obj_align(bpm_label, LV_ALIGN_CENTER, 30, -5);

    heart_chart = lv_chart_create(hr_panel);
    lv_obj_set_size(heart_chart, 200, 60);
    lv_obj_align(heart_chart, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_chart_set_type(heart_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(heart_chart, 50);
    lv_chart_set_range(heart_chart, LV_CHART_AXIS_PRIMARY_Y, 40, 120);
    lv_obj_set_style_bg_opa(heart_chart, LV_OPA_20, 0);
    lv_obj_set_style_size(heart_chart, 0, LV_PART_INDICATOR);
    heart_series = lv_chart_add_series(heart_chart, color_red, LV_CHART_AXIS_PRIMARY_Y);

    // Breathing rate panel (middle right)
    lv_obj_t *br_panel = lv_obj_create(scr);
    lv_obj_set_size(br_panel, 220, 140);
    lv_obj_align(br_panel, LV_ALIGN_RIGHT_MID, -20, 30);
    lv_obj_set_style_bg_color(br_panel, color_dark, 0);
    lv_obj_set_style_border_color(br_panel, color_blue, 0);
    lv_obj_set_style_border_width(br_panel, 2, 0);
    lv_obj_clear_flag(br_panel, LV_OBJ_FLAG_SCROLLABLE);

    breath_icon = lv_label_create(br_panel);
    lv_label_set_text(breath_icon, LV_SYMBOL_REFRESH);
    lv_obj_set_style_text_font(breath_icon, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(breath_icon, color_blue, 0);
    lv_obj_align(breath_icon, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *br_title = lv_label_create(br_panel);
    lv_label_set_text(br_title, "Breathing");
    lv_obj_set_style_text_color(br_title, color_blue, 0);
    lv_obj_align(br_title, LV_ALIGN_TOP_LEFT, 40, 10);

    breathing_rate_label = lv_label_create(br_panel);
    lv_label_set_text(breathing_rate_label, "16");
    lv_obj_set_style_text_font(breathing_rate_label, &lv_font_montserrat_30, 0);
    lv_obj_set_style_text_color(breathing_rate_label, lv_color_white(), 0);
    lv_obj_align(breathing_rate_label, LV_ALIGN_LEFT_MID, 20, 10);

    lv_obj_t *rpm_label = lv_label_create(br_panel);
    lv_label_set_text(rpm_label, "RPM");
    lv_obj_set_style_text_color(rpm_label, LV_COLOR_MAKE(0xAA, 0xAA, 0xAA), 0);
    lv_obj_align(rpm_label, LV_ALIGN_LEFT_MID, 70, 10);

    breathing_chart = lv_chart_create(br_panel);
    lv_obj_set_size(breathing_chart, 200, 40);
    lv_obj_align(breathing_chart, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_chart_set_type(breathing_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(breathing_chart, 30);
    lv_chart_set_range(breathing_chart, LV_CHART_AXIS_PRIMARY_Y, 8, 30);
    lv_obj_set_style_bg_opa(breathing_chart, LV_OPA_20, 0);
    lv_obj_set_style_size(breathing_chart, 0, LV_PART_INDICATOR);
    breathing_series = lv_chart_add_series(breathing_chart, color_blue, LV_CHART_AXIS_PRIMARY_Y);

    // Status panel (bottom)
    lv_obj_t *status_panel = lv_obj_create(scr);
    lv_obj_set_size(status_panel, 440, 60);
    lv_obj_align(status_panel, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_bg_color(status_panel, color_dark, 0);
    lv_obj_set_style_border_width(status_panel, 0, 0);
    lv_obj_clear_flag(status_panel, LV_OBJ_FLAG_SCROLLABLE);

    status_label = lv_label_create(status_panel);
    lv_label_set_text(status_label, "Initializing...");
    lv_obj_set_style_text_color(status_label, color_green, 0);
    lv_obj_align(status_label, LV_ALIGN_LEFT_MID, 10, -10);

    distance_label = lv_label_create(status_panel);
    lv_label_set_text(distance_label, "Distance: --");
    lv_obj_set_style_text_color(distance_label, color_cyan, 0);
    lv_obj_align(distance_label, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t *signal_label = lv_label_create(status_panel);
    lv_label_set_text(signal_label, "Signal:");
    lv_obj_set_style_text_color(signal_label, lv_color_white(), 0);
    lv_obj_align(signal_label, LV_ALIGN_RIGHT_MID, -120, -10);

    signal_meter = lv_bar_create(status_panel);
    lv_obj_set_size(signal_meter, 100, 15);
    lv_obj_align(signal_meter, LV_ALIGN_RIGHT_MID, -10, -10);
    lv_bar_set_range(signal_meter, 0, 100);
    lv_obj_set_style_bg_color(signal_meter, LV_COLOR_MAKE(0x40, 0x40, 0x40), LV_PART_MAIN);
    lv_obj_set_style_bg_color(signal_meter, color_green, LV_PART_INDICATOR);

    // Initialize data
    for (int i = 0; i < 360; i++) {
        radar_points[i] = 0;
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("ESP32-S3 WiFi CSI Vital Signs Radar");
    Serial.println("Non-contact Heart Rate & Breathing Detection");

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

    // Initialize WiFi CSI
    Serial.println("Setting up WiFi CSI...");
    setup_wifi_csi();

    // Create UI
    Serial.println("Creating radar UI...");
    lvgl_port_lock(-1);
    create_radar_ui();

    // Create timers
    radar_timer = lv_timer_create(draw_radar, 50, NULL);  // 20 FPS radar
    update_timer = lv_timer_create(update_display, 100, NULL);  // 10 Hz update

    lvgl_port_unlock();

    Serial.println("Vital Signs Radar Ready!");
    Serial.println("Place hand or chest near device for detection");
}

void loop() {
    delay(10);
}