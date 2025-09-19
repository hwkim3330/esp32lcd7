/**
 * Advanced WiFi LiDAR Visualization System
 * Real-time WiFi Environment Mapping & Analysis
 * ESP32-S3 7" Touch LCD - Professional Grade
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <math.h>
#include <map>
#include <vector>
#include <algorithm>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

Board *board = NULL;

// Constants
#define MAX_NETWORKS 32
#define MAX_STATIONS 64
#define SCAN_INTERVAL 500
#define HISTORY_SIZE 50
#define RADAR_RADIUS 150
#define SPECTRUM_WIDTH 460
#define SPECTRUM_HEIGHT 80
#define CHANNEL_WIDTH 20
#define MAX_CHANNELS 14

// WiFi AP Information Structure
struct WiFiAP {
    String ssid;
    String bssid;
    int32_t rssi;
    int channel;
    wifi_auth_mode_t encryption;
    float distance;  // Calculated distance in meters
    float angle;     // Position angle for visualization
    float x, y;      // Calculated position
    uint32_t lastSeen;
    int history[HISTORY_SIZE];
    int historyIndex;
    float avgRssi;
    float variance;
    bool isMoving;
    float velocity;
};

// WiFi Station Information
struct WiFiStation {
    uint8_t mac[6];
    int32_t rssi;
    float distance;
    uint32_t lastSeen;
    int channel;
    bool isActive;
};

// Channel Information
struct ChannelInfo {
    int apCount;
    int stationCount;
    float utilization;
    float interference;
    int maxRssi;
    int minRssi;
};

// Global WiFi Data
static std::map<String, WiFiAP> wifiNetworks;
static std::vector<WiFiStation> wifiStations;
static ChannelInfo channels[MAX_CHANNELS + 1];
static int currentChannel = 1;
static bool isScanning = false;
static bool promiscuousMode = false;

// UI Objects - Main Display
static lv_obj_t *scr;
static lv_obj_t *radar_canvas;
static lv_obj_t *spectrum_canvas;
static lv_obj_t *heatmap_canvas;
static lv_obj_t *info_panel;
static lv_obj_t *stats_panel;
static lv_obj_t *control_panel;

// UI Objects - Information Labels
static lv_obj_t *network_count_label;
static lv_obj_t *station_count_label;
static lv_obj_t *channel_label;
static lv_obj_t *strongest_label;
static lv_obj_t *weakest_label;
static lv_obj_t *interference_label;
static lv_obj_t *scan_status_label;
static lv_obj_t *mode_label;

// UI Objects - Network List
static lv_obj_t *network_list;
static lv_obj_t *network_chart;
static lv_chart_series_t *rssi_series[5];

// UI Objects - Controls
static lv_obj_t *scan_btn;
static lv_obj_t *promiscuous_btn;
static lv_obj_t *channel_slider;
static lv_obj_t *range_slider;
static lv_obj_t *sensitivity_slider;
static lv_obj_t *view_selector;

// Display Settings
static int viewMode = 0;  // 0: Radar, 1: Spectrum, 2: Heatmap, 3: 3D
static float scanRange = 100.0;  // meters
static float sensitivity = -90.0;  // dBm threshold
static float radarAngle = 0;
static uint8_t radarTrail[360][RADAR_RADIUS];

// Animation & Timers
static lv_timer_t *scan_timer;
static lv_timer_t *update_timer;
static lv_timer_t *animation_timer;

// Color Scheme
static lv_color_t color_green = LV_COLOR_MAKE(0x00, 0xFF, 0x00);
static lv_color_t color_red = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
static lv_color_t color_blue = LV_COLOR_MAKE(0x00, 0x88, 0xFF);
static lv_color_t color_yellow = LV_COLOR_MAKE(0xFF, 0xFF, 0x00);
static lv_color_t color_cyan = LV_COLOR_MAKE(0x00, 0xFF, 0xFF);
static lv_color_t color_purple = LV_COLOR_MAKE(0xFF, 0x00, 0xFF);
static lv_color_t color_orange = LV_COLOR_MAKE(0xFF, 0xA5, 0x00);
static lv_color_t color_dark = LV_COLOR_MAKE(0x0A, 0x0A, 0x0A);

// Calculate distance from RSSI using path loss formula
float calculateDistance(int32_t rssi, int frequency = 2437) {
    // Path Loss = 20*log10(d) + 20*log10(f) + 32.44
    // RSSI = -Path Loss
    // Solving for d: d = 10^((abs(RSSI) - 32.44 - 20*log10(f)) / 20)

    float pathLoss = abs(rssi);
    float freqMHz = frequency;
    float exp = (pathLoss - 32.44 - 20.0 * log10(freqMHz)) / 20.0;
    float distance = pow(10.0, exp);

    // Apply correction factor based on environment
    distance *= 0.5;  // Indoor environment factor

    return constrain(distance, 0.1, scanRange);
}

// Convert channel to frequency
int channelToFrequency(int channel) {
    if (channel >= 1 && channel <= 14) {
        if (channel < 14) {
            return 2412 + (channel - 1) * 5;
        } else {
            return 2484;  // Channel 14
        }
    }
    // 5GHz band
    if (channel >= 36 && channel <= 165) {
        return 5180 + (channel - 36) * 5;
    }
    return 2437;  // Default
}

// WiFi Promiscuous Mode Callback
void IRAM_ATTR promiscuous_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (!promiscuousMode) return;

    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    wifi_pkt_rx_ctrl_t* rx_ctrl = &pkt->rx_ctrl;

    // Extract MAC addresses
    uint8_t* data = pkt->payload;
    uint8_t* addr1 = data + 4;   // Destination
    uint8_t* addr2 = data + 10;  // Source
    uint8_t* addr3 = data + 16;  // BSSID

    // Create or update station info
    WiFiStation station;
    memcpy(station.mac, addr2, 6);
    station.rssi = rx_ctrl->rssi;
    station.distance = calculateDistance(rx_ctrl->rssi);
    station.lastSeen = millis();
    station.channel = rx_ctrl->channel;
    station.isActive = true;

    // Add to stations list
    bool found = false;
    for (auto& s : wifiStations) {
        if (memcmp(s.mac, station.mac, 6) == 0) {
            s = station;
            found = true;
            break;
        }
    }
    if (!found && wifiStations.size() < MAX_STATIONS) {
        wifiStations.push_back(station);
    }

    // Update channel info
    if (rx_ctrl->channel <= MAX_CHANNELS) {
        channels[rx_ctrl->channel].stationCount++;
        channels[rx_ctrl->channel].utilization += 0.1;
        if (channels[rx_ctrl->channel].utilization > 100) {
            channels[rx_ctrl->channel].utilization = 100;
        }
    }
}

// Scan WiFi Networks
void scanNetworks() {
    if (isScanning) return;
    isScanning = true;

    // Clear old data
    for (auto& ch : channels) {
        ch.apCount = 0;
        ch.interference = 0;
        ch.maxRssi = -100;
        ch.minRssi = 0;
    }

    // Perform WiFi scan
    int n = WiFi.scanNetworks(false, true, false, 300, currentChannel);

    for (int i = 0; i < n && i < MAX_NETWORKS; i++) {
        String bssid = WiFi.BSSIDstr(i);

        // Update or create AP entry
        WiFiAP& ap = wifiNetworks[bssid];
        ap.ssid = WiFi.SSID(i);
        ap.bssid = bssid;
        ap.rssi = WiFi.RSSI(i);
        ap.channel = WiFi.channel(i);
        ap.encryption = (wifi_auth_mode_t)WiFi.encryptionType(i);
        ap.distance = calculateDistance(ap.rssi, channelToFrequency(ap.channel));
        ap.lastSeen = millis();

        // Update history
        ap.history[ap.historyIndex] = ap.rssi;
        ap.historyIndex = (ap.historyIndex + 1) % HISTORY_SIZE;

        // Calculate average RSSI and variance
        float sum = 0, sumSq = 0;
        int count = 0;
        for (int j = 0; j < HISTORY_SIZE; j++) {
            if (ap.history[j] != 0) {
                sum += ap.history[j];
                sumSq += ap.history[j] * ap.history[j];
                count++;
            }
        }
        if (count > 0) {
            ap.avgRssi = sum / count;
            ap.variance = (sumSq / count) - (ap.avgRssi * ap.avgRssi);
            ap.isMoving = ap.variance > 100;  // High variance indicates movement
        }

        // Calculate position for visualization
        ap.angle = (i * 360.0 / n) + random(-20, 20);  // Spread out evenly with some randomness
        float distancePixels = map(ap.distance, 0, scanRange, 0, RADAR_RADIUS);
        ap.x = cos(ap.angle * PI / 180.0) * distancePixels;
        ap.y = sin(ap.angle * PI / 180.0) * distancePixels;

        // Update channel statistics
        if (ap.channel <= MAX_CHANNELS) {
            channels[ap.channel].apCount++;
            if (ap.rssi > channels[ap.channel].maxRssi) {
                channels[ap.channel].maxRssi = ap.rssi;
            }
            if (ap.rssi < channels[ap.channel].minRssi) {
                channels[ap.channel].minRssi = ap.rssi;
            }
        }
    }

    // Calculate channel interference
    for (int ch = 1; ch <= MAX_CHANNELS; ch++) {
        // Check adjacent channel interference
        float interference = 0;
        if (ch > 1) interference += channels[ch-1].apCount * 0.5;
        if (ch < MAX_CHANNELS) interference += channels[ch+1].apCount * 0.5;
        if (ch > 2) interference += channels[ch-2].apCount * 0.25;
        if (ch < MAX_CHANNELS-1) interference += channels[ch+2].apCount * 0.25;

        channels[ch].interference = interference;
    }

    isScanning = false;
}

// Draw Radar Display
void drawRadar(lv_timer_t *timer) {
    static lv_color_t *buf = nullptr;
    if (!buf) {
        buf = (lv_color_t*)ps_malloc(RADAR_RADIUS * 2 * RADAR_RADIUS * 2 * sizeof(lv_color_t));
        if (!buf) return;
    }
    lv_canvas_set_buffer(radar_canvas, buf, RADAR_RADIUS * 2, RADAR_RADIUS * 2, LV_IMG_CF_TRUE_COLOR);

    // Clear with slight fade for trail effect
    for (int y = 0; y < RADAR_RADIUS * 2; y++) {
        for (int x = 0; x < RADAR_RADIUS * 2; x++) {
            int idx = y * RADAR_RADIUS * 2 + x;
            lv_color_t current = buf[idx];
            buf[idx] = lv_color_mix(color_dark, current, 240);  // Fade to dark
        }
    }

    // Draw radar circles
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = LV_COLOR_MAKE(0x00, 0x40, 0x00);
    arc_dsc.width = 1;

    for (int r = 30; r <= RADAR_RADIUS; r += 30) {
        lv_canvas_draw_arc(radar_canvas, RADAR_RADIUS, RADAR_RADIUS, r, 0, 360, &arc_dsc);
    }

    // Draw cross lines
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = LV_COLOR_MAKE(0x00, 0x40, 0x00);
    line_dsc.width = 1;

    lv_point_t line_points[2];

    // Draw grid lines
    for (int angle = 0; angle < 360; angle += 30) {
        float rad = angle * PI / 180.0;
        line_points[0].x = RADAR_RADIUS;
        line_points[0].y = RADAR_RADIUS;
        line_points[1].x = RADAR_RADIUS + cos(rad) * RADAR_RADIUS;
        line_points[1].y = RADAR_RADIUS - sin(rad) * RADAR_RADIUS;
        lv_canvas_draw_line(radar_canvas, line_points, 2, &line_dsc);
    }

    // Draw range labels
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = LV_COLOR_MAKE(0x00, 0x80, 0x00);

    for (int r = 30; r <= RADAR_RADIUS; r += 60) {
        float distance = (r * scanRange) / RADAR_RADIUS;
        char distStr[10];
        snprintf(distStr, sizeof(distStr), "%.0fm", distance);
        lv_canvas_draw_text(radar_canvas, RADAR_RADIUS + 5, RADAR_RADIUS - r - 5,
                           50, &label_dsc, distStr);
    }

    // Draw sweep line
    radarAngle += 2;
    if (radarAngle >= 360) radarAngle = 0;

    float sweepRad = radarAngle * PI / 180.0;
    line_dsc.color = color_green;
    line_dsc.width = 2;

    // Draw sweep with gradient
    for (int i = 0; i < 20; i++) {
        float angle = radarAngle - i * 2;
        if (angle < 0) angle += 360;
        float rad = angle * PI / 180.0;

        line_dsc.opa = 255 - i * 12;
        line_points[0].x = RADAR_RADIUS;
        line_points[0].y = RADAR_RADIUS;
        line_points[1].x = RADAR_RADIUS + cos(rad) * RADAR_RADIUS;
        line_points[1].y = RADAR_RADIUS - sin(rad) * RADAR_RADIUS;
        lv_canvas_draw_line(radar_canvas, line_points, 2, &line_dsc);
    }

    // Draw WiFi APs
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);

    for (auto& pair : wifiNetworks) {
        WiFiAP& ap = pair.second;

        // Skip if too old or too weak
        if (millis() - ap.lastSeen > 10000) continue;
        if (ap.rssi < sensitivity) continue;

        // Calculate position
        float distancePixels = map(ap.distance, 0, scanRange, 0, RADAR_RADIUS);
        int x = RADAR_RADIUS + ap.x;
        int y = RADAR_RADIUS - ap.y;

        // Choose color based on signal strength
        lv_color_t apColor;
        if (ap.rssi > -50) {
            apColor = color_green;
        } else if (ap.rssi > -70) {
            apColor = color_yellow;
        } else if (ap.rssi > -85) {
            apColor = color_orange;
        } else {
            apColor = color_red;
        }

        // Draw AP with size based on signal strength
        int size = map(ap.rssi, -100, -30, 2, 12);

        // Draw glow effect
        rect_dsc.bg_opa = LV_OPA_30;
        rect_dsc.bg_color = apColor;
        rect_dsc.radius = size * 2;
        lv_area_t area;
        area.x1 = x - size * 2;
        area.y1 = y - size * 2;
        area.x2 = x + size * 2;
        area.y2 = y + size * 2;
        lv_canvas_draw_rect(radar_canvas, area.x1, area.y1,
                           area.x2 - area.x1, area.y2 - area.y1, &rect_dsc);

        // Draw AP center
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = size;
        area.x1 = x - size;
        area.y1 = y - size;
        area.x2 = x + size;
        area.y2 = y + size;
        lv_canvas_draw_rect(radar_canvas, area.x1, area.y1,
                           area.x2 - area.x1, area.y2 - area.y1, &rect_dsc);

        // Draw SSID label for strong signals
        if (ap.rssi > -70 && ap.ssid.length() > 0) {
            label_dsc.color = color_cyan;
            lv_canvas_draw_text(radar_canvas, x + size + 2, y - 5, 100,
                               &label_dsc, ap.ssid.c_str());
        }

        // Add to radar trail
        int angleIdx = (int)ap.angle % 360;
        int distIdx = min((int)distancePixels, RADAR_RADIUS - 1);
        radarTrail[angleIdx][distIdx] = 255;
    }

    // Draw stations if in promiscuous mode
    if (promiscuousMode) {
        rect_dsc.radius = 2;
        rect_dsc.bg_color = color_cyan;
        rect_dsc.bg_opa = LV_OPA_70;

        for (auto& station : wifiStations) {
            if (!station.isActive) continue;
            if (millis() - station.lastSeen > 5000) {
                station.isActive = false;
                continue;
            }

            float angle = random(0, 360);
            float distancePixels = map(station.distance, 0, scanRange, 0, RADAR_RADIUS);
            int x = RADAR_RADIUS + cos(angle * PI / 180.0) * distancePixels;
            int y = RADAR_RADIUS - sin(angle * PI / 180.0) * distancePixels;

            lv_area_t area;
            area.x1 = x - 2;
            area.y1 = y - 2;
            area.x2 = x + 2;
            area.y2 = y + 2;
            lv_canvas_draw_rect(radar_canvas, area.x1, area.y1, 4, 4, &rect_dsc);
        }
    }

    // Draw radar trail/persistence
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.radius = 0;

    for (int a = 0; a < 360; a++) {
        for (int r = 0; r < RADAR_RADIUS; r++) {
            if (radarTrail[a][r] > 0) {
                float rad = a * PI / 180.0;
                int x = RADAR_RADIUS + cos(rad) * r;
                int y = RADAR_RADIUS - sin(rad) * r;

                rect_dsc.bg_color = lv_color_mix(color_green, color_dark, radarTrail[a][r]);
                lv_area_t area;
                area.x1 = x;
                area.y1 = y;
                area.x2 = x + 1;
                area.y2 = y + 1;
                lv_canvas_draw_rect(radar_canvas, area.x1, area.y1, 1, 1, &rect_dsc);

                // Fade trail
                radarTrail[a][r] = max(0, radarTrail[a][r] - 3);
            }
        }
    }
}

// Draw Spectrum Analyzer
void drawSpectrum() {
    static lv_color_t *buf = nullptr;
    if (!buf) {
        buf = (lv_color_t*)ps_malloc(SPECTRUM_WIDTH * SPECTRUM_HEIGHT * sizeof(lv_color_t));
        if (!buf) return;
    }
    lv_canvas_set_buffer(spectrum_canvas, buf, SPECTRUM_WIDTH, SPECTRUM_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    // Clear canvas
    lv_canvas_fill_bg(spectrum_canvas, color_dark, LV_OPA_COVER);

    // Draw grid
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = LV_COLOR_MAKE(0x20, 0x20, 0x20);
    line_dsc.width = 1;

    lv_point_t line_points[2];

    // Horizontal grid lines (RSSI levels)
    for (int i = 0; i <= 5; i++) {
        int y = i * SPECTRUM_HEIGHT / 5;
        line_points[0].x = 0;
        line_points[0].y = y;
        line_points[1].x = SPECTRUM_WIDTH;
        line_points[1].y = y;
        lv_canvas_draw_line(spectrum_canvas, line_points, 2, &line_dsc);
    }

    // Draw channels
    int channelWidth = SPECTRUM_WIDTH / 14;

    for (int ch = 1; ch <= 14; ch++) {
        int x = (ch - 1) * channelWidth;

        // Channel separator
        line_points[0].x = x;
        line_points[0].y = 0;
        line_points[1].x = x;
        line_points[1].y = SPECTRUM_HEIGHT;
        lv_canvas_draw_line(spectrum_canvas, line_points, 2, &line_dsc);

        // Draw channel utilization bar
        float utilization = channels[ch].utilization;
        float interference = channels[ch].interference;

        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);

        // Background bar (interference)
        int interferenceHeight = (interference / 10.0) * SPECTRUM_HEIGHT;
        rect_dsc.bg_color = color_red;
        rect_dsc.bg_opa = LV_OPA_30;

        lv_area_t area;
        area.x1 = x + 2;
        area.y1 = SPECTRUM_HEIGHT - interferenceHeight;
        area.x2 = x + channelWidth - 2;
        area.y2 = SPECTRUM_HEIGHT;
        lv_canvas_draw_rect(spectrum_canvas, area.x1, area.y1,
                           area.x2 - area.x1, area.y2 - area.y1, &rect_dsc);

        // Draw each AP in this channel
        int apOffset = 0;
        for (auto& pair : wifiNetworks) {
            WiFiAP& ap = pair.second;
            if (ap.channel != ch) continue;
            if (millis() - ap.lastSeen > 10000) continue;

            // Calculate bar height based on RSSI
            int height = map(ap.rssi, -100, -30, 5, SPECTRUM_HEIGHT - 10);
            int barWidth = (channelWidth - 4) / max(1, channels[ch].apCount);
            int barX = x + 2 + (apOffset * barWidth);

            // Choose color based on encryption
            if (ap.encryption == WIFI_AUTH_OPEN) {
                rect_dsc.bg_color = color_green;
            } else if (ap.encryption == WIFI_AUTH_WPA2_PSK) {
                rect_dsc.bg_color = color_blue;
            } else {
                rect_dsc.bg_color = color_orange;
            }

            rect_dsc.bg_opa = LV_OPA_80;

            area.x1 = barX;
            area.y1 = SPECTRUM_HEIGHT - height;
            area.x2 = barX + barWidth - 1;
            area.y2 = SPECTRUM_HEIGHT;
            lv_canvas_draw_rect(spectrum_canvas, area.x1, area.y1,
                               area.x2 - area.x1, area.y2 - area.y1, &rect_dsc);

            apOffset++;
        }

        // Draw channel number
        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        label_dsc.color = color_cyan;

        char chStr[4];
        snprintf(chStr, sizeof(chStr), "%d", ch);
        lv_canvas_draw_text(spectrum_canvas, x + channelWidth/2 - 5,
                           SPECTRUM_HEIGHT - 15, 20, &label_dsc, chStr);
    }
}

// Update display information
void updateDisplay(lv_timer_t *timer) {
    // Count active networks and stations
    int activeNetworks = 0;
    int activeStations = 0;
    WiFiAP* strongest = nullptr;
    WiFiAP* weakest = nullptr;

    for (auto& pair : wifiNetworks) {
        WiFiAP& ap = pair.second;
        if (millis() - ap.lastSeen < 10000) {
            activeNetworks++;
            if (!strongest || ap.rssi > strongest->rssi) {
                strongest = &ap;
            }
            if (!weakest || ap.rssi < weakest->rssi) {
                weakest = &ap;
            }
        }
    }

    for (auto& station : wifiStations) {
        if (station.isActive) {
            activeStations++;
        }
    }

    // Update labels
    lv_label_set_text_fmt(network_count_label, "Networks: %d", activeNetworks);
    lv_label_set_text_fmt(station_count_label, "Stations: %d", activeStations);
    lv_label_set_text_fmt(channel_label, "Channel: %d", currentChannel);

    if (strongest) {
        lv_label_set_text_fmt(strongest_label, "Strongest: %s (%d dBm)",
                             strongest->ssid.c_str(), strongest->rssi);
    }

    if (weakest) {
        lv_label_set_text_fmt(weakest_label, "Weakest: %s (%d dBm)",
                             weakest->ssid.c_str(), weakest->rssi);
    }

    // Calculate overall interference
    float totalInterference = 0;
    for (int ch = 1; ch <= MAX_CHANNELS; ch++) {
        totalInterference += channels[ch].interference;
    }
    lv_label_set_text_fmt(interference_label, "Interference: %.1f%%",
                         totalInterference / MAX_CHANNELS);

    // Update scan status
    if (isScanning) {
        lv_label_set_text(scan_status_label, "SCANNING...");
        lv_obj_set_style_text_color(scan_status_label, color_yellow, 0);
    } else {
        lv_label_set_text(scan_status_label, "IDLE");
        lv_obj_set_style_text_color(scan_status_label, color_green, 0);
    }

    // Update network list
    lv_obj_clean(network_list);
    int listCount = 0;

    for (auto& pair : wifiNetworks) {
        WiFiAP& ap = pair.second;
        if (millis() - ap.lastSeen > 10000) continue;
        if (listCount >= 10) break;  // Limit list size

        lv_obj_t *item = lv_label_create(network_list);
        char itemText[100];
        snprintf(itemText, sizeof(itemText), "%-20s CH:%-2d %4d dBm %.1fm %s",
                ap.ssid.length() > 0 ? ap.ssid.c_str() : "[Hidden]",
                ap.channel, ap.rssi, ap.distance,
                ap.isMoving ? "📶" : "📍");

        lv_label_set_text(item, itemText);
        lv_obj_set_style_text_font(item, &lv_font_montserrat_14, 0);

        // Color based on signal
        if (ap.rssi > -50) {
            lv_obj_set_style_text_color(item, color_green, 0);
        } else if (ap.rssi > -70) {
            lv_obj_set_style_text_color(item, color_yellow, 0);
        } else {
            lv_obj_set_style_text_color(item, color_orange, 0);
        }

        lv_obj_align(item, LV_ALIGN_TOP_LEFT, 5, 5 + listCount * 18);
        listCount++;
    }

    // Update RSSI chart
    static int chartCounter = 0;
    if (chartCounter++ % 5 == 0) {  // Update every 5th call
        int seriesIdx = 0;
        for (auto& pair : wifiNetworks) {
            if (seriesIdx >= 5) break;
            WiFiAP& ap = pair.second;
            if (millis() - ap.lastSeen < 10000) {
                lv_chart_set_next_value(network_chart, rssi_series[seriesIdx],
                                       100 + ap.rssi);
                seriesIdx++;
            }
        }
    }
}

// Touch event handlers
static void scan_btn_event_cb(lv_event_t *e) {
    scanNetworks();
}

static void promiscuous_btn_event_cb(lv_event_t *e) {
    promiscuousMode = !promiscuousMode;

    if (promiscuousMode) {
        // Enable promiscuous mode
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
        lv_label_set_text(lv_obj_get_child(promiscuous_btn, 0), "PROMISCUOUS ON");
        lv_obj_set_style_bg_color(promiscuous_btn, color_red, 0);
    } else {
        // Disable promiscuous mode
        esp_wifi_set_promiscuous(false);
        lv_label_set_text(lv_obj_get_child(promiscuous_btn, 0), "PROMISCUOUS OFF");
        lv_obj_set_style_bg_color(promiscuous_btn, LV_COLOR_MAKE(0x40, 0x40, 0x40), 0);
    }
}

static void channel_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    currentChannel = lv_slider_get_value(slider);

    // Set WiFi channel
    if (promiscuousMode) {
        esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    }
}

static void range_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    scanRange = lv_slider_get_value(slider);
}

static void sensitivity_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    sensitivity = -lv_slider_get_value(slider);
}

static void view_selector_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    viewMode = lv_btnmatrix_get_selected_btn(btn);

    // Show/hide canvases based on view
    lv_obj_add_flag(radar_canvas, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(spectrum_canvas, LV_OBJ_FLAG_HIDDEN);

    switch(viewMode) {
        case 0:  // Radar
            lv_obj_clear_flag(radar_canvas, LV_OBJ_FLAG_HIDDEN);
            break;
        case 1:  // Spectrum
            lv_obj_clear_flag(spectrum_canvas, LV_OBJ_FLAG_HIDDEN);
            break;
    }
}

// Periodic scan timer
void scan_timer_cb(lv_timer_t *timer) {
    if (!isScanning && WiFi.getMode() != WIFI_MODE_NULL) {
        scanNetworks();
    }
}

// Create UI
void createWiFiLidarUI() {
    scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, color_dark, 0);

    // Title bar
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, 800, 35);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, LV_COLOR_MAKE(0x10, 0x10, 0x10), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(title_bar);
    lv_label_set_text(title, "WiFi LiDAR - Professional Environmental Scanner");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(title, color_cyan, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 10, 0);

    mode_label = lv_label_create(title_bar);
    lv_label_set_text(mode_label, "RADAR MODE");
    lv_obj_set_style_text_color(mode_label, color_green, 0);
    lv_obj_align(mode_label, LV_ALIGN_RIGHT_MID, -10, 0);

    // Main display area
    lv_obj_t *display_container = lv_obj_create(scr);
    lv_obj_set_size(display_container, 480, 380);
    lv_obj_align(display_container, LV_ALIGN_TOP_LEFT, 5, 40);
    lv_obj_set_style_bg_color(display_container, LV_COLOR_MAKE(0x05, 0x05, 0x05), 0);
    lv_obj_set_style_border_color(display_container, color_green, 0);
    lv_obj_set_style_border_width(display_container, 1, 0);
    lv_obj_clear_flag(display_container, LV_OBJ_FLAG_SCROLLABLE);

    // Radar canvas
    radar_canvas = lv_canvas_create(display_container);
    lv_obj_set_size(radar_canvas, RADAR_RADIUS * 2, RADAR_RADIUS * 2);
    lv_obj_center(radar_canvas);

    // Spectrum canvas (initially hidden)
    spectrum_canvas = lv_canvas_create(display_container);
    lv_obj_set_size(spectrum_canvas, 460, SPECTRUM_HEIGHT);
    lv_obj_align(spectrum_canvas, LV_ALIGN_CENTER, 0, -50);
    lv_obj_add_flag(spectrum_canvas, LV_OBJ_FLAG_HIDDEN);

    // Info panel
    info_panel = lv_obj_create(scr);
    lv_obj_set_size(info_panel, 305, 180);
    lv_obj_align(info_panel, LV_ALIGN_TOP_RIGHT, -5, 40);
    lv_obj_set_style_bg_color(info_panel, LV_COLOR_MAKE(0x08, 0x08, 0x08), 0);
    lv_obj_set_style_border_color(info_panel, color_blue, 0);
    lv_obj_set_style_border_width(info_panel, 1, 0);
    lv_obj_clear_flag(info_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Info labels
    network_count_label = lv_label_create(info_panel);
    lv_label_set_text(network_count_label, "Networks: 0");
    lv_obj_set_style_text_color(network_count_label, color_cyan, 0);
    lv_obj_align(network_count_label, LV_ALIGN_TOP_LEFT, 10, 10);

    station_count_label = lv_label_create(info_panel);
    lv_label_set_text(station_count_label, "Stations: 0");
    lv_obj_set_style_text_color(station_count_label, color_cyan, 0);
    lv_obj_align(station_count_label, LV_ALIGN_TOP_LEFT, 10, 30);

    channel_label = lv_label_create(info_panel);
    lv_label_set_text(channel_label, "Channel: 1");
    lv_obj_set_style_text_color(channel_label, color_cyan, 0);
    lv_obj_align(channel_label, LV_ALIGN_TOP_LEFT, 10, 50);

    strongest_label = lv_label_create(info_panel);
    lv_label_set_text(strongest_label, "Strongest: ---");
    lv_obj_set_style_text_font(strongest_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(strongest_label, color_green, 0);
    lv_obj_align(strongest_label, LV_ALIGN_TOP_LEFT, 10, 70);

    weakest_label = lv_label_create(info_panel);
    lv_label_set_text(weakest_label, "Weakest: ---");
    lv_obj_set_style_text_font(weakest_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(weakest_label, color_red, 0);
    lv_obj_align(weakest_label, LV_ALIGN_TOP_LEFT, 10, 90);

    interference_label = lv_label_create(info_panel);
    lv_label_set_text(interference_label, "Interference: 0%");
    lv_obj_set_style_text_color(interference_label, color_orange, 0);
    lv_obj_align(interference_label, LV_ALIGN_TOP_LEFT, 10, 110);

    scan_status_label = lv_label_create(info_panel);
    lv_label_set_text(scan_status_label, "INITIALIZING...");
    lv_obj_set_style_text_color(scan_status_label, color_yellow, 0);
    lv_obj_align(scan_status_label, LV_ALIGN_TOP_LEFT, 10, 130);

    // RSSI Chart
    network_chart = lv_chart_create(info_panel);
    lv_obj_set_size(network_chart, 140, 60);
    lv_obj_align(network_chart, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_chart_set_type(network_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(network_chart, 30);
    lv_chart_set_range(network_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_obj_set_style_bg_opa(network_chart, LV_OPA_20, 0);

    for (int i = 0; i < 5; i++) {
        rssi_series[i] = lv_chart_add_series(network_chart,
            lv_color_hsv_to_rgb(i * 60, 100, 100), LV_CHART_AXIS_PRIMARY_Y);
    }

    // Network list
    network_list = lv_obj_create(scr);
    lv_obj_set_size(network_list, 305, 195);
    lv_obj_align(network_list, LV_ALIGN_TOP_RIGHT, -5, 225);
    lv_obj_set_style_bg_color(network_list, LV_COLOR_MAKE(0x08, 0x08, 0x08), 0);
    lv_obj_set_style_border_color(network_list, color_purple, 0);
    lv_obj_set_style_border_width(network_list, 1, 0);

    // Control panel
    control_panel = lv_obj_create(scr);
    lv_obj_set_size(control_panel, 480, 55);
    lv_obj_align(control_panel, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_set_style_bg_color(control_panel, LV_COLOR_MAKE(0x10, 0x10, 0x10), 0);
    lv_obj_set_style_border_color(control_panel, LV_COLOR_MAKE(0x30, 0x30, 0x30), 0);
    lv_obj_set_style_border_width(control_panel, 1, 0);
    lv_obj_clear_flag(control_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Scan button
    scan_btn = lv_btn_create(control_panel);
    lv_obj_set_size(scan_btn, 70, 40);
    lv_obj_align(scan_btn, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_bg_color(scan_btn, color_green, 0);
    lv_obj_add_event_cb(scan_btn, scan_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *scan_label = lv_label_create(scan_btn);
    lv_label_set_text(scan_label, "SCAN");
    lv_obj_center(scan_label);

    // Promiscuous button
    promiscuous_btn = lv_btn_create(control_panel);
    lv_obj_set_size(promiscuous_btn, 110, 40);
    lv_obj_align(promiscuous_btn, LV_ALIGN_LEFT_MID, 85, 0);
    lv_obj_set_style_bg_color(promiscuous_btn, LV_COLOR_MAKE(0x40, 0x40, 0x40), 0);
    lv_obj_add_event_cb(promiscuous_btn, promiscuous_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *prom_label = lv_label_create(promiscuous_btn);
    lv_label_set_text(prom_label, "PROMISCUOUS OFF");
    lv_obj_set_style_text_font(prom_label, &lv_font_montserrat_14, 0);
    lv_obj_center(prom_label);

    // Channel slider
    channel_slider = lv_slider_create(control_panel);
    lv_obj_set_size(channel_slider, 100, 15);
    lv_obj_align(channel_slider, LV_ALIGN_LEFT_MID, 205, -10);
    lv_slider_set_range(channel_slider, 1, 14);
    lv_slider_set_value(channel_slider, currentChannel, LV_ANIM_OFF);
    lv_obj_add_event_cb(channel_slider, channel_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *ch_label = lv_label_create(control_panel);
    lv_label_set_text(ch_label, "Channel");
    lv_obj_set_style_text_font(ch_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ch_label, color_cyan, 0);
    lv_obj_align(ch_label, LV_ALIGN_LEFT_MID, 205, 10);

    // Range slider
    range_slider = lv_slider_create(control_panel);
    lv_obj_set_size(range_slider, 80, 15);
    lv_obj_align(range_slider, LV_ALIGN_LEFT_MID, 315, -10);
    lv_slider_set_range(range_slider, 10, 200);
    lv_slider_set_value(range_slider, scanRange, LV_ANIM_OFF);
    lv_obj_add_event_cb(range_slider, range_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *range_label = lv_label_create(control_panel);
    lv_label_set_text(range_label, "Range");
    lv_obj_set_style_text_font(range_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(range_label, color_cyan, 0);
    lv_obj_align(range_label, LV_ALIGN_LEFT_MID, 315, 10);

    // Sensitivity slider
    sensitivity_slider = lv_slider_create(control_panel);
    lv_obj_set_size(sensitivity_slider, 70, 15);
    lv_obj_align(sensitivity_slider, LV_ALIGN_LEFT_MID, 405, -10);
    lv_slider_set_range(sensitivity_slider, 50, 100);
    lv_slider_set_value(sensitivity_slider, abs(sensitivity), LV_ANIM_OFF);
    lv_obj_add_event_cb(sensitivity_slider, sensitivity_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *sens_label = lv_label_create(control_panel);
    lv_label_set_text(sens_label, "Sensitivity");
    lv_obj_set_style_text_font(sens_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sens_label, color_cyan, 0);
    lv_obj_align(sens_label, LV_ALIGN_LEFT_MID, 400, 10);

    // View selector
    static const char * view_map[] = {"Radar", "Spectrum", ""};
    view_selector = lv_btnmatrix_create(scr);
    lv_btnmatrix_set_map(view_selector, view_map);
    lv_obj_set_size(view_selector, 305, 40);
    lv_obj_align(view_selector, LV_ALIGN_BOTTOM_RIGHT, -5, -10);
    lv_btnmatrix_set_btn_ctrl_all(view_selector, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(view_selector, true);
    lv_btnmatrix_set_btn_ctrl(view_selector, 0, LV_BTNMATRIX_CTRL_CHECKED);
    lv_obj_add_event_cb(view_selector, view_selector_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("Advanced WiFi LiDAR Visualization System");
    Serial.println("ESP32-S3 7\" Touch LCD");

    // Initialize board
    Serial.println("Initializing display...");
    board = new Board();
    board->init();

    if (!board->begin()) {
        Serial.println("Board begin failed!");
    }

    // Initialize LVGL
    Serial.println("Initializing LVGL...");
    lvgl_port_init(board->getLCD(), board->getTouch());

    // Initialize WiFi
    Serial.println("Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Create UI
    Serial.println("Creating WiFi LiDAR UI...");
    lvgl_port_lock(-1);
    createWiFiLidarUI();
    lvgl_port_unlock();

    // Create timers
    scan_timer = lv_timer_create(scan_timer_cb, SCAN_INTERVAL, NULL);
    update_timer = lv_timer_create(updateDisplay, 200, NULL);
    animation_timer = lv_timer_create(drawRadar, 50, NULL);

    // Initial scan
    scanNetworks();

    Serial.println("WiFi LiDAR System Ready!");
}

void loop() {
    delay(5);
}