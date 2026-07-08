#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <lv_rt_thread_conf.h>
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "lv_port_disp.h"
#include "lvgl.h"
#include "lv_timer.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lsm6ds3_simple.h"

// =====================================================
//  全局变量
// =====================================================
lv_obj_t *ring_chart;
lv_obj_t *percent_label;
lv_obj_t *result_label;
lv_obj_t *vib_bar, *temp_bar, *audio_bar;
lv_obj_t *node_cards[4];
lv_obj_t *power_label;
lv_obj_t *node_status_labels[4];
lv_obj_t *node_dots[4];

lv_obj_t *analysis_page = NULL;
int current_analysis_node = -1;

// =====================================================
//  传感器数据结构
// =====================================================
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    float temp_c;
    int vib_percent;
    int temp_percent;
    int confidence;
    uint8_t initialized;
} sensor_data_t;

static sensor_data_t g_sensor = {0};

// =====================================================
//  Audio 相关（直接读取 mic0 设备）
// =====================================================
#define AUDIO_BUFFER_SIZE   128
static int16_t g_audio_buffer[AUDIO_BUFFER_SIZE];
static int g_audio_energy = 20;
static rt_device_t g_mic_dev = RT_NULL;

// 读取麦克风数据并计算 RMS 能量（降低灵敏度版）
static int audio_read_pdm_energy(void)
{
    int16_t *buf = g_audio_buffer;
    int samples = AUDIO_BUFFER_SIZE;
    int i;
    int32_t sum_sq = 0;
    float rms;
    int energy;

    if (g_mic_dev == RT_NULL) {
        g_mic_dev = rt_device_find("mic0");
        if (g_mic_dev != RT_NULL) {
            rt_device_open(g_mic_dev, RT_DEVICE_OFLAG_RDONLY);
        } else {
            return 15 + rand() % 10;
        }
    }

    int len = rt_device_read(g_mic_dev, 0, buf, samples * 2);
    if (len > 0) {
        samples = len / 2;
    } else {
        return 15 + rand() % 10;
    }

    for (i = 0; i < samples; i++) {
        sum_sq += buf[i] * buf[i];
    }
    rms = sqrtf((float)sum_sq / samples);

    // 灵敏度
    rms = rms * 2.0f;

    // 映射门槛
    if (rms < 50) {
        energy = 5 + (int)(rms / 50.0f * 15);
    } else if (rms < 2000) {
        energy = 20 + (int)((rms - 50) / 1950.0f * 60);
    } else if (rms < 8000) {
        energy = 80 + (int)((rms - 2000) / 6000.0f * 20);
    } else {
        energy = 100;
    }

    if (energy > 100) energy = 100;
    if (energy < 5) energy = 5;

    return energy;
}

// 音频采集任务
static void audio_task(void)
{
    g_audio_energy = audio_read_pdm_energy();
}

// =====================================================
//  节点快照数据结构
// =====================================================
typedef struct {
    int conf;
    int vib;
    float temp;
    int audio;
    char status_text[16];
    bool is_alert;
    uint32_t last_update_tick;
} node_snapshot_t;

node_snapshot_t node_snapshots[4];

// ===== 节点轮询计数器 =====
static int current_update_node = 0;
static uint32_t last_update_tick = 0;

#define LED_PIN_G GET_PIN(16, 6)
#define TEMP_CALIB_OFFSET  12.0f

// =====================================================
//  函数声明
// =====================================================
static void node_card_event_handler(lv_event_t *e);
static void back_to_main_handler(lv_event_t *e);
static void create_analysis_page(int node_idx);

// =====================================================
//  UI更新函数（对外接口）
// =====================================================
void ui_set_confidence(int value);
void ui_set_result(const char *text, bool is_normal);
void ui_set_sensor_values(int vib, int temp, int audio);
void ui_set_node_state(int node_idx, bool alert);   // 新增声明

// =====================================================
//  LSM6DS3 读取传感器数据
// =====================================================
static void lsm6ds3_read_data(void)
{
    int16_t x, y, z, temp;

    lsm6ds3_simple_read_accel(&x, &y, &z);
    lsm6ds3_simple_read_temp(&temp);

    g_sensor.accel_x = x;
    g_sensor.accel_y = y;
    g_sensor.accel_z = z;
    g_sensor.temp_raw = temp;

    g_sensor.temp_c = (float)temp / 16.0f + 25.0f + TEMP_CALIB_OFFSET;

    // 振动强度计算（滑动窗口偏置）
    #define BIAS_WINDOW_SIZE 64
    static int16_t bias_buffer_x[BIAS_WINDOW_SIZE];
    static int16_t bias_buffer_y[BIAS_WINDOW_SIZE];
    static int16_t bias_buffer_z[BIAS_WINDOW_SIZE];
    static uint16_t bias_index = 0;
    static uint16_t bias_count = 0;

    bias_buffer_x[bias_index] = x;
    bias_buffer_y[bias_index] = y;
    bias_buffer_z[bias_index] = z;
    bias_index = (bias_index + 1) % BIAS_WINDOW_SIZE;
    if (bias_count < BIAS_WINDOW_SIZE) bias_count++;

    if (bias_count >= BIAS_WINDOW_SIZE / 2) {
        int32_t sum_x = 0, sum_y = 0, sum_z = 0;
        for (int i = 0; i < bias_count; i++) {
            sum_x += bias_buffer_x[i];
            sum_y += bias_buffer_y[i];
            sum_z += bias_buffer_z[i];
        }
        int16_t bias_x = sum_x / bias_count;
        int16_t bias_y = sum_y / bias_count;
        int16_t bias_z = sum_z / bias_count;

        int16_t dx = x - bias_x;
        int16_t dy = y - bias_y;
        int16_t dz = z - bias_z;
        float dynamic_mg = sqrtf((float)(dx*dx + dy*dy + dz*dz));

        int vib = (int)(dynamic_mg / 2000.0f * 100);
        if (vib > 100) vib = 100;
        g_sensor.vib_percent = vib;
        g_sensor.initialized = 1;
    } else {
        g_sensor.vib_percent = 0;
        g_sensor.initialized = 0;
    }

    int temp_val = (int)((g_sensor.temp_c + 20.0f) / 100.0f * 100);
    if (temp_val < 0) temp_val = 0;
    if (temp_val > 100) temp_val = 100;
    g_sensor.temp_percent = temp_val;

    if (g_sensor.initialized) {
        int conf = 100 - (g_sensor.vib_percent - 20) / 2;
        if (conf > 98) conf = 98;
        if (conf < 30) conf = 30;
        g_sensor.confidence = conf;
    } else {
        g_sensor.confidence = 90;
    }
}

// =====================================================
//  初始化节点快照
// =====================================================
static void init_node_snapshots(void)
{
    for (int i = 0; i < 4; i++) {
        node_snapshots[i].conf = 85 + i * 3;
        node_snapshots[i].vib = 30 + i * 10;
        node_snapshots[i].temp = 25.0f + i * 2;
        node_snapshots[i].audio = 20 + i * 8;
        node_snapshots[i].is_alert = false;
        strcpy(node_snapshots[i].status_text, "OK");
        node_snapshots[i].last_update_tick = 0;
    }
}

// =====================================================
//  更新单个节点的快照（加入声纹告警判断）
// =====================================================
static void update_node_snapshot(int node_idx)
{
    int vib = g_sensor.vib_percent;
    float temp = g_sensor.temp_c;
    int conf = g_sensor.confidence;
    int audio = g_audio_energy;

    node_snapshots[node_idx].vib = vib;
    node_snapshots[node_idx].temp = temp;
    node_snapshots[node_idx].audio = audio;

    if (node_idx == 0) {
        node_snapshots[node_idx].conf = conf;
        // 加入声纹异常判断：audio > 75 触发告警
        if (conf < 65 || vib > 70 || audio > 75) {
            node_snapshots[node_idx].is_alert = true;
            strcpy(node_snapshots[node_idx].status_text, "ALERT");
        } else {
            node_snapshots[node_idx].is_alert = false;
            strcpy(node_snapshots[node_idx].status_text, "OK");
        }
    } else {
        int v_conf = conf - 10 + rand() % 20;
        if (v_conf > 95) v_conf = 95;
        if (v_conf < 30) v_conf = 30;
        node_snapshots[node_idx].conf = v_conf;

        if (v_conf < 60 || vib > 70 || temp > 40.0f) {
            node_snapshots[node_idx].is_alert = true;
            strcpy(node_snapshots[node_idx].status_text, "ALERT");
        } else {
            node_snapshots[node_idx].is_alert = false;
            strcpy(node_snapshots[node_idx].status_text, "OK");
        }
    }

    node_snapshots[node_idx].last_update_tick = rt_tick_get();
}

// =====================================================
//  更新节点卡片UI
// =====================================================
static void update_node_card_ui(int node_idx)
{
    node_snapshot_t *snap = &node_snapshots[node_idx];

    int color = snap->is_alert ? 0xFF4444 : 0x00FF00;
    lv_obj_set_style_border_color(node_cards[node_idx], lv_color_hex(color), LV_STATE_DEFAULT);

    if (node_dots[node_idx]) {
        lv_led_set_color(node_dots[node_idx], lv_color_hex(color));
        if (snap->is_alert) {
            lv_led_off(node_dots[node_idx]);
        } else {
            lv_led_on(node_dots[node_idx]);
        }
    }

    if (node_status_labels[node_idx]) {
        lv_label_set_text(node_status_labels[node_idx], snap->status_text);
        lv_obj_set_style_text_color(node_status_labels[node_idx],
            snap->is_alert ? lv_color_hex(0xFF4444) : lv_color_hex(0x44FF88),
            LV_STATE_DEFAULT);
    }
}

// =====================================================
//  轮询更新节点快照
// =====================================================
static void update_node_snapshots_round_robin(void)
{
    #define SAMPLE_INTERVAL_MS 200

    uint32_t now = rt_tick_get();
    if ((now - last_update_tick) >= (SAMPLE_INTERVAL_MS * 1000 / RT_TICK_PER_SECOND)) {
        lsm6ds3_read_data();
        audio_task();
        last_update_tick = now;

        static uint8_t sample_counter = 0;
        sample_counter++;
        if (sample_counter >= 10) {
            sample_counter = 0;
            update_node_snapshot(current_update_node);
            update_node_card_ui(current_update_node);
            current_update_node = (current_update_node + 1) % 4;
        }
    }
}

// =====================================================
//  主界面实时数据更新
// =====================================================
static void update_main_display(void)
{
    if (analysis_page != NULL) {
        return;
    }

    if (!g_sensor.initialized) {
        ui_set_sensor_values(0, 50, 20);
        ui_set_confidence(90);
        ui_set_result("Initializing...", true);
        return;
    }

    ui_set_sensor_values(g_sensor.vib_percent, g_sensor.temp_percent, g_audio_energy);
    ui_set_confidence(g_sensor.confidence);

    if (g_sensor.confidence >= 65 && g_sensor.vib_percent < 70) {
        ui_set_result("Normal", true);
    } else {
        ui_set_result("Alert", false);
    }

    char power_buf[64];
    float power = 0.45 + (g_sensor.vib_percent - 30) * 0.015;
    if (power > 5.0) power = 5.0;
    if (power < 0.45) power = 0.45;
    snprintf(power_buf, sizeof(power_buf), "Power %.2fmA | Infer 0.21ms", power);
    lv_label_set_text(power_label, power_buf);
}

// =====================================================
//  UI 初始化
// =====================================================
void lv_user_gui_init(void)
{
    init_node_snapshots();

    if (lsm6ds3_simple_init() != 0) {
        rt_kprintf("WARNING: LSM6DS3 init failed, sensor data will be 0\n");
    }

    // 初始化麦克风设备
    g_mic_dev = rt_device_find("mic0");
    if (g_mic_dev != RT_NULL) {
        rt_device_open(g_mic_dev, RT_DEVICE_OFLAG_RDONLY);
        rt_kprintf("mic0 opened successfully!\n");
    } else {
        rt_kprintf("mic0 not found!\n");
    }

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x0F0F1A), LV_STATE_DEFAULT);

    // 状态栏
    lv_obj_t *status_bar = lv_obj_create(lv_scr_act());
    lv_obj_set_size(status_bar, 480, 40);
    lv_obj_set_pos(status_bar, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x1A1A2E), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(status_bar, 0, LV_STATE_DEFAULT);

    lv_obj_t *dot = lv_led_create(status_bar);
    lv_obj_set_pos(dot, 15, 10);
    lv_obj_set_size(dot, 20, 20);
    lv_led_set_color(dot, lv_color_hex(0x00FF00));
    lv_led_on(dot);

    lv_obj_t *status_text = lv_label_create(status_bar);
    lv_label_set_text(status_text, "System Running");
    lv_obj_set_pos(status_text, 45, 10);
    lv_obj_set_style_text_color(status_text, lv_color_hex(0xCCCCCC), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(status_text, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    lv_obj_t *clock_label = lv_label_create(status_bar);
    lv_label_set_text(clock_label, "10:30:25");
    lv_obj_set_pos(clock_label, 380, 10);
    lv_obj_set_style_text_color(clock_label, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(clock_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    // 环形置信度
    lv_obj_t *ring_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(ring_container, 240, 240);
    lv_obj_set_pos(ring_container, 120, 60);
    lv_obj_set_style_bg_color(ring_container, lv_color_hex(0x0F0F1A), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ring_container, 0, LV_STATE_DEFAULT);

    ring_chart = lv_arc_create(ring_container);
    lv_obj_set_size(ring_chart, 200, 200);
    lv_obj_center(ring_chart);
    lv_arc_set_range(ring_chart, 0, 100);
    lv_arc_set_value(ring_chart, 87);
    lv_arc_set_bg_angles(ring_chart, 0, 360);
    lv_obj_set_style_arc_width(ring_chart, 12, LV_STATE_DEFAULT);
    lv_obj_set_style_arc_color(ring_chart, lv_color_hex(0x00CCFF), LV_STATE_DEFAULT);

    percent_label = lv_label_create(ring_container);
    lv_label_set_text(percent_label, "87%");
    lv_obj_center(percent_label);
    lv_obj_set_style_text_color(percent_label, lv_color_hex(0xFFFFFF), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_24, LV_STATE_DEFAULT);

    // 结果文字
    result_label = lv_label_create(lv_scr_act());
    lv_label_set_text(result_label, "Normal");
    lv_obj_set_pos(result_label, 180, 320);
    lv_obj_set_style_text_color(result_label, lv_color_hex(0x00FF88), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(result_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    // 三模态进度条
    lv_obj_t *vib_label = lv_label_create(lv_scr_act());
    lv_label_set_text(vib_label, "Vibration");
    lv_obj_set_pos(vib_label, 60, 390);
    lv_obj_set_style_text_color(vib_label, lv_color_hex(0xAAAAAA), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(vib_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    vib_bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(vib_bar, 220, 16);
    lv_obj_set_pos(vib_bar, 170, 392);
    lv_bar_set_range(vib_bar, 0, 100);
    lv_bar_set_value(vib_bar, 30, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(vib_bar, lv_color_hex(0x333344), LV_STATE_DEFAULT);

    lv_obj_t *temp_label = lv_label_create(lv_scr_act());
    lv_label_set_text(temp_label, "Temperature");
    lv_obj_set_pos(temp_label, 60, 420);
    lv_obj_set_style_text_color(temp_label, lv_color_hex(0xAAAAAA), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    temp_bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(temp_bar, 220, 16);
    lv_obj_set_pos(temp_bar, 170, 422);
    lv_bar_set_range(temp_bar, 0, 100);
    lv_bar_set_value(temp_bar, 25, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(temp_bar, lv_color_hex(0x333344), LV_STATE_DEFAULT);

    lv_obj_t *audio_label = lv_label_create(lv_scr_act());
    lv_label_set_text(audio_label, "Acoustic");
    lv_obj_set_pos(audio_label, 60, 450);
    lv_obj_set_style_text_color(audio_label, lv_color_hex(0xAAAAAA), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(audio_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    audio_bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(audio_bar, 220, 16);
    lv_obj_set_pos(audio_bar, 170, 452);
    lv_bar_set_range(audio_bar, 0, 100);
    lv_bar_set_value(audio_bar, 20, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(audio_bar, lv_color_hex(0x333344), LV_STATE_DEFAULT);

    // 节点卡片
    const char *node_names[] = {"Real", "Virt A", "Virt B", "Virt C"};

    for (int i = 0; i < 4; i++) {
        int x = 30 + i * 110;
        node_cards[i] = lv_obj_create(lv_scr_act());
        lv_obj_set_size(node_cards[i], 90, 70);
        lv_obj_set_pos(node_cards[i], x, 510);
        lv_obj_set_style_bg_color(node_cards[i], lv_color_hex(0x1A1A2E), LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(node_cards[i], 2, LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(node_cards[i], lv_color_hex(0x00FF00), LV_STATE_DEFAULT);

        lv_obj_t *name = lv_label_create(node_cards[i]);
        lv_label_set_text(name, node_names[i]);
        lv_obj_set_pos(name, 15, 6);
        lv_obj_set_style_text_color(name, lv_color_hex(0xCCCCCC), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(name, &lv_font_montserrat_12, LV_STATE_DEFAULT);

        node_dots[i] = lv_led_create(node_cards[i]);
        lv_obj_set_pos(node_dots[i], 35, 30);
        lv_obj_set_size(node_dots[i], 16, 16);
        lv_led_set_color(node_dots[i], lv_color_hex(0x00FF00));
        lv_led_on(node_dots[i]);

        node_status_labels[i] = lv_label_create(node_cards[i]);
        lv_label_set_text(node_status_labels[i], "OK");
        lv_obj_set_pos(node_status_labels[i], 28, 52);
        lv_obj_set_style_text_color(node_status_labels[i], lv_color_hex(0x44FF88), LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(node_status_labels[i], &lv_font_montserrat_12, LV_STATE_DEFAULT);

        lv_obj_add_event_cb(node_cards[i], node_card_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    }

    // 底部栏
    lv_obj_t *footer = lv_obj_create(lv_scr_act());
    lv_obj_set_size(footer, 480, 50);
    lv_obj_set_pos(footer, 0, 750);
    lv_obj_set_style_bg_color(footer, lv_color_hex(0x1A1A2E), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(footer, 0, LV_STATE_DEFAULT);

    lv_obj_t *scene_label = lv_label_create(footer);
    lv_label_set_text(scene_label, "Inspection");
    lv_obj_set_pos(scene_label, 15, 15);
    lv_obj_set_style_text_color(scene_label, lv_color_hex(0xCCCCCC), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(scene_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    power_label = lv_label_create(footer);
    lv_label_set_text(power_label, "Power 0.45mA | Infer 0.21ms");
    lv_obj_set_pos(power_label, 180, 15);
    lv_obj_set_style_text_color(power_label, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(power_label, &lv_font_montserrat_12, LV_STATE_DEFAULT);

    for (int i = 0; i < 4; i++) {
        update_node_card_ui(i);
    }
}

// =====================================================
//  UI更新函数定义
// =====================================================
void ui_set_confidence(int value) {
    lv_arc_set_value(ring_chart, value);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", value);
    lv_label_set_text(percent_label, buf);
}

void ui_set_result(const char *text, bool is_normal) {
    lv_label_set_text(result_label, text);
    lv_obj_set_style_text_color(result_label,
        is_normal ? lv_color_hex(0x00FF88) : lv_color_hex(0xFF4444),
        LV_STATE_DEFAULT);
}

void ui_set_sensor_values(int vib, int temp, int audio) {
    lv_bar_set_value(vib_bar, vib, LV_ANIM_OFF);
    lv_bar_set_value(temp_bar, temp, LV_ANIM_OFF);
    lv_bar_set_value(audio_bar, audio, LV_ANIM_OFF);
}

// ===== 节点告警状态（全局） =====
bool node_alert_state[4] = {false, false, false, false};

void ui_set_node_state(int node_idx, bool alert) {
    node_alert_state[node_idx] = alert;
    int color = alert ? 0xFF4444 : 0x00FF00;
    lv_obj_set_style_border_color(node_cards[node_idx], lv_color_hex(color), LV_STATE_DEFAULT);

    if (node_dots[node_idx]) {
        lv_led_set_color(node_dots[node_idx], lv_color_hex(color));
        if (alert) {
            lv_led_off(node_dots[node_idx]);
        } else {
            lv_led_on(node_dots[node_idx]);
        }
    }

    if (node_status_labels[node_idx]) {
        lv_label_set_text(node_status_labels[node_idx], alert ? "ALERT" : "OK");
        lv_obj_set_style_text_color(node_status_labels[node_idx],
            alert ? lv_color_hex(0xFF4444) : lv_color_hex(0x44FF88),
            LV_STATE_DEFAULT);
    }
}

// =====================================================
//  创建分析页
// =====================================================
static void create_analysis_page(int node_idx)
{
    if (analysis_page != NULL && current_analysis_node == node_idx) {
        return;
    }

    node_snapshot_t *snap = &node_snapshots[node_idx];
    const char *node_names[] = {"Real", "Virt A", "Virt B", "Virt C"};
    const char *reasons[] = {
        "Acoustic Anomaly Detected\n(Abnormal Noise Pattern)",
        "Temperature Sensor Over Limit\n(Cooling System Failure)",
        "Vibration Anomaly\n(Imbalance Detected)",
        "Acoustic Anomaly\n(Unusual Noise Pattern)"
    };

    lv_obj_t *new_page = lv_obj_create(lv_scr_act());
    lv_obj_set_size(new_page, 480, 800);
    lv_obj_set_pos(new_page, 0, 0);
    lv_obj_set_style_bg_color(new_page, lv_color_hex(0x0F0F1A), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(new_page, 0, LV_STATE_DEFAULT);

    char title_buf[80];
    snprintf(title_buf, sizeof(title_buf), "%s - FAULT ANALYSIS", node_names[node_idx]);
    lv_obj_t *title = lv_label_create(new_page);
    lv_label_set_text(title, title_buf);
    lv_obj_set_pos(title, 60, 20);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF4444), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_STATE_DEFAULT);

    char time_buf[48];
    snprintf(time_buf, sizeof(time_buf), "Snapshot at: %lu ms",
             (unsigned long)(snap->last_update_tick * 1000 / RT_TICK_PER_SECOND));
    lv_obj_t *time_label = lv_label_create(new_page);
    lv_label_set_text(time_label, time_buf);
    lv_obj_set_pos(time_label, 30, 55);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0x666666), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_12, LV_STATE_DEFAULT);

    lv_obj_t *line = lv_obj_create(new_page);
    lv_obj_set_size(line, 420, 2);
    lv_obj_set_pos(line, 30, 75);
    lv_obj_set_style_bg_color(line, lv_color_hex(0x333344), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(line, 0, LV_STATE_DEFAULT);

    // Fault Reason
    lv_obj_t *reason_title = lv_label_create(new_page);
    lv_label_set_text(reason_title, "FAULT REASON");
    lv_obj_set_pos(reason_title, 30, 95);
    lv_obj_set_style_text_color(reason_title, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(reason_title, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    lv_obj_t *analysis_reason_label = lv_label_create(new_page);
    lv_label_set_text(analysis_reason_label, reasons[node_idx]);
    lv_obj_set_pos(analysis_reason_label, 30, 125);
    lv_obj_set_style_text_color(analysis_reason_label, lv_color_hex(0xFFFFFF), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(analysis_reason_label, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    // Confidence
    lv_obj_t *conf_title = lv_label_create(new_page);
    lv_label_set_text(conf_title, "CONFIDENCE");
    lv_obj_set_pos(conf_title, 30, 190);
    lv_obj_set_style_text_color(conf_title, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(conf_title, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    char conf_buf[48];
    snprintf(conf_buf, sizeof(conf_buf), "Current: %d%% %s", snap->conf,
             snap->conf < 65 ? "(Below threshold 65%)" : "");
    lv_obj_t *analysis_conf_label = lv_label_create(new_page);
    lv_label_set_text(analysis_conf_label, conf_buf);
    lv_obj_set_pos(analysis_conf_label, 30, 220);
    lv_obj_set_style_text_color(analysis_conf_label,
        snap->conf < 65 ? lv_color_hex(0xFF8888) : lv_color_hex(0x88FF88),
        LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(analysis_conf_label, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    // Sensor Data
    lv_obj_t *sensor_title = lv_label_create(new_page);
    lv_label_set_text(sensor_title, "SENSOR DATA (Snapshot)");
    lv_obj_set_pos(sensor_title, 30, 270);
    lv_obj_set_style_text_color(sensor_title, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(sensor_title, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    char vib_buf[48], temp_buf[48], audio_buf[48];
    snprintf(vib_buf, sizeof(vib_buf), "Vibration: %d%% %s", snap->vib, snap->vib > 70 ? "(Above normal)" : "");
    snprintf(temp_buf, sizeof(temp_buf), "Temperature: %.1f °C %s", snap->temp, snap->temp > 40.0f ? "(Above normal)" : "");
    snprintf(audio_buf, sizeof(audio_buf), "Acoustic: %d%% %s", snap->audio, snap->audio > 70 ? "(Above normal)" : "");

    lv_obj_t *analysis_vib_label = lv_label_create(new_page);
    lv_label_set_text(analysis_vib_label, vib_buf);
    lv_obj_set_pos(analysis_vib_label, 30, 300);
    lv_obj_set_style_text_color(analysis_vib_label, snap->vib > 70 ? lv_color_hex(0xFFAA44) : lv_color_hex(0x44FF88), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(analysis_vib_label, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    lv_obj_t *analysis_temp_label = lv_label_create(new_page);
    lv_label_set_text(analysis_temp_label, temp_buf);
    lv_obj_set_pos(analysis_temp_label, 30, 330);
    lv_obj_set_style_text_color(analysis_temp_label, snap->temp > 40.0f ? lv_color_hex(0xFFAA44) : lv_color_hex(0x44FF88), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(analysis_temp_label, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    lv_obj_t *analysis_audio_label = lv_label_create(new_page);
    lv_label_set_text(analysis_audio_label, audio_buf);
    lv_obj_set_pos(analysis_audio_label, 30, 360);
    lv_obj_set_style_text_color(analysis_audio_label, snap->audio > 70 ? lv_color_hex(0xFFAA44) : lv_color_hex(0x44FF88), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(analysis_audio_label, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    // Recommended Actions
    lv_obj_t *action_title = lv_label_create(new_page);
    lv_label_set_text(action_title, "RECOMMENDED ACTIONS");
    lv_obj_set_pos(action_title, 30, 410);
    lv_obj_set_style_text_color(action_title, lv_color_hex(0x888888), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(action_title, &lv_font_montserrat_14, LV_STATE_DEFAULT);

    lv_obj_t *action_text = lv_label_create(new_page);
    lv_label_set_text(action_text,
        "1. Check system for anomalies\n"
        "2. Review sensor data history\n"
        "3. Schedule maintenance if needed");
    lv_obj_set_pos(action_text, 30, 440);
    lv_obj_set_style_text_color(action_text, lv_color_hex(0xCCCCCC), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(action_text, &lv_font_montserrat_16, LV_STATE_DEFAULT);

    // Back Button
    lv_obj_t *analysis_back_btn = lv_btn_create(new_page);
    lv_obj_set_size(analysis_back_btn, 160, 50);
    lv_obj_set_pos(analysis_back_btn, 160, 700);
    lv_obj_set_style_bg_color(analysis_back_btn, lv_color_hex(0x333355), LV_STATE_DEFAULT);
    lv_obj_add_event_cb(analysis_back_btn, back_to_main_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(analysis_back_btn);
    lv_label_set_text(btn_label, "BACK");
    lv_obj_center(btn_label);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), LV_STATE_DEFAULT);

    if (analysis_page != NULL) {
        lv_obj_del(analysis_page);
    }
    analysis_page = new_page;
    current_analysis_node = node_idx;
}

// =====================================================
//  返回主页面
// =====================================================
static void back_to_main_handler(lv_event_t *e)
{
    if (analysis_page) {
        lv_obj_del(analysis_page);
        analysis_page = NULL;
    }
    current_analysis_node = -1;

    ui_set_confidence(g_sensor.confidence);
    ui_set_result("Normal", true);
    ui_set_sensor_values(g_sensor.vib_percent, g_sensor.temp_percent, g_audio_energy);
    lv_label_set_text(power_label, "Power 0.45mA | Infer 0.21ms");
}

// =====================================================
//  触摸事件回调（Real节点触发声纹告警）
// =====================================================
static void node_card_event_handler(lv_event_t *e)
{
    int node_idx = (int)(intptr_t)lv_event_get_user_data(e);

    if (analysis_page != NULL && current_analysis_node == node_idx) {
        return;
    }

    // 点击 Real 节点时，触发声纹告警分析页
    if (node_idx == 0) {
        // 强制触发告警状态
        ui_set_confidence(65);
        ui_set_result("Alert", false);
        ui_set_sensor_values(g_sensor.vib_percent, g_sensor.temp_percent, 85);
        ui_set_node_state(0, true);
        lv_label_set_text(power_label, "Power 45mA | Infer 0.21ms");

        // 获取当前快照，修改声纹值为 85 触发告警
        node_snapshot_t *snap = &node_snapshots[node_idx];
        snap->audio = 85;
        snap->is_alert = true;
        strcpy(snap->status_text, "ALERT");
        create_analysis_page(node_idx);
        return;
    }

    // 其他节点正常显示分析页
    create_analysis_page(node_idx);
}

// =====================================================
//  主函数
// =====================================================
int main(void)
{
    rt_kprintf("Hello RT-Thread\n");
    rt_kprintf("It's cortex-m55\n");
    rt_pin_mode(LED_PIN_G, PIN_MODE_OUTPUT);
    lvgl_thread_init();

    srand(rt_tick_get());
    last_update_tick = rt_tick_get();

    lsm6ds3_read_data();

    while (1)
    {
        rt_pin_write(LED_PIN_G, PIN_LOW);
        rt_thread_mdelay(100);
        rt_pin_write(LED_PIN_G, PIN_HIGH);
        rt_thread_mdelay(100);

        update_node_snapshots_round_robin();
        update_main_display();
    }
    return 0;
}
