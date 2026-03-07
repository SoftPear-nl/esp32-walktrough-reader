/*
 * ST7789 320x240 display driver using LVGL v9 + esp_lcd
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "st7789_lvgl";

/* ── Rotary encoder/button state ─────────────── */
volatile int encoder_count = 0;
static int last_btn_state = 1;
static int last_enc_btn_state = 1;

// Encoder state for ISR
static volatile int last_enc_state = 0;
static volatile int enc_sequence = 0;
static volatile int enc_cycle_count = 0;

/* ── Pin definitions ──────────────────────────────────────── */
#define SCREEN_1_SCL_PIN  1   /* SPI SCLK  */
#define SCREEN_1_SDA_PIN  10  /* SPI MOSI  */
#define SCREEN_1_RES_PIN  11  /* LCD RST   */
#define SCREEN_1_DC_PIN   12  /* LCD DC    */
#define SCREEN_1_CS_PIN   13  /* LCD CS    */
#define ROT_ENC_A_PIN     4
#define ROT_ENC_B_PIN     7
#define ROT_ENC_BTN_PIN   6
#define BTN_PIN      5

/* ── Display geometry ─────────────────────────────────────── */
#define LCD_H_RES         240
#define LCD_V_RES         320
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_HOST          SPI2_HOST

/* Draw buffer: double-buffered, 24 lines each */
#define DRAW_BUF_LINES    24
#define DRAW_BUF_SIZE     (LCD_H_RES * DRAW_BUF_LINES * sizeof(uint16_t))

/* ── Globals ──────────────────────────────────────────────── */
static SemaphoreHandle_t lvgl_mux = NULL;

// Rotary encoder ISR
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    int enc_a = gpio_get_level(ROT_ENC_A_PIN);
    int enc_b = gpio_get_level(ROT_ENC_B_PIN);
    int enc_state = (enc_a << 1) | enc_b;

    // State machine for quadrature decoding
    static int prev_state = 0;
    static int step = 0;

    // Transition table: [prev][curr] = direction
    // 0: no move, 1: CW, -1: CCW
    static const int transition_table[4][4] = {
        {0, 1, -1, 0}, // prev 00
        {-1, 0, 0, 1}, // prev 01
        {1, 0, 0, -1}, // prev 10
        {0, -1, 1, 0}  // prev 11
    };

    int dir = transition_table[prev_state][enc_state];
    if (dir != 0) {
        step += dir;
        // Only count when 4 steps completed (one detent)
        if (step >= 4) {
            encoder_count++;
            step = 0;
        } else if (step <= -4) {
            encoder_count--;
            step = 0;
        }
    }
    prev_state = enc_state;
}

/* ── Rotary encoder/button polling task ──────── */
static void input_task(void *arg)
{
    ESP_LOGI(TAG, "Input task started");
    while (1) {
        int btn = gpio_get_level(BTN_PIN);
        int enc_btn = gpio_get_level(ROT_ENC_BTN_PIN);

        // Log encoder count and direction
        static int last_reported_count = 0;
        if (encoder_count != last_reported_count) {
            if (encoder_count > last_reported_count) {
                ESP_LOGI(TAG, "Encoder CW: %d", encoder_count);
            } else {
                ESP_LOGI(TAG, "Encoder CCW: %d", encoder_count);
            }
            last_reported_count = encoder_count;
        }

        // Button press detection (active low)
        if (btn == 0 && last_btn_state == 1) {
            ESP_LOGI(TAG, "Button pressed");
        }
        last_btn_state = btn;

        // Encoder push button detection (active low)
        if (enc_btn == 0 && last_enc_btn_state == 1) {
            ESP_LOGI(TAG, "Encoder button pressed");
        }
        last_enc_btn_state = enc_btn;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── LVGL tick source (v9 explicit callback) ──────────────── */
static uint32_t lvgl_tick_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── LVGL flush callback ──────────────────────────────────── */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    ESP_LOGI(TAG, "flush x1=%d y1=%d x2=%d y2=%d", area->x1, area->y1, area->x2, area->y2);
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              px_map);
    lv_display_flush_ready(disp);
}

/* ── LVGL timer task ──────────────────────────────────────── */
static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    uint32_t next_delay_ms = 1;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(next_delay_ms > 0 ? next_delay_ms : 1));
        if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
            next_delay_ms = lv_timer_handler();
            xSemaphoreGive(lvgl_mux);
        }
    }
}

/* ── Demo UI ──────────────────────────────────────────────── */
static void create_demo_ui(void)
{
    lv_obj_t *scr = lv_screen_active();

    /* Background colour */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1A1A2E), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* Title */
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "ST7789 + LVGL\n320 x 240");
    lv_obj_set_style_text_color(label, lv_color_hex(0x00D4FF), LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -30);

    /* Animated spinner */
    lv_obj_t *spinner = lv_spinner_create(scr);
    lv_obj_set_size(spinner, 60, 60);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 50);
}

/* ── app_main ─────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "Initialising ST7789 display");

    /* Rotary encoder and button GPIO setup */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROT_ENC_A_PIN) | (1ULL << ROT_ENC_B_PIN) |
                       (1ULL << ROT_ENC_BTN_PIN) | (1ULL << BTN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Encoder pins: enable interrupts
    gpio_set_intr_type(ROT_ENC_A_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ROT_ENC_B_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ROT_ENC_A_PIN, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ROT_ENC_B_PIN, encoder_isr_handler, NULL);

    xTaskCreate(input_task, "input_task", 2048, NULL, 4, NULL);

    /* 1. SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = SCREEN_1_SDA_PIN,
        .miso_io_num     = -1,
        .sclk_io_num     = SCREEN_1_SCL_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * DRAW_BUF_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 2. Panel IO (SPI -> ST7789) */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num        = SCREEN_1_DC_PIN,
        .cs_gpio_num        = SCREEN_1_CS_PIN,
        .pclk_hz            = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 8,
        .spi_mode           = 0,
        .trans_queue_depth  = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io_handle));

    /* 3. ST7789 panel */
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num  = SCREEN_1_RES_PIN,
        .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel  = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));     /* portrait: no axis swap */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(120)); /* allow panel to settle after init */
    ESP_LOGI(TAG, "ST7789 panel ready");

    /* 4. LVGL initialisation */
    lv_init();
    lv_tick_set_cb(lvgl_tick_cb); /* explicit v9 tick source */

    static uint8_t buf1[DRAW_BUF_SIZE];
    static uint8_t buf2[DRAW_BUF_SIZE];

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_buffers(disp, buf1, buf2, DRAW_BUF_SIZE,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* 5. LVGL mutex + task (stack 8 kB, priority 5) */
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 5, NULL);

    /* 6. Build demo UI and force first render */
    if (xSemaphoreTake(lvgl_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        create_demo_ui();
        lv_refr_now(disp); /* trigger immediate flush */
        xSemaphoreGive(lvgl_mux);
    }

    ESP_LOGI(TAG, "UI ready — entering idle loop");
}
