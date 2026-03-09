#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

static const char *TAG = "reader";

/* -- Pin definitions ---------------------------------------- */
#define SCREEN_1_SCL_PIN  1
#define SCREEN_1_SDA_PIN  10
#define SCREEN_1_RES_PIN  11
#define SCREEN_1_DC_PIN   12
#define SCREEN_1_CS_PIN   13
#define ROT_ENC_A_PIN     7
#define ROT_ENC_B_PIN     4
#define ROT_ENC_BTN_PIN   6
#define BTN_PIN           5

/* -- Display geometry --------------------------------------- */
#define LCD_H_RES           240
#define LCD_V_RES           320
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)
#define LCD_HOST            SPI2_HOST

/* -- Font / layout ------------------------------------------ */
#define FONT_W      8
#define FONT_H      8
#define LINE_PAD    2                              /* blank pixels above glyph */
#define CELL_W      FONT_W                         /* 8 px wide per character  */
#define CELL_H      (FONT_H + LINE_PAD)            /* 12 px tall per row       */
#define COLS        (LCD_H_RES / CELL_W)           /* 30 chars per row         */
#define ROWS        (LCD_V_RES / CELL_H)           /* 32 total rows            */
#define TEXT_ROWS   (ROWS - 1)                      /* 31 rows for text content  */

/* Colors: RGB565 stored byte-swapped so SPI little-endian send is correct */
#define COL_BG  0xC518u   /* #1A1A2E dark navy, RGB565 0x18C5 byte-swapped */
#define COL_FG  0x1CE7u   /* #E0E0E0 light grey, RGB565 0xE71C byte-swapped */

/* -- 8x8 bitmap font, ASCII 0x20-0x7E ----------------------- */
/* Each entry: 8 bytes, one per scanline, MSB = leftmost pixel */
static const uint8_t font8[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x20   */
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, /* 0x21 ! */
    {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00}, /* 0x22 " */
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, /* 0x23 # */
    {0x18,0x7E,0x03,0x3E,0x60,0x3F,0x18,0x00}, /* 0x24 $ */
    {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00}, /* 0x25 % */
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, /* 0x26 & */
    {0x18,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, /* 0x27 ' */
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, /* 0x28 ( */
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, /* 0x29 ) */
    {0x00,0x6C,0x38,0xFE,0x38,0x6C,0x00,0x00}, /* 0x2A * */
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, /* 0x2B + */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x0C}, /* 0x2C , */
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, /* 0x2D - */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, /* 0x2E . */
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x03,0x00}, /* 0x2F / */
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, /* 0x30 0 */
    {0x18,0x1C,0x18,0x18,0x18,0x18,0x7E,0x00}, /* 0x31 1 */
    {0x3C,0x66,0x60,0x38,0x0C,0x66,0x7E,0x00}, /* 0x32 2 */
    {0x3C,0x66,0x60,0x38,0x60,0x66,0x3C,0x00}, /* 0x33 3 */
    {0x70,0x78,0x6C,0x66,0xFE,0x60,0xF0,0x00}, /* 0x34 4 */
    {0x7E,0x06,0x3E,0x60,0x60,0x66,0x3C,0x00}, /* 0x35 5 */
    {0x38,0x0C,0x06,0x3E,0x66,0x66,0x3C,0x00}, /* 0x36 6 */
    {0x7E,0x66,0x60,0x30,0x18,0x18,0x18,0x00}, /* 0x37 7 */
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, /* 0x38 8 */
    {0x3C,0x66,0x66,0x7C,0x60,0x30,0x1C,0x00}, /* 0x39 9 */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, /* 0x3A : */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x0C}, /* 0x3B ; */
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, /* 0x3C < */
    {0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00}, /* 0x3D = */
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, /* 0x3E > */
    {0x3C,0x66,0x60,0x30,0x18,0x00,0x18,0x00}, /* 0x3F ? */
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, /* 0x40 @ */
    {0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, /* 0x41 A */
    {0x3E,0x66,0x66,0x3E,0x66,0x66,0x3E,0x00}, /* 0x42 B */
    {0x3C,0x66,0x06,0x06,0x06,0x66,0x3C,0x00}, /* 0x43 C */
    {0x1E,0x36,0x66,0x66,0x66,0x36,0x1E,0x00}, /* 0x44 D */
    {0x7E,0x06,0x06,0x1E,0x06,0x06,0x7E,0x00}, /* 0x45 E */
    {0x7E,0x06,0x06,0x1E,0x06,0x06,0x06,0x00}, /* 0x46 F */
    {0x3C,0x66,0x06,0x76,0x66,0x66,0x3C,0x00}, /* 0x47 G */
    {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, /* 0x48 H */
    {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, /* 0x49 I */
    {0x78,0x30,0x30,0x30,0x30,0x36,0x1C,0x00}, /* 0x4A J */
    {0x66,0x36,0x1E,0x0E,0x1E,0x36,0x66,0x00}, /* 0x4B K */
    {0x06,0x06,0x06,0x06,0x06,0x06,0x7E,0x00}, /* 0x4C L */
    {0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0x00}, /* 0x4D M */
    {0x66,0x6E,0x7E,0x76,0x66,0x66,0x66,0x00}, /* 0x4E N */
    {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, /* 0x4F O */
    {0x3E,0x66,0x66,0x3E,0x06,0x06,0x06,0x00}, /* 0x50 P */
    {0x3C,0x66,0x66,0x66,0x76,0x3C,0x70,0x00}, /* 0x51 Q */
    {0x3E,0x66,0x66,0x3E,0x1E,0x36,0x66,0x00}, /* 0x52 R */
    {0x3C,0x66,0x06,0x3C,0x60,0x66,0x3C,0x00}, /* 0x53 S */
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /* 0x54 T */
    {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, /* 0x55 U */
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, /* 0x56 V */
    {0xC6,0xC6,0xC6,0xD6,0xFE,0xEE,0xC6,0x00}, /* 0x57 W */
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, /* 0x58 X */
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, /* 0x59 Y */
    {0x7E,0x60,0x30,0x18,0x0C,0x06,0x7E,0x00}, /* 0x5A Z */
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, /* 0x5B [ */
    {0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x00}, /* 0x5C \ */
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, /* 0x5D ] */
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, /* 0x5E ^ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, /* 0x5F _ */
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, /* 0x60 ` */
    {0x00,0x00,0x3C,0x60,0x7C,0x66,0x7C,0x00}, /* 0x61 a */
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00}, /* 0x62 b */
    {0x00,0x00,0x3C,0x06,0x06,0x66,0x3C,0x00}, /* 0x63 c */
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, /* 0x64 d */
    {0x00,0x00,0x3C,0x66,0x7E,0x06,0x3C,0x00}, /* 0x65 e */
    {0x38,0x6C,0x0C,0x1E,0x0C,0x0C,0x1E,0x00}, /* 0x66 f */
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x3C}, /* 0x67 g */
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x00}, /* 0x68 h */
    {0x18,0x00,0x1C,0x18,0x18,0x18,0x3C,0x00}, /* 0x69 i */
    {0x30,0x00,0x38,0x30,0x30,0x36,0x1C,0x00}, /* 0x6A j */
    {0x06,0x06,0x66,0x36,0x1E,0x36,0x66,0x00}, /* 0x6B k */
    {0x1C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, /* 0x6C l */
    {0x00,0x00,0x6C,0xFE,0xD6,0xC6,0xC6,0x00}, /* 0x6D m */
    {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x00}, /* 0x6E n */
    {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, /* 0x6F o */
    {0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06}, /* 0x70 p */
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, /* 0x71 q */
    {0x00,0x00,0x3E,0x66,0x06,0x06,0x06,0x00}, /* 0x72 r */
    {0x00,0x00,0x7C,0x06,0x3C,0x60,0x3E,0x00}, /* 0x73 s */
    {0x0C,0x0C,0x3E,0x0C,0x0C,0x0C,0x38,0x00}, /* 0x74 t */
    {0x00,0x00,0x66,0x66,0x66,0x66,0x7C,0x00}, /* 0x75 u */
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, /* 0x76 v */
    {0x00,0x00,0xC6,0xD6,0xFE,0xEE,0x6C,0x00}, /* 0x77 w */
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, /* 0x78 x */
    {0x00,0x00,0x66,0x66,0x66,0x7C,0x60,0x3C}, /* 0x79 y */
    {0x00,0x00,0x7E,0x30,0x18,0x0C,0x7E,0x00}, /* 0x7A z */
    {0x38,0x18,0x18,0x0E,0x18,0x18,0x38,0x00}, /* 0x7B { */
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, /* 0x7C | */
    {0x0E,0x18,0x18,0x38,0x18,0x18,0x0E,0x00}, /* 0x7D } */
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x7E ~ */
};

/* -- Virtual line (file offset + length) -------------------- */
typedef struct { int offset; int len; } Line;

static FILE  *file_fd     = NULL;
static Line  *lines       = NULL;
static int    total_lines = 0;
static int    top_line    = 0;

/* -- Current file path (for NVS key) ------------------------ */
#define MAX_FILES    16
#define MAX_PATH_LEN 264   /* "/spiffs/" (8) + NAME_MAX (255) + NUL (1) */
static char current_file[MAX_PATH_LEN] = "/spiffs/ff10.txt";

/* -- File picker state -------------------------------------- */
static char             file_list[MAX_FILES][MAX_PATH_LEN];
static int              file_count  = 0;
static int              picker_sel  = 0;
static int              picker_top  = 0;
#define PICKER_VISIBLE  (TEXT_ROWS - 1)

typedef enum { MODE_READER, MODE_PICKER, MODE_SETTINGS } app_mode_t;
static volatile app_mode_t app_mode          = MODE_READER;
static volatile bool       btn_pressed       = false;
static volatile bool       picker_confirm    = false;
static volatile bool       settings_confirm  = false;

/* Settings screen state */
#define SETTINGS_ITEMS  1
static int  settings_sel = 0;
static bool wifi_active  = false;

/* -- Rotary encoder / button state -------------------------- */
static volatile int encoder_count      = 0;
static int          last_btn_state     = 1;
static int          last_enc_btn_state = 1;

/* -- Scroll speed modes ------------------------------------- */
#define NUM_SCROLL_MODES 3
static const int  scroll_steps[NUM_SCROLL_MODES] = { 2, 10, 0 }; /* 0 = page */
static const char *scroll_labels[NUM_SCROLL_MODES] = { "x2", "x10", "page" };
static volatile int scroll_mode = 0;
static volatile bool status_dirty = false;

/* -- LCD panel handle --------------------------------------- */
static esp_lcd_panel_handle_t panel_handle = NULL;

/* -- Double pixel-row buffers (heap-allocated for DMA) ------ */
static uint16_t *row_bufs[2] = { NULL, NULL };
static int       cur_buf     = 0;

/* -- NVS scroll position helpers ----------------------------- */
/* Build a short NVS key from filename (max 15 chars) */
static void nvs_key_from_path(const char *path, char *key, size_t key_sz)
{
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    const char *dot = strrchr(base, '.');
    int name_len = dot ? (int)(dot - base) : (int)strlen(base);
    if (name_len > 11) name_len = 11;        /* "pos_" + 11 = 15 */
    snprintf(key, key_sz, "pos_%.11s", base);
    key[4 + name_len] = '\0';
}

static int load_scroll_pos(const char *path)
{
    char key[16];
    nvs_key_from_path(path, key, sizeof(key));
    nvs_handle_t h;
    int32_t val = 0;
    if (nvs_open("reader", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, key, &val);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Loaded scroll pos %ld for %s", (long)val, key);
    return (int)val;
}

static void save_scroll_pos(const char *path, int pos)
{
    char key[16];
    nvs_key_from_path(path, key, sizeof(key));
    nvs_handle_t h;
    if (nvs_open("reader", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, key, (int32_t)pos);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* -- Rotary encoder ISR ------------------------------------- */
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    int enc_a = gpio_get_level(ROT_ENC_A_PIN);
    int enc_b = gpio_get_level(ROT_ENC_B_PIN);
    int enc_state = (enc_a << 1) | enc_b;

    static int prev_state = 0;
    static int step       = 0;

    static const int transition_table[4][4] = {
        { 0,  1, -1,  0},
        {-1,  0,  0,  1},
        { 1,  0,  0, -1},
        { 0, -1,  1,  0}
    };

    int dir = transition_table[prev_state][enc_state];
    if (dir != 0) {
        step += dir;
        if (step >= 4)       { encoder_count++; step = 0; }
        else if (step <= -4) { encoder_count--; step = 0; }
    }
    prev_state = enc_state;
}

/* -- Input task: button monitoring -------------------------- */
static void input_task(void *arg)
{
    while (1) {
        int btn     = gpio_get_level(BTN_PIN);
        int enc_btn = gpio_get_level(ROT_ENC_BTN_PIN);
        if (btn == 0 && last_btn_state == 1) {
            btn_pressed = true;
        }
        last_btn_state = btn;
        if (enc_btn == 0 && last_enc_btn_state == 1) {
            if (app_mode == MODE_READER) {
                scroll_mode = (scroll_mode + 1) % NUM_SCROLL_MODES;
                status_dirty = true;
                ESP_LOGI(TAG, "Scroll mode: %s", scroll_labels[scroll_mode]);
            } else if (app_mode == MODE_PICKER) {
                picker_confirm = true;
            } else if (app_mode == MODE_SETTINGS) {
                settings_confirm = true;
            }
        }
        last_enc_btn_state = enc_btn;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* -- Draw one text row and push to display ------------------ */
static void draw_row_ex(int display_row, const char *text, int len,
                        uint16_t fg, uint16_t bg)
{
    uint16_t *buf = row_bufs[cur_buf];

    for (int i = 0; i < LCD_H_RES * CELL_H; i++) buf[i] = bg;

    int chars = len < COLS ? len : COLS;
    for (int col = 0; col < chars; col++) {
        uint8_t c = (uint8_t)text[col];
        if (c < 0x20 || c > 0x7E) c = 0x20;
        const uint8_t *glyph = font8[c - 0x20];
        for (int gy = 0; gy < FONT_H; gy++) {
            uint8_t bits = glyph[gy];
            for (int gx = 0; gx < FONT_W; gx++) {
                uint16_t px = (bits & (1u << gx)) ? fg : bg;
                buf[(LINE_PAD + gy) * LCD_H_RES + col * CELL_W + gx] = px;
            }
        }
    }

    int y0 = display_row * CELL_H;
    esp_lcd_panel_draw_bitmap(panel_handle, 0, y0, LCD_H_RES, y0 + CELL_H, buf);
    cur_buf ^= 1;   /* flip to the other buffer for the next call */
}

static void draw_row(int display_row, const char *text, int len)
{
    draw_row_ex(display_row, text, len, COL_FG, COL_BG);
}

/* -- Draw just the status bar ------------------------------- */
static void draw_status_bar(void)
{
    char status[COLS + 1];
    int cur_line = top_line + 1;
    const char *mode = scroll_labels[scroll_mode];
    snprintf(status, sizeof(status), " %d/%d  [%s]", cur_line, total_lines, mode);
    draw_row_ex(TEXT_ROWS, status, (int)strlen(status), COL_BG, COL_FG);
}

/* -- Render current view to display ------------------------- */
static void render_view(void)
{
    /* One fseek to the first visible line, then one fread covering all rows.
     * SPIFFS fseek is O(file_size), so minimising seek count is critical. */
    static char vbuf[4096];
    int got = 0, start_off = 0;

    if (file_fd && total_lines > 0) {
        int last = top_line + TEXT_ROWS - 1;
        if (last >= total_lines) last = total_lines - 1;
        start_off   = lines[top_line].offset;
        int end_off = lines[last].offset + lines[last].len;
        int rd_len  = end_off - start_off;
        if (rd_len > 0 && rd_len <= (int)sizeof(vbuf)) {
            fseek(file_fd, start_off, SEEK_SET);
            got = (int)fread(vbuf, 1, (size_t)rd_len, file_fd);
        }
    }

    for (int r = 0; r < TEXT_ROWS; r++) {
        int idx = top_line + r;
        if (idx < total_lines) {
            int buf_off = lines[idx].offset - start_off;
            int len     = lines[idx].len;
            if (len == 0) {
                draw_row(r, "", 0);
            } else if (buf_off >= 0 && buf_off + len <= got) {
                draw_row(r, vbuf + buf_off, len);
            } else {
                draw_row(r, "", 0);
            }
        } else {
            draw_row(r, "", 0);
        }
    }
    draw_status_bar();
}

/* -- Find wrap point for word-wrapping ---------------------- */
/* Returns the length of the next virtual line segment.       */
/* Tries to break at space/tab after a word, or after '-'.    */
/* Falls back to hard wrap if no break point is found.        */
static int find_wrap(const char *s, int len)
{
    if (len <= COLS) return len;

    /* Scan backwards from COLS to find a break point */
    for (int i = COLS; i > 0; i--) {
        char c = s[i];
        /* Break before a space/tab (the space starts the next line and
           will be consumed below, so it won't dangle). */
        if (c == ' ' || c == '\t') return i;
        /* Break after a hyphen/dash that isn't at position 0 */
        if ((s[i - 1] == '-') && i > 1) return i;
    }
    return COLS;   /* no good break point – hard wrap */
}

/* -- Parse file into word-wrapped virtual lines (streaming) - */
static void parse_lines(void)
{
    if (!file_fd) return;
    rewind(file_fd);

    int capacity = 512;
    lines = malloc((size_t)capacity * sizeof(Line));
    if (!lines) { ESP_LOGE(TAG, "lines malloc failed"); return; }
    total_lines = 0;

#define ADD_LINE(off, slen) do { \
    if (total_lines >= capacity) { \
        capacity *= 2; \
        Line *_t = realloc(lines, (size_t)capacity * sizeof(Line)); \
        if (!_t) { ESP_LOGE(TAG, "realloc failed"); goto parse_done; } \
        lines = _t; \
    } \
    lines[total_lines++] = (Line){ (off), (slen) }; \
} while (0)

    /* Read sequentially — never seek. SPIFFS fseek rescans from the start
     * each call making N seeks O(N²). One forward pass is O(N). */
    char   win[COLS + 2]; /* sliding window of current physical-line chars */
    int    win_len = 0;   /* valid bytes in win                            */
    int    win_off = 0;   /* absolute file offset of win[0]               */
    int    abs_off = 0;   /* absolute file offset of next byte to read    */
    char   rdbuf[256];
    int    n;

    while ((n = (int)fread(rdbuf, 1, sizeof(rdbuf), file_fd)) > 0) {
        for (int i = 0; i < n; i++, abs_off++) {
            char c = rdbuf[i];
            if (c == '\n') {
                if (win_len == 0) {
                    ADD_LINE(win_off, 0);   /* blank line */
                } else {
                    int off = 0;
                    while (off < win_len) {
                        int seg = find_wrap(win + off, win_len - off);
                        ADD_LINE(win_off + off, seg);
                        off += seg;
                        while (off < win_len && (win[off] == ' ' || win[off] == '\t')) off++;
                    }
                }
                win_off = abs_off + 1;
                win_len = 0;
            } else {
                win[win_len++] = c;
                if (win_len == COLS + 1) {
                    /* Window full — emit one segment, keep remainder */
                    int seg = find_wrap(win, win_len);
                    ADD_LINE(win_off, seg);
                    int skip = seg;
                    while (skip < win_len && (win[skip] == ' ' || win[skip] == '\t')) skip++;
                    win_off += skip;
                    win_len -= skip;
                    if (win_len > 0) memmove(win, win + skip, (size_t)win_len);
                }
            }
        }
    }

    /* Flush last line (no trailing newline) */
    if (win_len > 0) {
        int off = 0;
        while (off < win_len) {
            int seg = find_wrap(win + off, win_len - off);
            ADD_LINE(win_off + off, seg);
            off += seg;
            while (off < win_len && (win[off] == ' ' || win[off] == '\t')) off++;
        }
    }

parse_done:
#undef ADD_LINE
    ESP_LOGI(TAG, "Parsed %d virtual lines (sequential)", total_lines);
}

/* -- Scan all files from SPIFFS into file_list[] ------------ */
static void scan_spiffs_files(void)
{
    file_count = 0;
    DIR *dir = opendir("/spiffs");
    if (!dir) { ESP_LOGE(TAG, "opendir /spiffs failed"); return; }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && file_count < MAX_FILES) {
        if (entry->d_name[0] == '.') continue;  /* skip . and hidden */
        snprintf(file_list[file_count], MAX_PATH_LEN, "/spiffs/%s", entry->d_name);
        file_count++;
    }
    closedir(dir);
    ESP_LOGI(TAG, "Found %d file(s) on SPIFFS", file_count);
}

/* -- Draw the file picker menu ------------------------------ */
static void draw_picker_view(void)
{
    /* Title bar */
    const char *title = "  -- SELECT FILE --  ";
    draw_row_ex(0, title, (int)strlen(title), COL_BG, COL_FG);

    /* File entry rows */
    for (int r = 0; r < PICKER_VISIBLE; r++) {
        int fi = picker_top + r;
        if (fi < file_count) {
            const char *base = strrchr(file_list[fi], '/');
            base = base ? base + 1 : file_list[fi];
            if (fi == picker_sel)
                draw_row_ex(r + 1, base, (int)strlen(base), COL_BG, COL_FG);
            else
                draw_row(r + 1, base, (int)strlen(base));
        } else {
            draw_row(r + 1, "", 0);
        }
    }

    /* Status/hint bar */
    const char *hint = "ENC:sel BTN:settings PUSH:open";
    draw_row_ex(TEXT_ROWS, hint, (int)strlen(hint), COL_BG, COL_FG);
}

/* -- Settings view ------------------------------------------ */
static void wifi_ap_start(void);  /* defined in Wi-Fi section */
static void wifi_ap_stop(void);

static void draw_settings_view(void)
{
    char row[COLS + 1];

    draw_row_ex(0, "       SETTINGS        ", 23, COL_BG, COL_FG);

    snprintf(row, sizeof(row), " Wi-Fi File Transfer [%s]",
             wifi_active ? "ON " : "OFF");
    if (settings_sel == 0)
        draw_row_ex(1, row, (int)strlen(row), COL_BG, COL_FG);
    else
        draw_row(1, row, (int)strlen(row));

    if (wifi_active) {
        draw_row(2, "  Status: Active    ", 20);
        draw_row(3, "  SSID: ESP32-Reader", 20);
        draw_row(4, "  Pass: reader123   ", 20);
        draw_row(5, "  http://192.168.4.1", 20);
    } else {
        draw_row(2, "  Status: Off       ", 20);
        draw_row(3, "  SSID: ESP32-Reader", 20);
        draw_row(4, "  Pass: reader123   ", 20);
        draw_row(5, "", 0);
    }
    for (int r = 6; r < TEXT_ROWS; r++) draw_row(r, "", 0);

    draw_row_ex(TEXT_ROWS, "ENC:sel PUSH:toggle BTN:next",
                28, COL_BG, COL_FG);
}

/* -- Load a new file, restore saved scroll position --------- */
static void open_file(const char *path)
{
    if (file_fd) { fclose(file_fd); file_fd = NULL; }
    free(lines);
    lines       = NULL;
    total_lines = 0;
    top_line    = 0;
    strncpy(current_file, path, MAX_PATH_LEN - 1);
    current_file[MAX_PATH_LEN - 1] = '\0';
    file_fd = fopen(current_file, "r");
    if (file_fd) parse_lines();
    int saved   = load_scroll_pos(current_file);
    int max_top = total_lines > TEXT_ROWS ? total_lines - TEXT_ROWS : 0;
    if (saved > max_top) saved = max_top;
    if (saved < 0)       saved = 0;
    top_line      = saved;
    encoder_count = 0;
}

/* -- Reader task: poll encoder and re-render on change ------ */
#define SAVE_DELAY_MS  5000   /* write NVS 5 s after last scroll */
static void reader_task(void *arg)
{
    int last_top         = -1;
    int saved_top        = -1;
    TickType_t last_change_tick = 0;
    bool       pending_save    = false;

    while (1) {
        /* ---- BTN cycles: READER -> PICKER -> SETTINGS -> READER ---- */
        if (btn_pressed) {
            btn_pressed = false;
            if (app_mode == MODE_READER) {
                save_scroll_pos(current_file, top_line);
                scan_spiffs_files();
                picker_sel = 0;
                for (int i = 0; i < file_count; i++) {
                    if (strcmp(file_list[i], current_file) == 0) { picker_sel = i; break; }
                }
                picker_top = picker_sel > (PICKER_VISIBLE / 2) ? picker_sel - (PICKER_VISIBLE / 2) : 0;
                encoder_count  = 0;
                picker_confirm = false;
                app_mode = MODE_PICKER;
                draw_picker_view();
            } else if (app_mode == MODE_PICKER) {
                encoder_count    = 0;
                settings_sel     = 0;
                settings_confirm = false;
                app_mode = MODE_SETTINGS;
                draw_settings_view();
            } else if (app_mode == MODE_SETTINGS) {
                encoder_count = 0;
                app_mode = MODE_READER;
                last_top = -1;
                render_view();
            }
        }

        /* ---- Picker mode ---- */
        if (app_mode == MODE_PICKER) {
            int delta = encoder_count;
            if (delta != 0) {
                encoder_count = 0;
                picker_sel += delta;
                if (picker_sel < 0)           picker_sel = 0;
                if (picker_sel >= file_count) picker_sel = file_count - 1;
                /* Keep selected item in view */
                if (picker_sel < picker_top)
                    picker_top = picker_sel;
                if (picker_sel >= picker_top + PICKER_VISIBLE)
                    picker_top = picker_sel - PICKER_VISIBLE + 1;
                if (picker_top < 0) picker_top = 0;
                draw_picker_view();
            }
            if (picker_confirm) {
                picker_confirm = false;
                if (file_count > 0) open_file(file_list[picker_sel]);
                app_mode = MODE_READER;
                last_top = -1;
                render_view();
            }
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        /* ---- Settings mode ---- */
        if (app_mode == MODE_SETTINGS) {
            int delta = encoder_count;
            if (delta != 0) {
                encoder_count = 0;
                settings_sel += delta;
                if (settings_sel < 0)               settings_sel = 0;
                if (settings_sel >= SETTINGS_ITEMS) settings_sel = SETTINGS_ITEMS - 1;
                draw_settings_view();
            }
            if (settings_confirm) {
                settings_confirm = false;
                if (settings_sel == 0) {
                    if (!wifi_active) wifi_ap_start();
                    else              wifi_ap_stop();
                    draw_settings_view();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }

        /* ---- Reader mode ---- */
        int step = scroll_steps[scroll_mode];
        if (step == 0) step = TEXT_ROWS;  /* page mode */
        int max_top = total_lines > TEXT_ROWS ? total_lines - TEXT_ROWS : 0;

        /* Consume encoder delta */
        int delta = encoder_count;
        if (delta != 0) {
            encoder_count = 0;
            int new_top = top_line + delta * step;
            if (new_top < 0)       new_top = 0;
            if (new_top > max_top) new_top = max_top;
            top_line = new_top;
        }

        if (top_line != last_top) {
            render_view();
            last_top = top_line;
            last_change_tick = xTaskGetTickCount();
            pending_save = true;
        }
        /* Redraw status bar if scroll mode changed */
        if (status_dirty) {
            draw_status_bar();
            status_dirty = false;
        }
        /* Debounced NVS save */
        if (pending_save &&
            (xTaskGetTickCount() - last_change_tick) >= pdMS_TO_TICKS(SAVE_DELAY_MS)) {
            if (top_line != saved_top) {
                save_scroll_pos(current_file, top_line);
                saved_top = top_line;
            }
            pending_save = false;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ===== Wi-Fi AP + HTTP File Manager ========================= */

#define WIFI_AP_SSID    "ESP32-Reader"
#define WIFI_AP_PASS    "reader123"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_STA 3

/* Minimal single-page file manager, served at http://192.168.4.1 */
static const char index_html[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>ESP32 File Manager</title>"
    "<style>"
    "*{box-sizing:border-box}"
    "body{font-family:sans-serif;max-width:650px;margin:30px auto;padding:0 15px;background:#f5f5f5}"
    "h2{color:#2c3e50}"
    "table{width:100%;border-collapse:collapse;background:#fff;box-shadow:0 1px 3px rgba(0,0,0,.1)}"
    "th{background:#2c3e50;color:#fff;padding:10px}"
    "td{padding:8px 10px;border-bottom:1px solid #eee}"
    ".btn{padding:5px 12px;border:none;border-radius:4px;cursor:pointer;font-size:13px}"
    ".dl{background:#3498db;color:#fff}"
    ".del{background:#e74c3c;color:#fff}"
    ".box{background:#fff;padding:20px;border-radius:6px;box-shadow:0 1px 3px rgba(0,0,0,.1);margin-top:20px}"
    "#st{margin-top:8px;font-weight:bold}"
    "</style></head><body>"
    "<h2>ESP32 File Manager</h2>"
    "<table><thead><tr><th>File</th><th>Size</th><th>Download</th><th>Delete</th></tr></thead>"
    "<tbody id=\"tb\"></tbody></table>"
    "<div class=\"box\"><h3 style=\"margin-top:0\">Upload File</h3>"
    "<input type=\"file\" id=\"fi\">"
    "<button class=\"btn dl\" onclick=\"upload()\">Upload</button>"
    "<div id=\"st\"></div></div>"
    "<script>"
    "function sz(b){return b<1024?b+' B':b<1048576?(b/1024).toFixed(1)+' KB':(b/1048576).toFixed(2)+' MB';}"
    "function refresh(){"
    "fetch('/list').then(r=>r.json()).then(files=>{"
    "var t=document.getElementById('tb');t.innerHTML='';"
    "files.forEach(f=>{"
    "var r=t.insertRow();"
    "r.insertCell().textContent=f.name;"
    "r.insertCell().textContent=sz(f.size);"
    "var a=document.createElement('a');"
    "a.href='/download?name='+encodeURIComponent(f.name);"
    "a.textContent='Download';a.className='btn dl';a.style.textDecoration='none';"
    "r.insertCell().appendChild(a);"
    "var b=document.createElement('button');"
    "b.textContent='Delete';b.className='btn del';"
    "b.onclick=function(){if(confirm('Delete '+f.name+'?'))"
    "fetch('/delete?name='+encodeURIComponent(f.name),{method:'DELETE'}).then(()=>refresh());};"
    "r.insertCell().appendChild(b);"
    "});}).catch(e=>{document.getElementById('tb').innerHTML='<tr><td colspan=4>'+e+'</td></tr>';});}"
    "function upload(){"
    "var fi=document.getElementById('fi'),st=document.getElementById('st');"
    "if(!fi.files.length){st.textContent='Select a file.';return;}"
    "var f=fi.files[0];st.style.color='#333';st.textContent='Uploading '+f.name+'...';"
    "fetch('/upload?name='+encodeURIComponent(f.name),{method:'POST',body:f})"
    ".then(r=>{if(!r.ok)throw new Error('HTTP '+r.status);return r.text();})"
    ".then(msg=>{st.style.color='green';st.textContent=msg;fi.value='';refresh();})"
    ".catch(e=>{st.style.color='red';st.textContent='Error: '+e;});}"
    "refresh();</script></body></html>";

/* Decode a percent-encoded URL query value */
static void url_decode(const char *src, char *dst, size_t dst_sz)
{
    size_t i = 0, j = 0;
    while (src[i] && j < dst_sz - 1) {
        if (src[i] == '%' && src[i+1] && src[i+2]) {
            char hex[3] = { src[i+1], src[i+2], '\0' };
            dst[j++] = (char)strtol(hex, NULL, 16);
            i += 3;
        } else if (src[i] == '+') {
            dst[j++] = ' ';
            i++;
        } else {
            dst[j++] = src[i++];
        }
    }
    dst[j] = '\0';
}

/* Reject names that could cause path traversal */
static bool valid_filename(const char *name)
{
    if (!name || name[0] == '\0' || name[0] == '.') return false;
    for (const char *p = name; *p; p++) {
        if (*p == '/' || *p == '\\') return false;
    }
    return true;
}

/* GET / — file manager HTML */
static esp_err_t http_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, index_html);
    return ESP_OK;
}

/* GET /list — JSON array [{name,size},...] */
static esp_err_t http_list_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "[");
    DIR *dir = opendir("/spiffs");
    if (dir) {
        bool first = true;
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_name[0] == '.') continue;
            char path[MAX_PATH_LEN];
            snprintf(path, sizeof(path), "/spiffs/%s", entry->d_name);
            struct stat st;
            stat(path, &st);
            char row[320];
            snprintf(row, sizeof(row), "%s{\"name\":\"%s\",\"size\":%ld}",
                     first ? "" : ",", entry->d_name, (long)st.st_size);
            httpd_resp_sendstr_chunk(req, row);
            first = false;
        }
        closedir(dir);
    }
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

/* GET /download?name=<filename> — stream file */
static esp_err_t http_download_handler(httpd_req_t *req)
{
    char qs[256], raw[256], name[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK ||
        httpd_query_key_value(qs, "name", raw, sizeof(raw)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name");
        return ESP_FAIL;
    }
    url_decode(raw, name, sizeof(name));
    if (!valid_filename(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "/spiffs/%s", name);
    FILE *f = fopen(path, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/octet-stream");
    char disp[320];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", name);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);
    char *buf = malloc(1024);
    if (!buf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    size_t n;
    while ((n = fread(buf, 1, 1024, f)) > 0) {
        httpd_resp_send_chunk(req, buf, (ssize_t)n);
    }
    free(buf);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* DELETE /delete?name=<filename> — remove file */
static esp_err_t http_delete_handler(httpd_req_t *req)
{
    char qs[256], raw[256], name[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK ||
        httpd_query_key_value(qs, "name", raw, sizeof(raw)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name");
        return ESP_FAIL;
    }
    url_decode(raw, name, sizeof(name));
    if (!valid_filename(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "/spiffs/%s", name);
    if (remove(path) != 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Delete failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* POST /upload?name=<filename> — write body to SPIFFS */
static esp_err_t http_upload_handler(httpd_req_t *req)
{
    char qs[256], raw[256], name[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK ||
        httpd_query_key_value(qs, "name", raw, sizeof(raw)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing name");
        return ESP_FAIL;
    }
    url_decode(raw, name, sizeof(name));
    if (!valid_filename(name)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }
    char path[MAX_PATH_LEN];
    snprintf(path, sizeof(path), "/spiffs/%s", name);
    FILE *f = fopen(path, "w");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot create file");
        return ESP_FAIL;
    }
    char *buf = malloc(1024);
    if (!buf) {
        fclose(f); remove(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    size_t remaining = req->content_len;
    bool ok = true;
    while (remaining > 0) {
        int chunk = (int)(remaining > 1024 ? 1024 : remaining);
        int got   = httpd_req_recv(req, buf, chunk);
        if (got <= 0) { ok = false; break; }
        if ((size_t)fwrite(buf, 1, (size_t)got, f) != (size_t)got) { ok = false; break; }
        remaining -= (size_t)got;
    }
    free(buf);
    fclose(f);
    if (!ok) {
        remove(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Transfer error");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Upload complete");
    return ESP_OK;
}

static httpd_handle_t http_server_handle = NULL;

static void wifi_system_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = sizeof(WIFI_AP_SSID) - 1,
            .channel        = WIFI_AP_CHANNEL,
            .password       = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_STA,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_LOGI(TAG, "Wi-Fi driver ready (AP off; enable via Settings)");
}

static void wifi_ap_start(void)
{
    if (wifi_active) return;
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size     = 8192;
    ESP_ERROR_CHECK(httpd_start(&http_server_handle, &cfg));

    const httpd_uri_t uris[] = {
        { .uri = "/",         .method = HTTP_GET,    .handler = http_root_handler     },
        { .uri = "/list",     .method = HTTP_GET,    .handler = http_list_handler     },
        { .uri = "/download", .method = HTTP_GET,    .handler = http_download_handler },
        { .uri = "/delete",   .method = HTTP_DELETE, .handler = http_delete_handler   },
        { .uri = "/upload",   .method = HTTP_POST,   .handler = http_upload_handler   },
    };
    for (int i = 0; i < 5; i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &uris[i]));
    }
    wifi_active = true;
    ESP_LOGI(TAG, "Wi-Fi AP on  SSID=\"%s\" -> http://192.168.4.1", WIFI_AP_SSID);
}

static void wifi_ap_stop(void)
{
    if (!wifi_active) return;
    httpd_stop(http_server_handle);
    http_server_handle = NULL;
    ESP_ERROR_CHECK(esp_wifi_stop());
    wifi_active = false;
    ESP_LOGI(TAG, "Wi-Fi AP off");
}

/* ===== end Wi-Fi / HTTP ===================================== */

/* -- app_main ----------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting reader");

    /* NVS init */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 0. GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROT_ENC_A_PIN) | (1ULL << ROT_ENC_B_PIN) |
                        (1ULL << ROT_ENC_BTN_PIN) | (1ULL << BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_intr_type(ROT_ENC_A_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ROT_ENC_B_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ROT_ENC_A_PIN, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ROT_ENC_B_PIN, encoder_isr_handler, NULL);
    xTaskCreate(input_task, "input", 2048, NULL, 4, NULL);

    /* 1. SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = SCREEN_1_SDA_PIN,
        .miso_io_num     = -1,
        .sclk_io_num     = SCREEN_1_SCL_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * CELL_H * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 2. Panel IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num       = SCREEN_1_DC_PIN,
        .cs_gpio_num       = SCREEN_1_CS_PIN,
        .pclk_hz           = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io_handle));

    /* 3. ST7789 */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = SCREEN_1_RES_PIN,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "ST7789 ready");

    /* 4. SPIFFS + file */
    esp_vfs_spiffs_conf_t spiffs_conf = {
        .base_path              = "/spiffs",
        .partition_label        = "storage",
        .max_files              = 5,
        .format_if_mount_failed = false,
    };
    /* Allocate double row buffers in DMA-capable memory */
    size_t buf_sz = LCD_H_RES * CELL_H * sizeof(uint16_t);
    row_bufs[0] = heap_caps_malloc(buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    row_bufs[1] = heap_caps_malloc(buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(row_bufs[0] && row_bufs[1]);

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));
    file_fd = fopen(current_file, "r");
    if (file_fd) parse_lines();

    /* 5. Wi-Fi driver init (AP starts on demand via Settings screen) */
    wifi_system_init();

    /* 6. Restore scroll position + initial render + reader task */
    int saved = load_scroll_pos(current_file);
    int max_top = total_lines > TEXT_ROWS ? total_lines - TEXT_ROWS : 0;
    if (saved > max_top) saved = max_top;
    if (saved < 0) saved = 0;
    encoder_count = 0;
    top_line = saved;
    render_view();
    xTaskCreate(reader_task, "reader", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Ready");
}
