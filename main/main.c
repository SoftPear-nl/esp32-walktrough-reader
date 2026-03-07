#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
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

static const char *TAG = "reader";

/* -- Pin definitions ---------------------------------------- */
#define SCREEN_1_SCL_PIN  1
#define SCREEN_1_SDA_PIN  10
#define SCREEN_1_RES_PIN  11
#define SCREEN_1_DC_PIN   12
#define SCREEN_1_CS_PIN   13
#define ROT_ENC_A_PIN     4
#define ROT_ENC_B_PIN     7
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

/* -- Virtual line (start pointer + length) ------------------ */
typedef struct { const char *start; int len; } Line;

static char  *file_buf    = NULL;
static Line  *lines       = NULL;
static int    total_lines = 0;
static int    top_line    = 0;

/* -- Current file path (for NVS key) ------------------------ */
static const char *current_file = "/spiffs/ff10.txt";

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
        if (btn == 0 && last_btn_state == 1)
            ESP_LOGI(TAG, "Button pressed");
        last_btn_state = btn;
        if (enc_btn == 0 && last_enc_btn_state == 1) {
            scroll_mode = (scroll_mode + 1) % NUM_SCROLL_MODES;
            status_dirty = true;
            ESP_LOGI(TAG, "Scroll mode: %s", scroll_labels[scroll_mode]);
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
    for (int r = 0; r < TEXT_ROWS; r++) {
        int idx = top_line + r;
        if (idx < total_lines)
            draw_row(r, lines[idx].start, lines[idx].len);
        else
            draw_row(r, "", 0);
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

/* -- Count virtual lines for one physical line -------------- */
static int count_vlines(const char *s, int len)
{
    if (len == 0) return 1;
    int n = 0;
    int off = 0;
    while (off < len) {
        int seg = find_wrap(s + off, len - off);
        n++;
        off += seg;
        /* skip leading space on the new wrapped line */
        while (off < len && (s[off] == ' ' || s[off] == '\t')) off++;
    }
    return n;
}

/* -- Parse file into word-wrapped virtual lines ------------- */
static void parse_lines(void)
{
    if (!file_buf) return;

    /* First pass: count virtual lines */
    int count = 0;
    const char *p = file_buf;
    while (*p) {
        const char *ls = p;
        while (*p && *p != '\n') p++;
        count += count_vlines(ls, (int)(p - ls));
        if (*p == '\n') p++;
    }

    lines = malloc((size_t)count * sizeof(Line));
    if (!lines) { ESP_LOGE(TAG, "lines malloc failed"); return; }

    /* Second pass: populate */
    total_lines = 0;
    p = file_buf;
    while (*p) {
        const char *ls = p;
        while (*p && *p != '\n') p++;
        int ll = (int)(p - ls);
        if (ll == 0) {
            lines[total_lines++] = (Line){ ls, 0 };
        } else {
            int off = 0;
            while (off < ll) {
                int seg = find_wrap(ls + off, ll - off);
                lines[total_lines++] = (Line){ ls + off, seg };
                off += seg;
                /* skip leading whitespace on wrapped continuation */
                while (off < ll && (ls[off] == ' ' || ls[off] == '\t')) off++;
            }
        }
        if (*p == '\n') p++;
    }
    ESP_LOGI(TAG, "Parsed %d virtual lines (word-wrapped)", total_lines);
}

/* -- Read entire file from SPIFFS into heap ----------------- */
static char *read_spiffs_file(const char *path)
{
    struct stat st;
    if (stat(path, &st) != 0) { ESP_LOGE(TAG, "stat failed: %s", path); return NULL; }
    char *buf = malloc((size_t)st.st_size + 1);
    if (!buf) { ESP_LOGE(TAG, "malloc failed"); return NULL; }
    FILE *f = fopen(path, "r");
    if (!f) { free(buf); ESP_LOGE(TAG, "fopen failed: %s", path); return NULL; }
    size_t n = fread(buf, 1, (size_t)st.st_size, f);
    fclose(f);
    buf[n] = '\0';
    ESP_LOGI(TAG, "Read %u bytes from %s", (unsigned)n, path);
    return buf;
}

/* -- Reader task: poll encoder and re-render on change ------ */
#define SAVE_DELAY_MS  5000   /* write NVS 5 s after last scroll */
static void reader_task(void *arg)
{
    int last_top   = -1;
    int saved_top  = -1;
    TickType_t last_change_tick = 0;
    bool       pending_save    = false;

    while (1) {
        int step = scroll_steps[scroll_mode];
        if (step == 0) step = TEXT_ROWS;  /* page mode */
        int max_top = total_lines > TEXT_ROWS ? total_lines - TEXT_ROWS : 0;

        /* Consume encoder delta */
        int delta = encoder_count;
        if (delta != 0) {
            encoder_count = 0;
            int new_top = top_line + delta * step;
            if (new_top < 0) new_top = 0;
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
    file_buf = read_spiffs_file(current_file);
    if (file_buf) parse_lines();

    /* 5. Restore scroll position + initial render + reader task */
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
