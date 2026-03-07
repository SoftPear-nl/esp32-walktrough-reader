/**
 * @file lv_conf.h
 * LVGL v9 configuration for ST7789 320x240 display on ESP32-C6
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/* Color depth: 16 (RGB565) matches ST7789 native format */
#define LV_COLOR_DEPTH 16

/*=========================
   MEMORY SETTINGS
 *=========================*/

/* Size of the memory available for `lv_malloc()` in bytes (>= 2kB) */
#define LV_MEM_SIZE (48U * 1024U)

/* Use custom malloc/free from libc */
#define LV_STDLIB_INCLUDE <stdlib.h>
#define LV_MALLOC       malloc
#define LV_REALLOC      realloc
#define LV_FREE         free

/*===================
   HAL SETTINGS
 *===================*/

/* Default display refresh period in milliseconds */
#define LV_DEF_REFR_PERIOD 10

/* Input device read period in milliseconds */
#define LV_INDEV_DEF_READ_PERIOD 30

/* Use a custom tick source — disabled; lv_tick_set_cb() is called in app_main instead */
#define LV_TICK_CUSTOM 0

/*=======================
   FEATURE CONFIGURATION
 *=======================*/

/* Enable logging */
#define LV_USE_LOG 1
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF 1
#endif

/* Assertion handling */
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/*=================
   DRAW BACKEND
 *=================*/

#define LV_USE_DRAW_SW 1

/*==================
   BUILT-IN THEMES
 *==================*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK 0
    #define LV_THEME_DEFAULT_GROW 1
#endif

/*=================
   FONT USAGE
 *=================*/
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_12

/*=================
   WIDGET USAGE
 *=================*/
#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BUTTON     1
#define LV_USE_BUTTONMATRIX 1
#define LV_USE_CANVAS     1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMAGE      1
#define LV_USE_LABEL      1
#define LV_USE_LINE       1
#define LV_USE_ROLLER     1
#define LV_USE_SCALE      1
#define LV_USE_SLIDER     1
#define LV_USE_SPINNER    1
#define LV_USE_SWITCH     1
#define LV_USE_TABLE      1
#define LV_USE_TEXTAREA   1

/*==================
    EXTRA LAYOUTS
 *==================*/
#define LV_USE_FLEX   1
#define LV_USE_GRID   1

#endif /* LV_CONF_H */
