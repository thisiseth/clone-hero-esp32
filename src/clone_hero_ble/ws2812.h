//compilation of espressif ws2812 rmt driver into a single package
//espressif code licence:
/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "esp_idf_version.h"

#if !CONFIG_SOC_RMT_SUPPORTED
  #error no RMT support
#endif
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
  #error esp-idf < 5.0.0
#endif

#ifdef __cplusplus
extern "C" {
#endif

//
//led_strip_types.h
//

#include <stdint.h>

/**
 * @brief LED strip pixel format
 */
typedef enum {
    LED_PIXEL_FORMAT_GRB,    /*!< Pixel format: GRB */
    LED_PIXEL_FORMAT_GRBW,   /*!< Pixel format: GRBW */
    LED_PIXEL_FORMAT_INVALID /*!< Invalid pixel format */
} led_pixel_format_t;

/**
 * @brief LED strip model
 * @note Different led model may have different timing parameters, so we need to distinguish them.
 */
typedef enum {
    LED_MODEL_WS2812, /*!< LED strip model: WS2812 */
    LED_MODEL_SK6812, /*!< LED strip model: SK6812 */
    LED_MODEL_INVALID /*!< Invalid LED strip model */
} led_model_t;

/**
 * @brief LED strip handle
 */
typedef struct led_strip_t *led_strip_handle_t;

/**
 * @brief LED Strip Configuration
 */
typedef struct {
    int strip_gpio_num;      /*!< GPIO number that used by LED strip */
    uint32_t max_leds;       /*!< Maximum LEDs in a single strip */
    led_pixel_format_t led_pixel_format; /*!< LED pixel format */
    led_model_t led_model;   /*!< LED model */

    struct {
        uint32_t invert_out: 1; /*!< Invert output signal */
    } flags;                    /*!< Extra driver flags */
} led_strip_config_t;

//
//led_strip_rmt.h
//

#include <stdint.h>
#include "esp_err.h"
//#include "led_strip_types.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/rmt_types.h"
#endif

/**
 * @brief LED Strip RMT specific configuration
 */
typedef struct {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    uint8_t rmt_channel;        /*!< Specify the channel number, the legacy RMT driver doesn't support channel allocator */
#else // new driver supports specify the clock source and clock resolution
    rmt_clock_source_t clk_src; /*!< RMT clock source */
    uint32_t resolution_hz;     /*!< RMT tick resolution, if set to zero, a default resolution (10MHz) will be applied */
#endif
    size_t mem_block_symbols;   /*!< How many RMT symbols can one RMT channel hold at one time. Set to 0 will fallback to use the default size. */
    struct {
        uint32_t with_dma: 1;   /*!< Use DMA to transmit data */
    } flags;                    /*!< Extra driver flags */
} led_strip_rmt_config_t;

/**
 * @brief Create LED strip based on RMT TX channel
 *
 * @param led_config LED strip configuration
 * @param rmt_config RMT specific configuration
 * @param ret_strip Returned LED strip handle
 * @return
 *      - ESP_OK: create LED strip handle successfully
 *      - ESP_ERR_INVALID_ARG: create LED strip handle failed because of invalid argument
 *      - ESP_ERR_NO_MEM: create LED strip handle failed because of out of memory
 *      - ESP_FAIL: create LED strip handle failed because some other error
 */
esp_err_t led_strip_new_rmt_device(const led_strip_config_t *led_config, const led_strip_rmt_config_t *rmt_config, led_strip_handle_t *ret_strip);

//
//led_strip.h
//

#include <stdint.h>
#include "esp_err.h"
//#include "led_strip_rmt.h"
#include "esp_idf_version.h"
/**
 * @brief Set RGB for a specific pixel
 *
 * @param strip: LED strip
 * @param index: index of pixel to set
 * @param red: red part of color
 * @param green: green part of color
 * @param blue: blue part of color
 *
 * @return
 *      - ESP_OK: Set RGB for a specific pixel successfully
 *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
 *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
 */
esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

/**
 * @brief Set RGBW for a specific pixel
 *
 * @note Only call this function if your led strip does have the white component (e.g. SK6812-RGBW)
 * @note Also see `led_strip_set_pixel` if you only want to specify the RGB part of the color and bypass the white component
 *
 * @param strip: LED strip
 * @param index: index of pixel to set
 * @param red: red part of color
 * @param green: green part of color
 * @param blue: blue part of color
 * @param white: separate white component
 *
 * @return
 *      - ESP_OK: Set RGBW color for a specific pixel successfully
 *      - ESP_ERR_INVALID_ARG: Set RGBW color for a specific pixel failed because of an invalid argument
 *      - ESP_FAIL: Set RGBW color for a specific pixel failed because other error occurred
 */
esp_err_t led_strip_set_pixel_rgbw(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white);

/**
 * @brief Set HSV for a specific pixel
 *
 * @param strip: LED strip
 * @param index: index of pixel to set
 * @param hue: hue part of color (0 - 360)
 * @param saturation: saturation part of color (0 - 255, rescaled from 0 - 1. e.g. saturation = 0.5, rescaled to 127)
 * @param value: value part of color (0 - 255, rescaled from 0 - 1. e.g. value = 0.5, rescaled to 127)
 *
 * @return
 *      - ESP_OK: Set HSV color for a specific pixel successfully
 *      - ESP_ERR_INVALID_ARG: Set HSV color for a specific pixel failed because of an invalid argument
 *      - ESP_FAIL: Set HSV color for a specific pixel failed because other error occurred
 */
esp_err_t led_strip_set_pixel_hsv(led_strip_handle_t strip, uint32_t index, uint16_t hue, uint8_t saturation, uint8_t value);

/**
 * @brief Refresh memory colors to LEDs
 *
 * @param strip: LED strip
 *
 * @return
 *      - ESP_OK: Refresh successfully
 *      - ESP_FAIL: Refresh failed because some other error occurred
 *
 * @note:
 *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
 */
esp_err_t led_strip_refresh(led_strip_handle_t strip);

/**
 * @brief Clear LED strip (turn off all LEDs)
 *
 * @param strip: LED strip
 *
 * @return
 *      - ESP_OK: Clear LEDs successfully
 *      - ESP_FAIL: Clear LEDs failed because some other error occurred
 */
esp_err_t led_strip_clear(led_strip_handle_t strip);

/**
 * @brief Free LED strip resources
 *
 * @param strip: LED strip
 *
 * @return
 *      - ESP_OK: Free resources successfully
 *      - ESP_FAIL: Free resources failed because error occurred
 */
esp_err_t led_strip_del(led_strip_handle_t strip);

//
//led_strip_interface.h
//

#include <stdint.h>
#include "esp_err.h"

typedef struct led_strip_t led_strip_t; /*!< Type of LED strip */

/**
 * @brief LED strip interface definition
 */
struct led_strip_t {
    /**
     * @brief Set RGB for a specific pixel
     *
     * @param strip: LED strip
     * @param index: index of pixel to set
     * @param red: red part of color
     * @param green: green part of color
     * @param blue: blue part of color
     *
     * @return
     *      - ESP_OK: Set RGB for a specific pixel successfully
     *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
     *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
     */
    esp_err_t (*set_pixel)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);

    /**
     * @brief Set RGBW for a specific pixel. Similar to `set_pixel` but also set the white component
     *
     * @param strip: LED strip
     * @param index: index of pixel to set
     * @param red: red part of color
     * @param green: green part of color
     * @param blue: blue part of color
     * @param white: separate white component
     *
     * @return
     *      - ESP_OK: Set RGBW color for a specific pixel successfully
     *      - ESP_ERR_INVALID_ARG: Set RGBW color for a specific pixel failed because of an invalid argument
     *      - ESP_FAIL: Set RGBW color for a specific pixel failed because other error occurred
     */
    esp_err_t (*set_pixel_rgbw)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white);

    /**
     * @brief Refresh memory colors to LEDs
     *
     * @param strip: LED strip
     * @param timeout_ms: timeout value for refreshing task
     *
     * @return
     *      - ESP_OK: Refresh successfully
     *      - ESP_FAIL: Refresh failed because some other error occurred
     *
     * @note:
     *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
     */
    esp_err_t (*refresh)(led_strip_t *strip);

    /**
     * @brief Clear LED strip (turn off all LEDs)
     *
     * @param strip: LED strip
     * @param timeout_ms: timeout value for clearing task
     *
     * @return
     *      - ESP_OK: Clear LEDs successfully
     *      - ESP_FAIL: Clear LEDs failed because some other error occurred
     */
    esp_err_t (*clear)(led_strip_t *strip);

    /**
     * @brief Free LED strip resources
     *
     * @param strip: LED strip
     *
     * @return
     *      - ESP_OK: Free resources successfully
     *      - ESP_FAIL: Free resources failed because error occurred
     */
    esp_err_t (*del)(led_strip_t *strip);
};

#ifdef __cplusplus
}

class Ws2812
{
  public:
    Ws2812(uint8_t gpio);
    ~Ws2812();
    void SetColor(uint8_t r, uint8_t g, uint8_t b);
  private:
    led_strip_handle_t led_handle;
};

Ws2812::Ws2812(uint8_t gpio)
{
  led_strip_config_t strip_config = 
  {
      .strip_gpio_num = gpio,        // The GPIO that connected to the LED strip's data line
      .max_leds = 1,                            // The number of LEDs in the strip,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
      .led_model = LED_MODEL_WS2812,            // LED strip model
      .flags = { false }                // whether to invert the output signal
  };

  // LED strip backend configuration: RMT
  led_strip_rmt_config_t rmt_config = 
  {
    .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
    .flags = { false },   
  };

  led_strip_new_rmt_device(&strip_config, &rmt_config, &led_handle);
}

Ws2812::~Ws2812()
{
  led_strip_del(led_handle);
}

void Ws2812::SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_handle, 0, r, g, b);
    led_strip_refresh(led_handle);
}
#endif
