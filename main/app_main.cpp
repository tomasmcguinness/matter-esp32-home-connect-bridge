#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <common_macros.h>

#include <esp_matter.h>
#include <app/server/Server.h>

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <math.h>
#include "jpeg_decoder.h"

#include <string>

#include "esp_lcd_panel_ssd1681.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"

#include "lvgl.h"

#include "esp_tls.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include <lib/dnssd/Types.h>

#include "cJSON.h"

#include "lvgl.h"

#include "ssd1683.h"

static const char *TAG = "app";

#define LCD_HOST SPI2_HOST
#define PARALLEL_LINES 16
#define LCD_BK_LIGHT_ON_LEVEL 0
#define PIN_NUM_CS 10
#define PIN_NUM_DC 46
#define PIN_NUM_RST 47
#define PIN_NUM_BUSY 48

#define IMAGE_W 400
#define IMAGE_H 300

using namespace esp_matter;

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 20000

static SemaphoreHandle_t panel_refreshing_sem = NULL;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        // Clean the buffer in case of a new request
        if (output_len == 0 && evt->user_data)
        {
            // we are just starting to copy the output data into the use
            memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
        }
        /*
         *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
         *  However, event handler can also be used in case chunked encoding is used.
         */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // If user_data buffer is configured, copy the response into the buffer
            int copy_len = 0;
            if (evt->user_data)
            {
                // The last byte in evt->user_data is kept for the NULL character in case of out-of-bound access.
                copy_len = std::min(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                if (copy_len)
                {
                    memcpy(evt->user_data + output_len, evt->data, copy_len);
                }
            }
            else
            {
                int content_len = esp_http_client_get_content_length(evt->client);
                if (output_buffer == NULL)
                {
                    // We initialize output_buffer with 0 because it is used by strlen() and similar functions therefore should be null terminated.
                    output_buffer = (char *)calloc(content_len + 1, sizeof(char));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                copy_len = std::min(evt->data_len, (content_len - output_len));
                if (copy_len)
                {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                }
            }
            output_len += copy_len;
        }

        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
    {
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
            ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
        }
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
    }
    break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
        esp_http_client_set_header(evt->client, "From", "user@example.com");
        esp_http_client_set_header(evt->client, "Accept", "text/html");
        esp_http_client_set_redirection(evt->client);
        break;
    default:
        ESP_LOGE(TAG, "Unhandled event");
        break;
    }
    return ESP_OK;
}

static void start_device_flow(void *param)
{
    char *local_response_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);

    esp_http_client_config_t config = {
        .url = "https://api.home-connect.com/security/oauth/device_authorization",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

    const char *post_data = "client_id=EAEC454A48CC2D7B95D43D5776F6049114E286CFA1D16E3D494AC67CF418F3E6";
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));

        cJSON *root = cJSON_Parse(local_response_buffer);

        if (root == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON");
            return;
        }

        const cJSON *deviceCodeJSON = cJSON_GetObjectItemCaseSensitive(root, "deviceCode");
        // ESP_LOGI(TAG, "deviceCode: %d", countJSON->valuedouble);

        const cJSON *nextJSON = cJSON_GetObjectItemCaseSensitive(root, "user_code");
        ESP_LOGI(TAG, "user_code: %s", nextJSON->valuestring);

        const cJSON *verificationUriJSON = cJSON_GetObjectItemCaseSensitive(root, "verification_uri");
        ESP_LOGI(TAG, "verification_uri: %s", verificationUriJSON->valuestring);

        const cJSON *completeVerificationUriJSON = cJSON_GetObjectItemCaseSensitive(root, "verification_uri_complete");
        ESP_LOGI(TAG, "complete verification_uri: %s", completeVerificationUriJSON->valuestring);

        // We now need to wait for the user to authorize.
        //
        esp_http_client_set_url(client, "https://api.home-connect.com/security/oauth/token");
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

        const char *post_data = "grant_type=device_code&device_code=&client_id=EAEC454A48CC2D7B95D43D5776F6049114E286CFA1D16E3D494AC67CF418F3E6";
        esp_http_client_set_post_field(client, post_data, strlen(post_data));

        while (1)
        {
            esp_err_t err = esp_http_client_perform(client);

            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));
            }
            else
            {
                ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
            }

            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
    else
    {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kESPSystemEvent:
        ESP_LOGI(TAG, "kESPSystemEvent");

        if (event->Platform.ESPSystemEvent.Base == IP_EVENT &&
            event->Platform.ESPSystemEvent.Id == IP_EVENT_STA_GOT_IP)
        {
            // Have an IP address.
            // Start the OAuth flow.
            //
            xTaskCreate(start_device_flow, "StartDeviceFlow", 4 * 1024, NULL, 5, NULL);
        }
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    return err;
}

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 8;             // Command is 8 bits
    t.tx_buffer = &cmd;       // The data is the cmd itself
    t.user = (void *)0;       // D/C needs to be set to 0
    // if (keep_cs_active)
    // {
    //     t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer
    // }
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

void lcd_data(spi_device_handle_t spi, const uint8_t data, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = 8;                               // Command is 8 bits
    t.tx_buffer = &data;                        // The data is the cmd itself
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

// set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level((gpio_num_t)PIN_NUM_DC, dc);
}

#define EXAMPLE_LVGL_DRAW_BUF_LINES 20
#define EXAMPLE_LCD_H_RES 400
#define EXAMPLE_LCD_V_RES 300
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

static _lock_t lvgl_api_lock;

#include <sys/param.h>

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, EXAMPLE_LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, EXAMPLE_LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv)
{
    xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
}

static void example_lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = (lv_display_rotation_t)lv_display_get_rotation(disp);

    switch (rotation)
    {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static uint8_t *converted_buffer_black;
static uint8_t *converted_buffer_red;

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    example_lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // node::config_t node_config;

    // node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    // ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // err = esp_matter::start(app_event_cb);
    //  ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    ESP_LOGI(TAG, "Setting up SPI...");

    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .sclk_io_num = 12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));






    // ESP_LOGI(TAG, "Install panel IO");
    // esp_lcd_panel_io_handle_t io_handle = NULL;
    // esp_lcd_panel_io_spi_config_t io_config = {
    //     .cs_gpio_num = PIN_NUM_CS,
    //     .dc_gpio_num = PIN_NUM_DC,
    //     .spi_mode = 0,
    //     .pclk_hz = 1000000,
    //     .trans_queue_depth = 10,
    //     .lcd_cmd_bits = 8,
    //     .lcd_param_bits = 8,
    // };
    // // Attach the LCD to the SPI bus
    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle));

    // // esp_lcd_panel_handle_t panel_handle = NULL;
    // // esp_lcd_panel_dev_config_t panel_config = {
    // //     .reset_gpio_num = PIN_NUM_RST,
    // //     .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
    // //     .bits_per_pixel = 16,
    // // };
    // // ESP_LOGI(TAG, "Install ST7789 panel driver");
    // // ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    // // ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    // // ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // gpio_set_level((gpio_num_t)7, true);
    // ESP_LOGI(TAG, "Display powered up...");

    // ESP_LOGI(TAG, "Creating SSD1681 panel...");
    // esp_lcd_ssd1681_config_t epaper_ssd1681_config = {
    //     .busy_gpio_num = PIN_NUM_BUSY,
    //     // NOTE: Enable this to reduce one buffer copy if you do not use swap-xy, mirror y or invert color
    //     // since those operations are not supported by ssd1681 and are implemented by software
    //     // Better use DMA-capable memory region, to avoid additional data copy
    //     .non_copy_mode = false,
    // };
    // esp_lcd_panel_dev_config_t panel_config = {
    //     .reset_gpio_num = PIN_NUM_RST,
    //     .flags = {
    //         .reset_active_high = false,
    //     },
    //     .vendor_config = &epaper_ssd1681_config};
    // esp_lcd_panel_handle_t panel_handle = NULL;
    // // NOTE: Please call gpio_install_isr_service() manually before esp_lcd_new_panel_ssd1681()
    // // because gpio_isr_handler_add() is called in esp_lcd_new_panel_ssd1681()
    // gpio_install_isr_service(0);
    // ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1681(io_handle, &panel_config, &panel_handle));

    // ESP_LOGI(TAG, "Resetting e-Paper display...");
    // ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "Initializing e-Paper display...");
    // ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "Turning e-Paper display on...");
    // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // vTaskDelay(100 / portTICK_PERIOD_MS);

    // static lv_disp_t *disp_handle;

    // ESP_LOGI(TAG, "Initializing LVGL");
    // const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    // ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // ESP_LOGI(TAG, "Adding Display to LVGL");
    // const lvgl_port_display_cfg_t disp_cfg = {
    //     .io_handle = io_handle,
    //     .panel_handle = panel_handle,
    //     .buffer_size = IMAGE_H * IMAGE_W,
    //     .double_buffer = false,
    //     .trans_size = 8000,
    //     .hres = IMAGE_H,
    //     .vres = IMAGE_W,
    //     .monochrome = true,
    //     .rotation = {
    //         .swap_xy = false,
    //         .mirror_x = false,
    //         .mirror_y = false,
    //     },
    //     .color_format = LV_COLOR_FORMAT_I1,
    //     .flags = {
    //         .buff_dma = false,
    //         .buff_spiram = true,
    //     }};
    // disp_handle = lvgl_port_add_disp(&disp_cfg);

    // ESP_LOGI(TAG, "Getting active screen");
    // lv_obj_t *scr = lv_display_get_screen_active(disp_handle);

    // ESP_LOGI(TAG, "Adding label");
    // lv_obj_t *lbl = lv_label_create(scr);
    // lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH " ROTATE");

    // ESP_LOGI(TAG, "Initialize LVGL library");
    // lv_init();

    // // create a lvgl display
    // lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    // // alloc draw buffers used by LVGL
    // // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    // size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    // void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    // assert(buf1);
    // void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    // assert(buf2);
    // // initialize LVGL draw buffers
    // lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // // associate the mipi panel handle to the display
    // lv_display_set_user_data(display, panel_handle);
    // // set color depth
    // lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // // set the callback which can copy the rendered image to an area of the display
    // lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    // ESP_LOGI(TAG, "Install LVGL tick timer");
    // // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    // const esp_timer_create_args_t lvgl_tick_timer_args = {
    //     .callback = &example_increase_lvgl_tick,
    //     .name = "lvgl_tick"};
    // esp_timer_handle_t lvgl_tick_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2 * 1000));

    // ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    // const esp_lcd_panel_io_callbacks_t cbs = {
    //     .on_color_trans_done = example_notify_lvgl_flush_ready,
    // };
    // /* Register done callback */
    // ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    // ESP_LOGI(TAG, "Create LVGL task");
    // xTaskCreate(example_lvgl_port_task, "LVGL", 4048, NULL, 5, NULL);

    // ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // // Lock the mutex due to the LVGL APIs are not thread-safe
    // _lock_acquire(&lvgl_api_lock);

    // lv_obj_t *scr = lv_display_get_screen_active(display);

    // lv_obj_t *lbl = lv_label_create(scr);
    // lv_label_set_text_static(lbl, LV_SYMBOL_REFRESH " ROTATE");

    // _lock_release(&lvgl_api_lock);

    // err = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    // ESP_ERROR_CHECK(err);

    // gpio_set_level((gpio_num_t)7, true);

    // ESP_LOGI(TAG, "Display powered up...");

    // spi_device_acquire_bus(spi, portMAX_DELAY);

    // gpio_config_t io_conf = {};
    // io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST));
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // gpio_config(&io_conf);

    // io_conf = {};
    // io_conf.pin_bit_mask = ((1ULL << PIN_NUM_BUSY));
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // gpio_config(&io_conf);

    // // Init the display
    // //
    // gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // gpio_set_level((gpio_num_t)PIN_NUM_RST, false);
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // gpio_set_level((gpio_num_t)PIN_NUM_RST, true);
    // vTaskDelay(10 / portTICK_PERIOD_MS);

    // ESP_LOGI(TAG, "Waiting for reset to finish...");

    // while (1)
    // {
    //     int level = gpio_get_level((gpio_num_t)PIN_NUM_BUSY);

    //     if (level == 0)
    //     {
    //         break;
    //     }
    // }

    // ESP_LOGI(TAG, "Reset complete. ");

    // lcd_cmd(spi, 0x12, true);

    // while (1)
    // {
    //     int level = gpio_get_level((gpio_num_t)PIN_NUM_BUSY);

    //     if (level == 0)
    //     {
    //         break;
    //     }
    // }

    // ESP_LOGI(TAG, "Soft reset complete");

    // lcd_cmd(spi, 0x21, true);
    // lcd_data(spi, 0x40, true);
    // lcd_data(spi, 0x00, true);
    // lcd_cmd(spi, 0x3C, true);
    // lcd_data(spi, 0x05, true);
    // lcd_cmd(spi, 0x11, true);
    // lcd_data(spi, 0x03, true);

    // lcd_cmd(spi, 0x44, true);
    // lcd_data(spi, ((0 >> 3) & 0xFF), true);
    // lcd_data(spi, (399 >> 3) & 0xFF, true);

    // lcd_cmd(spi, 0x45, true);
    // lcd_data(spi, (0 & 0xFF), true);
    // lcd_data(spi, ((0 >> 8) & 0xFF), true);
    // lcd_data(spi, (299 & 0xFF), true);
    // lcd_data(spi, ((299 >> 8) & 0xFF), true);

    // // Cursor
    // lcd_cmd(spi, 0x4E, true);
    // lcd_data(spi, (0 & 0xFF), true);

    // lcd_cmd(spi, 0x4F, true);
    // lcd_data(spi, (0 & 0xFF), true);
    // lcd_data(spi, ((0 >> 8) & 0xFF), true);

    // while (1)
    // {
    //     int level = gpio_get_level((gpio_num_t)PIN_NUM_BUSY);

    //     if (level == 0)
    //     {
    //         break;
    //     }
    // }

    // ESP_LOGI(TAG, "Init complete. Clearing screen...");

    // lcd_cmd(spi, 0x24, true);

    // for (int i = 0; i < 400; i++)
    // {
    //     for (int j = 0; j < 300; j++)
    //     {
    //         lcd_data(spi, 0xFF, true);
    //     }
    // }

    // lcd_cmd(spi, 0x26, true);

    // for (int i = 0; i < 400; i++)
    // {
    //     for (int j = 0; j < 300; j++)
    //     {
    //         lcd_data(spi, 0xFF, true);
    //     }
    // }

    // lcd_cmd(spi, 0x22, true);
    // lcd_data(spi, 0xF7, true);
    // lcd_cmd(spi, 0x20, true);

    // while (1)
    // {
    //     int level = gpio_get_level((gpio_num_t)PIN_NUM_BUSY);

    //     if (level == 0)
    //     {
    //         break;
    //     }
    // }

    // spi_device_release_bus(spi);

    // ESP_LOGI(TAG,"All done!");
}
