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

using namespace esp_matter;

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 20000

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

#include <sys/param.h>

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

    ESP_LOGI(TAG, "Configuring Display Device...");

    spi_device_interface_config_t devcfg = {
        .mode = 0,                                  //SPI mode 0
        .clock_speed_hz = 20 * 1000 * 1000,         //Clock out at 20 MHz
        .spics_io_num = PIN_NUM_CS,                 //CS pin
        .queue_size = 7,                            //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback,    //Specify pre-transfer callback to handle D/C line
    };

    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST) | (1ULL << 41) | (1ULL << 7) );
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)41, true);
    ESP_LOGI(TAG, "Applied power to LED");

    gpio_set_level((gpio_num_t)7, true);
    ESP_LOGI(TAG, "Applied power to display");

    lcd_init(spi, IMAGE_W, IMAGE_H);
    
    lcd_clear(spi, IMAGE_W, IMAGE_H);

    ESP_LOGI(TAG,"All done!");
}
