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
#include <esp_matter_bridge.h>
#include <app/server/Server.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <math.h>

#include "esp_tls.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include <lib/dnssd/Types.h>

#include "cJSON.h"

#include "qrcode.h"

#include "ssd1683.h"

#include <vector>
#include <format>

#include "dishwasher.h"

static const char *TAG = "app";

#define NVS_NAMESPACE "home_connect"

#define LCD_HOST SPI2_HOST
#define PARALLEL_LINES 16

using namespace esp_matter;
using namespace esp_matter_bridge;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters::OperationalState;

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 5000

spi_device_handle_t spi;

void esp_qrcode_print_display(esp_qrcode_handle_t qrcode)
{
    ESP_LOGI(TAG, "Displaying the QR Code");

    int size = esp_qrcode_get_size(qrcode);

    bool black_pixel = 0;

    ESP_LOGI(TAG, "QR Code Size: %d", size);

    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            black_pixel = esp_qrcode_get_module(qrcode, x, y);
            lcd_set_pixel(x, y, black_pixel);
        }
    }

    lcd_draw(spi);
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        // ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        // ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        // ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        // ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        //  Clean the buffer in case of a new request
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
        ESP_LOGE(TAG, "HTTP_EVENT_DISCONNECTED");
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
        // esp_http_client_set_header(evt->client, "From", "user@example.com");
        // esp_http_client_set_header(evt->client, "Accept", "text/html");
        // esp_http_client_set_redirection(evt->client);
        break;
    default:
        ESP_LOGE(TAG, "Unhandled event");
        break;
    }
    return ESP_OK;
}

void create_dishwasher_device()
{
    ESP_LOGE(TAG, "Creating Dishwasher device");

    node_t *node = node::get();
    static OperationalStateDelegate operational_state_delegate;

    dish_washer::config_t dish_washer_config;
    dish_washer_config.operational_state.delegate = &operational_state_delegate; // Set to nullptr if not using a delegate

    endpoint_t *endpoint = dish_washer::create(node, &dish_washer_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create dishwasher endpoint"));

    esp_matter::cluster_t *operational_state_cluster = esp_matter::cluster::get(endpoint, chip::app::Clusters::OperationalState::Id);

    esp_matter::cluster::operational_state::attribute::create_countdown_time(operational_state_cluster, 0);

    esp_matter::cluster::operational_state::command::create_start(operational_state_cluster);
    esp_matter::cluster::operational_state::command::create_stop(operational_state_cluster);
    esp_matter::cluster::operational_state::command::create_pause(operational_state_cluster);
    esp_matter::cluster::operational_state::command::create_resume(operational_state_cluster);
}

esp_err_t set_access_token(nvs_handle_t nvs_handle, esp_http_client_handle_t client)
{
    size_t required_size;
    esp_err_t err = nvs_get_str(nvs_handle, "access_token", NULL, &required_size);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load access_token from NVS");
        return err;
    }

    char *access_token = (char *)malloc(required_size);
    err = nvs_get_str(nvs_handle, "access_token", access_token, &required_size);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load access_token from NVS");
        return err;
    }

    ESP_LOGI(TAG, "Loaded access_token: %s", access_token);

    char auth_header[1400];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", access_token);

    esp_http_client_set_header(client, "Authorization", auth_header);

    return err;
}

esp_err_t fetch_programs(esp_http_client_handle_t client, char *haId)
{
    char url[256];
    snprintf(url, sizeof(url), "https://api.home-connect.com/api/homeappliances/%s/programs/available", haId);
    esp_http_client_set_url(client, url);

    ESP_LOGI(TAG, "Fetching programs...");
    esp_err_t err = esp_http_client_perform(client);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to fetch programs");
        return err;
    }

    ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %" PRId64,
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));

    if (esp_http_client_get_status_code(client) == 200)
    {
        char *local_response_buffer = NULL;
        esp_http_client_get_user_data(client, (void **)&local_response_buffer);

        cJSON *root = cJSON_Parse(local_response_buffer);

        if (root == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON");
            return ESP_FAIL;
        }
        else
        {
            cJSON *iterator = NULL;

            cJSON *dataJSON = cJSON_GetObjectItemCaseSensitive(root, "data");
            cJSON *programsJSON = cJSON_GetObjectItemCaseSensitive(dataJSON, "programs");

            cJSON_ArrayForEach(iterator, programsJSON)
            {
                cJSON *keyJSON = cJSON_GetObjectItemCaseSensitive(iterator, "key");
                ESP_LOGI(TAG, "key: %s", keyJSON->valuestring);

                cJSON *nameJSON = cJSON_GetObjectItemCaseSensitive(iterator, "name");
                ESP_LOGI(TAG, "type: %s", nameJSON->valuestring);
            }
        }
    }

    return err;
}

void run_loop(void *pvParameters)
{
    // Have we authenticated?
    //
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS");
        return;
    }

    // Create a client to start.
    //
    char *local_response_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);

    esp_http_client_config_t config = {
        .url = "https://api.home-connect.com/security/oauth/device_authorization",
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .buffer_size_tx = 2048,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Accept-Language", "en-gb");
    
    while (1)
    {
        if (nvs_find_key(nvs_handle, "access_token", NULL) != ESP_OK)
        {
            ESP_LOGI(TAG, "Performing HomeConnect authentication...");

            // We have no access token, so we need to authenticate.
            //
            esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

            const char *post_data = "client_id=EAEC454A48CC2D7B95D43D5776F6049114E286CFA1D16E3D494AC67CF418F3E6";
            esp_http_client_set_post_field(client, post_data, strlen(post_data));

            esp_err_t err = esp_http_client_perform(client);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to perform authentication POST");
                return;
            }

            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));

            cJSON *root = cJSON_Parse(local_response_buffer);

            if (root == NULL)
            {
                ESP_LOGE(TAG, "Failed to parse JSON");
                return;
            }

            const cJSON *deviceCodeJSON = cJSON_GetObjectItemCaseSensitive(root, "device_code");
            ESP_LOGI(TAG, "device_code: %s", deviceCodeJSON->valuestring);

            const cJSON *userCodeJSON = cJSON_GetObjectItemCaseSensitive(root, "user_code");
            ESP_LOGI(TAG, "user_code: %s", userCodeJSON->valuestring);

            const cJSON *completeVerificationUriJSON = cJSON_GetObjectItemCaseSensitive(root, "verification_uri_complete");
            ESP_LOGI(TAG, "verification_uri_complete: %s", completeVerificationUriJSON->valuestring);

            // Show the QR code to the user to scan.
            //
            esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
            cfg.display_func = esp_qrcode_print_display;

            esp_qrcode_generate(&cfg, completeVerificationUriJSON->valuestring);

            // We now need to wait for the user to authorize.
            //
            int count = 0;

            ESP_LOGI(TAG, "Waiting for user to Authorize the device...");

            while (1)
            {
                ESP_LOGI(TAG, "Checking for token....");

                esp_http_client_set_url(client, "https://api.home-connect.com/security/oauth/token");

                char post_data[200];
                snprintf(post_data, sizeof(post_data), "grant_type=device_code&device_code=%s&client_id=EAEC454A48CC2D7B95D43D5776F6049114E286CFA1D16E3D494AC67CF418F3E6", deviceCodeJSON->valuestring);

                esp_http_client_set_post_field(client, post_data, strlen(post_data));

                esp_err_t err = esp_http_client_perform(client);

                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                             esp_http_client_get_status_code(client),
                             esp_http_client_get_content_length(client));

                    if (esp_http_client_get_status_code(client) == 200)
                    {
                        cJSON *root = cJSON_Parse(local_response_buffer);

                        if (root == NULL)
                        {
                            ESP_LOGE(TAG, "Failed to parse JSON");
                            return;
                        }

                        const cJSON *accessTokenJSON = cJSON_GetObjectItemCaseSensitive(root, "access_token");
                        ESP_LOGI(TAG, "access_token: %s", accessTokenJSON->valuestring);

                        const cJSON *refreshTokenJSON = cJSON_GetObjectItemCaseSensitive(root, "refresh_token");
                        ESP_LOGI(TAG, "refresh_token: %s", refreshTokenJSON->valuestring);

                        ESP_LOGI(TAG, "Saving tokens...");

                        nvs_set_str(nvs_handle, "access_token", accessTokenJSON->valuestring);
                        nvs_set_str(nvs_handle, "refresh_token", refreshTokenJSON->valuestring);
                        nvs_commit(nvs_handle);

                        // Jump to the outer while loop.
                        break;
                    }
                }
                else
                {
                        vTaskDelay(10000 / portTICK_PERIOD_MS);
                }
            }
        }
        else if (nvs_find_key(nvs_handle, "ha_id", NULL) != ESP_OK)
        {
            // We have not fetched the appliances yet.
            //
            ESP_LOGI(TAG, "Fetching HomeConnect appliances...");

            esp_http_client_set_url(client, "https://api.home-connect.com/api/homeappliances");
            esp_http_client_set_method(client, HTTP_METHOD_GET);
            esp_http_client_set_timeout_ms(client, 5000);

            set_access_token(nvs_handle, client);

            ESP_LOGI(TAG, "Fetching appliances...");
            err = esp_http_client_perform(client);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to fetch appliances");
                return;
            }

            ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %" PRId64,
                     esp_http_client_get_status_code(client),
                     esp_http_client_get_content_length(client));

            if (esp_http_client_get_status_code(client) == 200)
            {
                cJSON *root = cJSON_Parse(local_response_buffer);

                if (root == NULL)
                {
                    ESP_LOGE(TAG, "Failed to parse JSON");
                    return;
                }
                else
                {
                    cJSON *iterator = NULL;

                    cJSON *dataJSON = cJSON_GetObjectItemCaseSensitive(root, "data");
                    cJSON *homeAppliancesJSON = cJSON_GetObjectItemCaseSensitive(dataJSON, "homeappliances");

                    cJSON_ArrayForEach(iterator, homeAppliancesJSON)
                    {
                        cJSON *haIdJSON = cJSON_GetObjectItemCaseSensitive(iterator, "haId");
                        ESP_LOGI(TAG, "haId: %s", haIdJSON->valuestring);

                        cJSON *typeJSON = cJSON_GetObjectItemCaseSensitive(iterator, "type");
                        ESP_LOGI(TAG, "type: %s", typeJSON->valuestring);

                        // Fetch the available programs
                        //
                        err = fetch_programs(client, haIdJSON->valuestring);

                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Failed to fetch programs.");
                            return;
                        }

                        nvs_set_str(nvs_handle, "ha_id", haIdJSON->valuestring);
                        nvs_commit(nvs_handle);
                    }
                }
            }
        }
        else
        {
            ESP_LOGI(TAG, "Loop...");
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }
}

/*
void perform_device_flow_authentication(intptr_t arg)
{
    ESP_LOGI(TAG, "Starting device flow authentication!");

    char *local_response_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);

    esp_http_client_config_t config = {
        .url = "https://api.home-connect.com/security/oauth/device_authorization",
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

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

        const cJSON *deviceCodeJSON = cJSON_GetObjectItemCaseSensitive(root, "device_code");
        ESP_LOGI(TAG, "device_code: %s", deviceCodeJSON->valuestring);

        const cJSON *userCodeJSON = cJSON_GetObjectItemCaseSensitive(root, "user_code");
        ESP_LOGI(TAG, "user_code: %s", userCodeJSON->valuestring);

        const cJSON *completeVerificationUriJSON = cJSON_GetObjectItemCaseSensitive(root, "verification_uri_complete");
        ESP_LOGI(TAG, "verification_uri_complete: %s", completeVerificationUriJSON->valuestring);

        // Show the QR code to the user to scan.
        //
        esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
        cfg.display_func = esp_qrcode_print_display;

        esp_qrcode_generate(&cfg, completeVerificationUriJSON->valuestring);

        // We now need to wait for the user to authorize.
        //
        int count = 0;

        while (1)
        {
            ESP_LOGI(TAG, "Checking for token...");

            esp_http_client_set_url(client, "https://api.home-connect.com/security/oauth/token");

            char post_data[200];
            snprintf(post_data, sizeof(post_data), "grant_type=device_code&device_code=%s&client_id=EAEC454A48CC2D7B95D43D5776F6049114E286CFA1D16E3D494AC67CF418F3E6", deviceCodeJSON->valuestring);

            esp_http_client_set_post_field(client, post_data, strlen(post_data));

            esp_err_t err = esp_http_client_perform(client);

            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRId64,
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));

                if (esp_http_client_get_status_code(client) == 200)
                {
                    cJSON *root = cJSON_Parse(local_response_buffer);

                    if (root == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to parse JSON");
                        return;
                    }

                    const cJSON *accessTokenJSON = cJSON_GetObjectItemCaseSensitive(root, "access_token");
                    ESP_LOGI(TAG, "access_token: %s", accessTokenJSON->valuestring);

                    const cJSON *refreshTokenJSON = cJSON_GetObjectItemCaseSensitive(root, "refresh_token");
                    ESP_LOGI(TAG, "refresh_token: %s", refreshTokenJSON->valuestring);

                    nvs_handle_t nvs_handle;
                    esp_err_t err;

                    ESP_LOGI(TAG, "Saving tokens...");

                    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
                    if (err == ESP_OK)
                    {
                        nvs_set_str(nvs_handle, "access_token", accessTokenJSON->valuestring);
                        nvs_set_str(nvs_handle, "refresh_token", refreshTokenJSON->valuestring);
                        nvs_commit(nvs_handle);
                    }
                    else
                    {
                        return;
                    }
                    nvs_close(nvs_handle);

                    esp_http_client_set_url(client, "https://api.home-connect.com/api/homeappliances");
                    esp_http_client_set_method(client, HTTP_METHOD_GET);

                    set_access_token(client);

                    ESP_LOGI(TAG, "Fetching appliances...");
                    err = esp_http_client_perform(client);

                    if (err == ESP_OK)
                    {
                        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %" PRId64,
                                 esp_http_client_get_status_code(client),
                                 esp_http_client_get_content_length(client));

                        if (esp_http_client_get_status_code(client) == 200)
                        {
                            cJSON *root = cJSON_Parse(local_response_buffer);

                            if (root == NULL)
                            {
                                ESP_LOGE(TAG, "Failed to parse JSON");
                                return;
                            }
                            else
                            {
                                cJSON *iterator = NULL;

                                cJSON *dataJSON = cJSON_GetObjectItemCaseSensitive(root, "data");
                                cJSON *homeAppliancesJSON = cJSON_GetObjectItemCaseSensitive(dataJSON, "homeappliances");

                                cJSON_ArrayForEach(iterator, homeAppliancesJSON)
                                {
                                    cJSON *haIdJSON = cJSON_GetObjectItemCaseSensitive(iterator, "haId");
                                    ESP_LOGI(TAG, "haId: %s", haIdJSON->valuestring);

                                    cJSON *typeJSON = cJSON_GetObjectItemCaseSensitive(iterator, "type");
                                    ESP_LOGI(TAG, "type: %s", typeJSON->valuestring);

                                    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
                                    if (err == ESP_OK)
                                    {
                                        nvs_set_str(nvs_handle, "ha_id", haIdJSON->valuestring);
                                        nvs_commit(nvs_handle);
                                    }
                                    nvs_close(nvs_handle);

                                    // TODO Only handle dishwashers!
                                    //

                                    // Fetch the available programs
                                    //
                                    fetch_programs(client, haIdJSON->valuestring);

                                    return;
                                }
                            }
                        }
                        else
                        {
                            ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
                            return;
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
                        return;
                    }
                }

                // Token wasn't fetched. Give it another go.
                //
                count++;

                if (count > 6)
                {
                    vTaskDelete(NULL);
                    return;
                }

                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
            else
            {
                ESP_LOGE(TAG, "Token request failed. Closing connection and waiting...");
                esp_http_client_close(client);

                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
        }
    }

    return;
}
*/

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kESPSystemEvent:
        ESP_LOGI(TAG, "kESPSystemEvent");

        if (event->Platform.ESPSystemEvent.Base == IP_EVENT &&
            event->Platform.ESPSystemEvent.Id == IP_EVENT_STA_GOT_IP)
        {
            TaskHandle_t xHandle = NULL;

            /* Create the task, storing the handle. */
            xTaskCreate(
                run_loop,         /* Function that implements the task. */
                "NAME",           /* Text name for the task. */
                5 * 1024,         /* Stack size in words, not bytes. */
                NULL,             /* Parameter passed into the task. */
                tskIDLE_PRIORITY, /* Priority at which the task is created. */
                &xHandle);
        }
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        lcd_clear(spi);
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        {
            chip::RendezvousInformationFlags rendezvoudFlags = chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE);

            chip::PayloadContents payload;
            GetPayloadContents(payload, rendezvoudFlags);

            char payloadBuffer[chip::QRCodeBasicSetupPayloadGenerator::kMaxQRCodeBase38RepresentationLength + 1];
            chip::MutableCharSpan qrCode(payloadBuffer);

            esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
            cfg.display_func = esp_qrcode_print_display;

            if (GetQRCode(qrCode, payload) == CHIP_NO_ERROR)
            {
                esp_qrcode_generate(&cfg, qrCode.data());
            }
            else
            {
                ESP_LOGE(TAG, "Failed to generate the commissioning QR code");
            }
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

// set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level((gpio_num_t)PIN_NUM_DC, dc);
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

    ESP_LOGI(TAG, "Setting up SPI...");

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
        .mode = 0,                               // SPI mode 0
        .clock_speed_hz = 20 * 1000 * 1000,      // Clock out at 20 MHz
        .spics_io_num = PIN_NUM_CS,              // CS pin
        .queue_size = 7,                         // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };

    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_DC) | (1ULL << PIN_NUM_RST) | (1ULL << 41) | (1ULL << 7));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)41, true);
    ESP_LOGI(TAG, "Applied power to LED");

    gpio_set_level((gpio_num_t)7, true);
    ESP_LOGI(TAG, "Applied power to display");

    lcd_init(spi);

    lcd_clear(spi);

    lcd_draw_string(spi, 10, 150, 8, "Merry Christmas, ya filthy animals");

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // If we have an dishwasher, be sure to create the endpoint, since it's dynamic
    //
    // nvs_handle_t nvs_handle;
    // err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    // if (err == ESP_OK && nvs_find_key(nvs_handle, "ha_id", NULL) == ESP_OK)
    // {
    //     create_dishwasher_device();
    // }
    // nvs_close(nvs_handle);

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    ESP_LOGI(TAG, "All done!");
}
