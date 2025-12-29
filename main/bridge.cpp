#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_bridge.h>

using esp_matter::node_t;

extern uint16_t aggregator_endpoint_id;

static const char *TAG = "bridge";

esp_err_t app_bridge_create_bridged_device(node_t *node, uint16_t parent_endpoint_id, uint32_t matter_device_type_id, void *priv_data)
{
    ESP_LOGI(TAG, "Creating Bridged Device of Type: %lu", matter_device_type_id);

    esp_matter_bridge::device_t *new_dev = esp_matter_bridge::create_device(node, parent_endpoint_id, matter_device_type_id, NULL);

    esp_matter::endpoint::enable(new_dev->endpoint);

    return ESP_OK;
}

esp_err_t app_bridge_initialize(node_t *node, esp_matter_bridge::bridge_device_type_callback_t device_type_cb)
{
    esp_err_t err = esp_matter_bridge::initialize(node, device_type_cb);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize the esp_matter_bridge");
    }

    return err;
}