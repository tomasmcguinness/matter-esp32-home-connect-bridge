#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_bridge.h>

using esp_matter::node_t;

esp_err_t app_bridge_create_bridged_device(node_t *node, uint16_t parent_endpoint_id, uint32_t matter_device_type_id, void *priv_data);
esp_err_t app_bridge_initialize(node_t *node, esp_matter_bridge::bridge_device_type_callback_t device_type_cb);