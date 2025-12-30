#include <esp_log.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-enums.h>
#include <app/util/generic-callbacks.h>
#include <protocols/interaction_model/StatusCode.h>
#include <esp_debug_helpers.h>
#include "program_manager.h"
#include "dishwasher.h"
#include "esp_crt_bundle.h"
#include <nvs_flash.h>
#include "esp_tls.h"

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OperationalState;
using namespace chip::app::Clusters::DeviceEnergyManagement;
using namespace chip::app::Clusters::DeviceEnergyManagement::Attributes;
using namespace chip::Protocols::InteractionModel;

static const char *TAG = "dishwasher";

extern program_manager_t g_program_manager;

#define MAX_HTTP_OUTPUT_BUFFER 5000
#define NVS_NAMESPACE "home_connect"

extern esp_err_t _http_event_handler(esp_http_client_event_t *evt);

//******************************
//* OPERATIONAL STATE DELEGATE *
//******************************

static OperationalState::Instance *gOperationalStateInstance = nullptr;
static OperationalStateDelegate *gOperationalStateDelegate = nullptr;

// TODO Implement Init() to capture instance and delegate. See DishwasherMode for example.
//
// gOperationalStateInstance = GetInstance();
// gOperationalStateDelegate = this;

DataModel::Nullable<uint32_t> OperationalStateDelegate::GetCountdownTime()
{
    ESP_LOGI(TAG, "OperationalStateDelegate::GetCountdownTime");
    return DataModel::NullNullable;
}

CHIP_ERROR OperationalStateDelegate::GetOperationalStateAtIndex(size_t index, GenericOperationalState &operationalState)
{
    ESP_LOGI(TAG, "OperationalStateDelegate::GetOperationalStateAtIndex");

    if (index > mOperationalStateList.size() - 1)
    {
        return CHIP_ERROR_NOT_FOUND;
    }

    operationalState = mOperationalStateList[index];

    return CHIP_NO_ERROR;
}

CHIP_ERROR OperationalStateDelegate::GetOperationalPhaseAtIndex(size_t index, MutableCharSpan &operationalPhase)
{
    ESP_LOGI(TAG, "OperationalStateDelegate::GetOperationalPhaseAtIndex");

    if (index >= mOperationalPhaseList.size())
    {
        return CHIP_ERROR_NOT_FOUND;
    }

    return CopyCharSpanToMutableCharSpan(mOperationalPhaseList[index], operationalPhase);
}

void OperationalStateDelegate::HandlePauseStateCallback(GenericOperationalError &err)
{
    ESP_LOGI(TAG, "HandlePauseStateCallback");
    err.Set(to_underlying(ErrorStateEnum::kNoError));
}

void OperationalStateDelegate::HandleResumeStateCallback(GenericOperationalError &err)
{
    ESP_LOGI(TAG, "HandleResumeStateCallback");
    err.Set(to_underlying(ErrorStateEnum::kNoError));
}

void OperationalStateDelegate::HandleStartStateCallback(GenericOperationalError &err)
{
    ESP_LOGI(TAG, "HandleStartStateCallback");

    start_selected_program();

    err.Set(to_underlying(ErrorStateEnum::kNoError));
}

void OperationalStateDelegate::HandleStopStateCallback(GenericOperationalError &err)
{
    ESP_LOGI(TAG, "HandleStopStateCallback");
    err.Set(to_underlying(ErrorStateEnum::kNoError));
}

void OperationalStateDelegate::PostAttributeChangeCallback(AttributeId attributeId, uint8_t type, uint16_t size, uint8_t *value)
{
    ESP_LOGI(TAG, "OperationalStateDelegate::PostAttributeChangeCallback");
}

void OperationalState::Shutdown()
{
    if (gOperationalStateInstance != nullptr)
    {
        delete gOperationalStateInstance;
        gOperationalStateInstance = nullptr;
    }
    if (gOperationalStateDelegate != nullptr)
    {
        delete gOperationalStateDelegate;
        gOperationalStateDelegate = nullptr;
    }
}

//****************************
//* DISHWASHER MODE DELEGATE *
//****************************

using chip::Protocols::InteractionModel::Status;
template <typename T>
using List = chip::app::DataModel::List<T>;
using ModeTagStructType = chip::app::Clusters::detail::Structs::ModeTagStruct::Type;

static DishwasherModeDelegate *gDishwasherModeDelegate = nullptr;
static ModeBase::Instance *gDishwasherModeInstance = nullptr;

CHIP_ERROR DishwasherModeDelegate::Init()
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::Init()");

    gDishwasherModeInstance = GetInstance();
    gDishwasherModeDelegate = this;

    return CHIP_NO_ERROR;
}

void DishwasherModeDelegate::HandleChangeToMode(uint8_t NewMode, ModeBase::Commands::ChangeToModeResponse::Type &response)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::HandleChangeToMode()");
    response.status = to_underlying(ModeBase::StatusCode::kSuccess);
}

CHIP_ERROR DishwasherModeDelegate::GetModeLabelByIndex(uint8_t modeIndex, chip::MutableCharSpan &label)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeLabelByIndex(%d)", modeIndex);

    if (modeIndex >= g_program_manager.program_count)
    {
        ESP_LOGI(TAG, "CHIP_ERROR_PROVIDER_LIST_EXHAUSTED");
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }

    program_t *program = find_program(&g_program_manager, modeIndex);

    return chip::CopyCharSpanToMutableCharSpan(CharSpan::fromCharString(program->name), label);
}

CHIP_ERROR DishwasherModeDelegate::GetModeValueByIndex(uint8_t modeIndex, uint8_t &value)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeValueByIndex(%d)", modeIndex);

    if (modeIndex >= g_program_manager.program_count)
    {
        ESP_LOGI(TAG, "CHIP_ERROR_PROVIDER_LIST_EXHAUSTED");
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }

    program_t *program = find_program(&g_program_manager, modeIndex);

    value = program->program_id;

    return CHIP_NO_ERROR;
}

CHIP_ERROR DishwasherModeDelegate::GetModeTagsByIndex(uint8_t modeIndex, List<ModeTagStructType> &tags)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeTagsByIndex(%d)", modeIndex);

    if (modeIndex >= g_program_manager.program_count)
    {
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }

    // Always return kNormal.
    //
    detail::Structs::ModeTagStruct::Type modeTagsNormal[1] = {{.value = to_underlying(ModeTag::kNormal)}};

    DataModel::List<const detail::Structs::ModeTagStruct::Type> modeTags = DataModel::List<const detail::Structs::ModeTagStruct::Type>(modeTagsNormal);

    std::copy(modeTags.begin(), modeTags.end(), tags.begin());
    tags.reduce_size(modeTags.size());

    return CHIP_NO_ERROR;
}

ModeBase::Instance *DishwasherMode::GetInstance()
{
    return gDishwasherModeInstance;
}

ModeBase::Delegate *DishwasherMode::GetDelegate()
{
    return gDishwasherModeDelegate;
}

void DishwasherMode::Shutdown()
{
    if (gDishwasherModeInstance != nullptr)
    {
        delete gDishwasherModeInstance;
        gDishwasherModeInstance = nullptr;
    }
    if (gDishwasherModeDelegate != nullptr)
    {
        delete gDishwasherModeDelegate;
        gDishwasherModeDelegate = nullptr;
    }
}

esp_err_t start_selected_program()
{
    ESP_LOGI(TAG, "OperationalStateDelegate::ExecuteStart()");

    ModeBase::Instance *mode = DishwasherMode::GetInstance();

    if (!mode)
    {
        ESP_LOGE(TAG, "Mode Instance is null");
        return ESP_FAIL;
    }

    uint8_t current_mode = mode->GetCurrentMode();

    ESP_LOGI(TAG, "Starting program id: %u", current_mode);

    program_t *selected_program = find_program(&g_program_manager, current_mode);

    ESP_LOGI(TAG, "Starting program: %s", selected_program->name);

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    // Load the access token from NVS.
    //
    size_t required_size;
    ret = nvs_get_str(nvs_handle, "access_token", NULL, &required_size);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load access_token from NVS");
        return ret;
    }

    char *access_token = (char *)malloc(required_size);
    ret = nvs_get_str(nvs_handle, "access_token", access_token, &required_size);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load access_token from NVS");
        return ret;
    }

    ESP_LOGI(TAG, "Loaded access_token: %s", access_token);

    char auth_header[1400];
    snprintf(auth_header, sizeof(auth_header), "Bearer %s", access_token);

    // Load the Home Connect ID from NVS.
    //
    ret = nvs_get_str(nvs_handle, "ha_id", NULL, &required_size);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load access_token from NVS");
        return ret;
    }

    char *ha_id = (char *)malloc(required_size);
    ret = nvs_get_str(nvs_handle, "ha_id", ha_id, &required_size);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load ha_id from NVS");
        return ret;
    }

    char url[256];
    snprintf(url, sizeof(url), "https://api.home-connect.com/api/homeappliances/%s/programs/active", ha_id);

    char *local_response_buffer = (char *)malloc(MAX_HTTP_OUTPUT_BUFFER + 1);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PUT,
        .event_handler = _http_event_handler,
        .buffer_size_tx = 2048,
        .user_data = local_response_buffer,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Accept-Language", "en-gb");
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // BUILD THE BODY
    //
    cJSON *root = cJSON_CreateObject();
    cJSON *data;
    cJSON *options;
    cJSON *option;

    cJSON_AddItemToObject(root, "data", data = cJSON_CreateObject());
    cJSON_AddStringToObject(data, "key", selected_program->key);

    cJSON_AddItemToObject(data, "options", options = cJSON_CreateArray());

    cJSON_AddItemToArray(options, option = cJSON_CreateObject());
    cJSON_AddStringToObject(option, "key", "BSH.Common.Option.StartInRelative");
    cJSON_AddNumberToObject(option, "value", 60);
    cJSON_AddStringToObject(option, "unit", "seconds");

    char *payload = cJSON_PrintUnformatted(root);

    esp_http_client_set_post_field(client, payload, strlen(payload));

    cJSON_Delete(root);

    ESP_LOGI(TAG, "Starting program with payload: %s", payload);

    ESP_LOGI(TAG, "Sending PUT request to: %s", url);

    ret = esp_http_client_perform(client);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to perform start program PUT");
        return ret;
    }

    ESP_LOGI(TAG, "HTTP PUT Status = %d, content_length = %" PRId64,
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));

    char *buff = NULL;
    esp_http_client_get_user_data(client, (void **)&buff);

    if (esp_http_client_get_status_code(client) == 200)
    {
        ESP_LOGI(TAG, "Response: %s", buff);
        cJSON *root = cJSON_Parse(buff);

        if (root == NULL)
        {
            ESP_LOGE(TAG, "Failed to parse JSON");
            return ESP_FAIL;
        }
        else
        {
        }
    }

    return ESP_OK;
}