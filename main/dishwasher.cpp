#include <esp_log.h>
#include <dishwasher.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-enums.h>
#include <app/util/generic-callbacks.h>
#include <esp_debug_helpers.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OperationalState;
using namespace chip::app::Clusters::DeviceEnergyManagement;
using namespace chip::app::Clusters::DeviceEnergyManagement::Attributes;
using namespace chip::Protocols::InteractionModel;

static const char *TAG = "dishwasher";

DataModel::Nullable<uint32_t> OperationalStateDelegate::GetCountdownTime()
{
    ESP_LOGI(TAG, "GetCountdownTime");
    return DataModel::NullNullable;
}

CHIP_ERROR OperationalStateDelegate::GetOperationalStateAtIndex(size_t index, GenericOperationalState &operationalState)
{
    ESP_LOGI(TAG, "GetOperationalStateAtIndex");
    return CHIP_ERROR_NOT_FOUND;
}

CHIP_ERROR OperationalStateDelegate::GetOperationalPhaseAtIndex(size_t index, MutableCharSpan &operationalPhase)
{
    ESP_LOGI(TAG, "GetOperationalPhaseAtIndex");
    return CHIP_ERROR_NOT_FOUND;
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

static OperationalState::Instance *gOperationalStateInstance = nullptr;
static OperationalStateDelegate *gOperationalStateDelegate = nullptr;

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

OperationalState::Instance *OperationalState::GetInstance()
{
    return gOperationalStateInstance;
}

OperationalState::OperationalStateDelegate *OperationalState::GetDelegate()
{
    return gOperationalStateDelegate;
}

void emberAfOperationalStateClusterInitCallback(chip::EndpointId endpointId)
{
    ESP_LOGI(TAG, "emberAfOperationalStateClusterInitCallback()");

    VerifyOrDie(endpointId == 1); // this cluster is only enabled for endpoint 1.
    VerifyOrDie(gOperationalStateInstance == nullptr && gOperationalStateDelegate == nullptr);

    gOperationalStateDelegate = new OperationalStateDelegate;
    EndpointId operationalStateEndpoint = 0x01;
    gOperationalStateInstance = new OperationalState::Instance(gOperationalStateDelegate, operationalStateEndpoint);

    gOperationalStateInstance->SetOperationalState(to_underlying(OperationalState::OperationalStateEnum::kStopped));
    gOperationalStateInstance->SetCurrentPhase(0);

    gOperationalStateInstance->Init();

    uint8_t value = to_underlying(OperationalStateEnum::kStopped);
    gOperationalStateDelegate->PostAttributeChangeCallback(chip::app::Clusters::OperationalState::Attributes::OperationalState::Id, ZCL_INT8U_ATTRIBUTE_TYPE, sizeof(uint8_t), &value);
    gOperationalStateDelegate->PostAttributeChangeCallback(chip::app::Clusters::OperationalState::Attributes::CurrentPhase::Id, ZCL_INT8U_ATTRIBUTE_TYPE, sizeof(uint8_t), 0);
}