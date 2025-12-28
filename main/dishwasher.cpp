#include <esp_log.h>
#include <dishwasher.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-enums.h>
#include <app/util/generic-callbacks.h>
#include <protocols/interaction_model/StatusCode.h>
#include <esp_debug_helpers.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::OperationalState;
using namespace chip::app::Clusters::DishwasherMode;
using namespace chip::app::Clusters::DeviceEnergyManagement;
using namespace chip::app::Clusters::DeviceEnergyManagement::Attributes;
using namespace chip::Protocols::InteractionModel;

static const char *TAG = "dishwasher";

//******************************
//* OPERATIONAL STATE DELEGATE *
//******************************

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
    return CHIP_NO_ERROR;
}

void DishwasherModeDelegate::HandleChangeToMode(uint8_t NewMode, ModeBase::Commands::ChangeToModeResponse::Type &response)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::HandleChangeToMode()");
    response.status = to_underlying(ModeBase::StatusCode::kSuccess);
}

CHIP_ERROR DishwasherModeDelegate::GetModeLabelByIndex(uint8_t modeIndex, chip::MutableCharSpan &label)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeLabelByIndex()");

    if (modeIndex >= MATTER_ARRAY_SIZE(kModeOptions))
    {
        ESP_LOGI(TAG, "CHIP_ERROR_PROVIDER_LIST_EXHAUSTED");
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }

    return chip::CopyCharSpanToMutableCharSpan(kModeOptions[modeIndex].label, label);
}

CHIP_ERROR DishwasherModeDelegate::GetModeValueByIndex(uint8_t modeIndex, uint8_t &value)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeValueByIndex(%d)", modeIndex);

    if (modeIndex >= MATTER_ARRAY_SIZE(kModeOptions))
    {
        ESP_LOGI(TAG, "CHIP_ERROR_PROVIDER_LIST_EXHAUSTED");
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }
    value = kModeOptions[modeIndex].mode;

    return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
}

CHIP_ERROR DishwasherModeDelegate::GetModeTagsByIndex(uint8_t modeIndex, List<ModeTagStructType> &tags)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::GetModeTagsByIndex()");

    if (modeIndex >= MATTER_ARRAY_SIZE(kModeOptions))
    {
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    }

    if (tags.size() < kModeOptions[modeIndex].modeTags.size())
    {
        return CHIP_ERROR_INVALID_ARGUMENT;
    }

    std::copy(kModeOptions[modeIndex].modeTags.begin(), kModeOptions[modeIndex].modeTags.end(), tags.begin());
    tags.reduce_size(kModeOptions[modeIndex].modeTags.size());

    return CHIP_NO_ERROR;
}

Status DishwasherModeDelegate::SetDishwasherMode(uint8_t modeValue)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::SetDishwasherMode");

    // We can only update the DishwasherMode when it's not running.
    //

    VerifyOrReturnError(DishwasherMode::GetInstance() != nullptr, Status::InvalidInState);

    if (!DishwasherMode::GetInstance()->IsSupportedMode(modeValue))
    {
        ChipLogError(AppServer, "SetDishwasherMode bad mode");
        return Status::ConstraintError;
    }

    Status status = DishwasherMode::GetInstance()->UpdateCurrentMode(modeValue);
    if (status != Status::Success)
    {
        ChipLogError(AppServer, "SetDishwasherMode updateMode failed 0x%02x", to_underlying(status));
        return status;
    }

    return chip::Protocols::InteractionModel::Status::Success;
}

void DishwasherModeDelegate::PostAttributeChangeCallback(AttributeId attributeId, uint8_t type, uint16_t size, uint8_t *value)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::PostAttributeChangeCallback");
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

void emberAfDishwasherModeClusterInitCallback(chip::EndpointId endpointId)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::emberAfDishwasherModeClusterInitCallback");

    VerifyOrDie(endpointId == 1); // this cluster is only enabled for endpoint 1.
    VerifyOrDie(gDishwasherModeDelegate == nullptr && gDishwasherModeInstance == nullptr);
    gDishwasherModeDelegate = new DishwasherMode::DishwasherModeDelegate;
    gDishwasherModeInstance = new ModeBase::Instance(gDishwasherModeDelegate, 0x1, DishwasherMode::Id, 0);
    gDishwasherModeInstance->Init();
}

void emberAfDishwasherModeClusterShutdownCallback(chip::EndpointId endpointId)
{
    ESP_LOGI(TAG, "DishwasherModeDelegate::emberAfDishwasherModeClusterShutdownCallback");

    VerifyOrDie(endpointId == 1); // this cluster is only enabled for endpoint 1.
    if (gDishwasherModeInstance)
    {
        gDishwasherModeInstance->Shutdown();
    }
    DishwasherMode::Shutdown();
}