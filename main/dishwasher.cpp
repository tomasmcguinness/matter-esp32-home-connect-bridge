#include <esp_log.h>
#include <app-common/zap-generated/attribute-type.h>
#include <app-common/zap-generated/cluster-enums.h>
#include <app/util/generic-callbacks.h>
#include <protocols/interaction_model/StatusCode.h>
#include <esp_debug_helpers.h>
#include "program_manager.h"
#include "dishwasher.h"

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::ModeBase;
using namespace chip::app::Clusters::OperationalState;
using namespace chip::app::Clusters::DishwasherMode;
using namespace chip::app::Clusters::DeviceEnergyManagement;
using namespace chip::app::Clusters::DeviceEnergyManagement::Attributes;
using namespace chip::Protocols::InteractionModel;

static const char *TAG = "dishwasher";

extern program_manager_t g_program_manager;

//******************************
//* OPERATIONAL STATE DELEGATE *
//******************************

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