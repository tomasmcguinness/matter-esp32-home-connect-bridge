#include <esp_err.h>
#include <esp_matter.h>
#include <app-common/zap-generated/cluster-objects.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/cluster-enums.h>
#include <app/clusters/mode-base-server/mode-base-server.h>
#include <app/clusters/mode-base-server/mode-base-cluster-objects.h>
#include <app/clusters/operational-state-server/operational-state-server.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::ModeBase;
using namespace chip::app::Clusters::OperationalState;
using namespace chip::app::Clusters::DishwasherMode;
using namespace chip::Protocols::InteractionModel;

namespace chip
{
    namespace app
    {
        namespace Clusters
        {
            namespace OperationalState
            {
                class OperationalStateDelegate : public Delegate
                {
                public:
                    uint32_t mRunningTime = 0;
                    uint32_t mPausedTime = 0;

                    chip::app::DataModel::Nullable<uint32_t> GetCountdownTime();

                    CHIP_ERROR GetOperationalStateAtIndex(size_t index, GenericOperationalState &operationalState) override;

                    CHIP_ERROR GetOperationalPhaseAtIndex(size_t index, MutableCharSpan &operationalPhase) override;

                    void HandlePauseStateCallback(GenericOperationalError &err) override;

                    void HandleResumeStateCallback(GenericOperationalError &err) override;

                    void HandleStartStateCallback(GenericOperationalError &err) override;

                    void HandleStopStateCallback(GenericOperationalError &err) override;

                    void PostAttributeChangeCallback(AttributeId attributeId, uint8_t type, uint16_t size, uint8_t *value);

                private:
                    const GenericOperationalState opStateList[4] = {
                        GenericOperationalState(to_underlying(OperationalStateEnum::kStopped)),
                        GenericOperationalState(to_underlying(OperationalStateEnum::kRunning)),
                        GenericOperationalState(to_underlying(OperationalStateEnum::kPaused)),
                        GenericOperationalState(to_underlying(OperationalStateEnum::kError)),
                    };
                };

                OperationalState::Instance *GetInstance();
                OperationalState::OperationalStateDelegate *GetDelegate();

                void Shutdown();

            } // namespace OperationalState
        } // namespace Clusters
    } // namespace app
} // namespace chip

namespace chip
{
    namespace app
    {
        namespace Clusters
        {
            namespace DishwasherMode
            {
                const uint8_t ModeNormal = 0;

                class DishwasherModeDelegate : public ModeBase::Delegate
                {
                private:
                    using ModeTagStructType = detail::Structs::ModeTagStruct::Type;
                    ModeTagStructType modeTagsNormal[1] = {{.value = to_underlying(ModeTag::kNormal)}};

                    const detail::Structs::ModeOptionStruct::Type kModeOptions[1] = {
                        detail::Structs::ModeOptionStruct::Type{.label = CharSpan::fromCharString("Eco 50°"),
                                                                .mode = ModeNormal,
                                                                .modeTags = DataModel::List<const ModeTagStructType>(modeTagsNormal)},
                        };

                    CHIP_ERROR Init() override;
                    void HandleChangeToMode(uint8_t mode, ModeBase::Commands::ChangeToModeResponse::Type &response) override;

                    CHIP_ERROR GetModeValueByIndex(uint8_t modeIndex, uint8_t &value) override;
                    CHIP_ERROR GetModeTagsByIndex(uint8_t modeIndex, DataModel::List<ModeTagStructType> &tags) override;

                public:
                    ~DishwasherModeDelegate() override = default;

                    CHIP_ERROR GetModeLabelByIndex(uint8_t modeIndex, MutableCharSpan &label) override;

                    void PostAttributeChangeCallback(AttributeId attributeId, uint8_t type, uint16_t size, uint8_t *value);

                    Protocols::InteractionModel::Status SetDishwasherMode(uint8_t mode);
                };

                ModeBase::Instance *GetInstance();
                ModeBase::Delegate *GetDelegate();

                void Shutdown();

            } // namespace DishwasherMode

        } // namespace Clusters
    } // namespace app
} // namespace chip