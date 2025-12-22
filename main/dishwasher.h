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