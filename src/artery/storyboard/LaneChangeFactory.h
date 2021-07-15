#ifndef ARTERY_LANCHANGEFACTORY_H_
#define ARTERY_LANCHANGEFACTORY_H_

#include "artery/storyboard/Effect.h"
#include "artery/storyboard/EffectFactory.h"
#include "artery/traci/VehicleController.h"
#include <memory>

namespace artery
{

/**
 * SpeedEffectFactory creates SpeedEffects
 */
class LaneChangeFactory : public EffectFactory
{
public:
    LaneChangeFactory(int lanid, double duration) :
        mLaneIndex(lanid), mDuration(duration)
    {
    }

    /**
     * Creates a new Effect for a TraCIMobility
     * \param Vehicle which should be affected from the Effect
     * \param Story from which the Effect should be created
     * \return Effect to add on the EffectStack from the TraCIMobility
     */
    std::shared_ptr<Effect> create(Vehicle&, Story&, ConditionResult&) override;

private:
    int mLaneIndex;
    double mDuration;
};

} // namespace artery

#endif /* ARTERY_LANCHANGEFACTORY_H_ */
