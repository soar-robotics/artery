#ifndef ARTERY_LANCHANGEMODEFACTORY_H_
#define ARTERY_LANCHANGEMODEFACTORY_H_

#include "artery/storyboard/Effect.h"
#include "artery/storyboard/EffectFactory.h"
#include "artery/traci/VehicleController.h"
#include <memory>

namespace artery
{

/**
 * SpeedEffectFactory creates SpeedEffects
 */
class LaneChangeModeFactory : public EffectFactory
{
public:
    LaneChangeModeFactory(int mode) :
        mMode(mode)
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
    int mMode;
};

} // namespace artery

#endif /* ARTERY_LANCHANGEMODEFACTORY_H_ */
