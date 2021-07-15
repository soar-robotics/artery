#include <artery/storyboard/LaneChangeModeFactory.h>
#include <artery/storyboard/LaneChangeMode.h>

namespace artery
{

std::shared_ptr<Effect> LaneChangeModeFactory::create(Vehicle& car, Story& story, ConditionResult& result)
{
    std::shared_ptr<Effect> ptr(new LaneChangeMode(car, mMode, story));
    return ptr;
}

} // namespace artery
