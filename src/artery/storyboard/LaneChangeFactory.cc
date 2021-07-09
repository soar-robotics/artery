#include <artery/storyboard/LaneChangeFactory.h>
#include <artery/storyboard/LaneChange.h>

namespace artery
{

std::shared_ptr<Effect> LaneChangeFactory::create(Vehicle& car, Story& story, ConditionResult& result)
{
    std::shared_ptr<Effect> ptr(new LaneChange(car, mLaneIndex, mDuration, story));
    return ptr;
}

} // namespace artery
