#include "artery/storyboard/LaneChange.h"
#include "artery/storyboard/Vehicle.h"
#include "artery/traci/VehicleController.h"
#include <boost/units/io.hpp>
#include <omnetpp/clog.h>

namespace artery
{

void LaneChange::applyEffect()
{
    auto& controller = getCar().getController();
    //m_current = controller.getMaxSpeed() / boost::units::si::meter_per_second;
    controller.changeLane(mLaneIndex, mDuration);

    EV_STATICCONTEXT;
    EV_DEBUG << "LaneChange applied to " << controller.getVehicleId() << ": dueation = " << mLaneIndex << "\n";
}

void LaneChange::removeEffect()
{
    /*auto& controller = getCar().getController();
    controller.setMaxSpeed(m_current * boost::units::si::meter_per_second);

    EV_STATICCONTEXT;
    EV_DEBUG << "LaneChange removed from " << controller.getVehicleId() << ": MaxSpeed = " << controller.getMaxSpeed() << "\n";
    */
}

void LaneChange::reapplyEffect()
{
    applyEffect();
}

} // namespace artery
