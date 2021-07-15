#include "artery/storyboard/LaneChangeMode.h"
#include "artery/storyboard/Vehicle.h"
#include "artery/traci/VehicleController.h"
#include <boost/units/io.hpp>
#include <omnetpp/clog.h>

namespace artery
{

void LaneChangeMode::applyEffect()
{
    auto& controller = getCar().getController();
    //m_current = controller.getMaxSpeed() / boost::units::si::meter_per_second;
    controller.setLaneChangeMode(mMode);

    EV_STATICCONTEXT;
    EV_DEBUG << "LaneChangeMode applied to " << controller.getVehicleId() << ": mMode = " << mMode << "\n";
}

void LaneChangeMode::removeEffect()
{
    /*
    auto& controller = getCar().getController();
    controller.setLaneChangeMode(m_current * boost::units::si::meter_per_second);

    EV_STATICCONTEXT;
    EV_DEBUG << "LaneChangeMode removed from " << controller.getVehicleId() << ": mMode = " << mMode<< "\n";
    */
}

void LaneChangeMode::reapplyEffect()
{
    applyEffect();
}

} // namespace artery
