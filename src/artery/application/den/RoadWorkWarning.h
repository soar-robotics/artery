/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_EMERGENCYVEHICLEWARNING_H_QRTLCYIY
#define ARTERY_EMERGENCYVEHICLEWARNING_H_QRTLCYIY

#include "artery/application/den/SuspendableUseCase.h"
#include "artery/application/Sampling.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/velocity.hpp>
#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"

#include "artery/application/V2XUtils.h"

namespace artery
{
namespace den
{

class RoadWorkWarning : public SuspendableUseCase//, public ItsG5Service
{
public:
    void check() override;
    void indicate(const artery::DenmObject&) override;
    void handleStoryboardTrigger(const StoryboardSignal&) override;

protected:
    void initialize(int) override;

    bool checkConditions();
    bool checkEgoDeceleration() const;
    bool checkEgoSpeed() const;

    vanetza::asn1::Denm createMessage();
    vanetza::btp::DataRequestB createRequest();

private:
    SkipEarlySampler<vanetza::units::Acceleration> mAccelerationSampler;
    vanetza::units::Velocity mSpeedThreshold;
    vanetza::units::Acceleration mDecelerationThreshold;
    bool mRWWVehicle = false;
    //double prevDistance = 0.0;
    double distance; 
    V2XUtils *utils;
    double hvHeading, rwHeading;
    //float X,Y;
    //float eX,eY;
    //float sX,sY;
    //float LineSegHd, LineSegLen;
    bool mRWWFlag = false;
};

} // namespace den
} // namespace artery

#endif /* ARTERY_EMERGENCYVEHICLEWARNING_H_QRTLCYIY */
