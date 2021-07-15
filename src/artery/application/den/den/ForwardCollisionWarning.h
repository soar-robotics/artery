/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_FORWARDCOLLISIONWARNING_H_QRTLCYIY
#define ARTERY_FORWARDCOLLISIONWARNING_H_QRTLCYIY

#include "artery/application/SuspendableUseCase.h"
#include "artery/application/NetworkInterface.h"
namespace artery
{
namespace den
{

class ForwardCollisionWarning : public SuspendableUseCase
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
        bool mFCWVehicle = false;
        double distance; 
        V2XUtils *utils;
        double hvHeading, fcwHeading;
        bool mFCWFlag = false;
};

} // namespace den
} // namespace artery

#endif