/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/den/StationaryVehicleTx.h"
#include "artery/application/SampleBufferAlgorithm.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/asn1/its/DangerousSituationSubCauseCode.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/velocity.hpp>
#include <algorithm>
#include "artery/application/StoryboardSignal.h"


#include <vanetza/units/area.hpp>
#include <vanetza/geonet/areas.hpp>

#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <vanetza/units/curvature.hpp>
#include <cstdint>
#include <map>

#include <boost/units/systems/si/prefixes.hpp>
#include <vanetza/asn1/denm.hpp>
//#include "artery/application/ItsG5Service.h"
//#include "artery/application/ItsG5BaseService.h"

namespace artery
{
namespace den
{

Define_Module(StationaryVehicleTx)
const double anglePrecision = 10000000.0;
const auto microdegree = vanetza::units::degree * boost::units::si::micro;
const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 30;
const uint16_t EVW_LEVEL_0 = 50;
const uint16_t EVW_LEVEL_1 = 200;
const uint16_t EVW_LEVEL_2 = 400;
const long STATIONARY_SINCE = 4;
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}
void StationaryVehicleTx::initialize(int stage)
{
    UseCase::initialize(stage);
    if (stage == 0)
    {
        using boost::units::si::meter_per_second;
        using boost::units::si::meter_per_second_squared;

        mAccelerationSampler.setDuration(par("sampleDuration"));
        mAccelerationSampler.setInterval(par("sampleInterval"));
        mSpeedThreshold = par("speedThreshold").doubleValue() * meter_per_second;
        mDecelerationThreshold = par("decelerationThreshold").doubleValue() * meter_per_second_squared;
        utils = new V2XUtils();
    }
}

void StationaryVehicleTx::check()
{
 
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    auto condi = checkConditions();
    //std::cout<<"check condition"<<condi<<std::endl;
    if (mSVFlag && condi == true)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        auto message = createMessage();
        auto request = createRequest();
        mService->sendDenm(std::move(message), request);
    }
}

bool StationaryVehicleTx::checkConditions()
{
    return checkEgoSpeed();// && checkEgoDeceleration();
}

bool StationaryVehicleTx::checkEgoSpeed() const
{
    return mVdp->speed() < (1 * boost::units::si::meter_per_second);
}

bool StationaryVehicleTx::checkEgoDeceleration() const
{
    const auto& samples = mAccelerationSampler.buffer();
    return samples.full() && std::all_of(samples.begin(), samples.end(),
        [this](const Sample<vanetza::units::Acceleration>& sample) -> bool {
            return sample.value < mDecelerationThreshold;
        });
}

vanetza::asn1::Denm StationaryVehicleTx::createMessage()
{
    auto msg = createMessageSkeleton();
    msg->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *msg->denm.management.relevanceDistance = RelevanceDistance_lessThan500m;
    msg->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *msg->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    msg->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *msg->denm.management.validityDuration = 2;
    msg->denm.management.stationType = StationType_unknown; // TODO retrieve type from SUMO

    msg->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    msg->denm.situation->informationQuality = 1;
    msg->denm.situation->eventType.causeCode = CauseCodeType_stationaryVehicle;
    msg->denm.situation->eventType.subCauseCode = DangerousSituationSubCauseCode_emergencyElectronicBrakeEngaged;
    
    msg->denm.alacarte = vanetza::asn1::allocate<AlacarteContainer_t>();
    msg->denm.alacarte->stationaryVehicle = vanetza::asn1::allocate<StationaryVehicleContainer_t>();
    msg->denm.alacarte->stationaryVehicle->stationarySince = vanetza::asn1::allocate<StationarySince_t>();
    //StationaryVehicleContainer_t &svContainer = *msg->denm.alacarte->stationaryVehicle;
    //StationarySince_t *svStationarySince = vanetza::asn1::allocate<StationarySince_t>();
    //*svStationarySince = StationarySince_lessThan2Minutes;
    *msg->denm.alacarte->stationaryVehicle->stationarySince = StationarySince_lessThan2Minutes;
    //ASN_SEQUENCE_ADD(&svContainer.stationarySince, svStationarySince);
    msg->denm.alacarte->stationaryVehicle->stationaryCause = vanetza::asn1::allocate<CauseCode_t>();
    msg->denm.alacarte->stationaryVehicle->stationaryCause->causeCode = CauseCodeType_stationaryVehicle;
    msg->denm.alacarte->stationaryVehicle->stationaryCause->subCauseCode = DangerousSituationSubCauseCode_emergencyElectronicBrakeEngaged;
    //msg->denm.alacarte->stationaryVehicle->energyStorageType = vanetza::asn1::allocate<EnergyStorageType_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification = vanetza::asn1::allocate<VehicleIdentification_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber = 
    vanetza::asn1::allocate<WMInumber_t>();
    //std::string s = "123";
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(3));
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->size = 3;

    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[0] = 'a';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[1] = 'b';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[2] = 'c';
    //*msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber = s;
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS = vanetza::asn1::allocate<VDS_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(6));
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->size = 6;

    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[0] = 'e';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[1] = 'f';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[2] = 'g';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[3] = 'h';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[4] = 'i';
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[5] = 'j';
    //*msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS = s;
    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    return msg;
}

vanetza::btp::DataRequestB StationaryVehicleTx::createRequest()
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;

    vanetza::btp::DataRequestB request;
    request.gn.traffic_class.tc_id(0);
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 2 };

    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = 500.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude = mVdp->latitude();
    destination.position.longitude = mVdp->longitude();
    request.gn.destination = destination;
    //auto head = mVdp->heading().value() * 5729.58;
    //std::cout<<"EVW heading "<<head<<" Station ID "<<mVdp->getStationId()<<std::endl;
    //std::cout<<"lat "<<mVdp->latitude().value()<<" lon "<<mVdp->longitude().value()<<std::endl;

    return request;
}
void StationaryVehicleTx::indicate(const artery::DenmObject& denm)
{
    
}
void StationaryVehicleTx::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "SV") {
        mSVFlag = true;
        std::cout<<"SV set \n";
    }
}
} // namespace den
} // namespace artery
