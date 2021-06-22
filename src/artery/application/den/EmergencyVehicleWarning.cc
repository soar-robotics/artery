/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/den/EmergencyVehicleWarning.h"
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

Define_Module(EmergencyVehicleWarning)
const double anglePrecision = 10000000.0;
const auto microdegree = vanetza::units::degree * boost::units::si::micro;
const uint16_t HEADING_COMPENSATION_FOLLOW= 35;
const uint16_t HEADING_COMPENSATION_OPPOSITE = 135;
const uint16_t EEBL_LEVEL_1 = 400;
const uint16_t EEBL_LEVEL_2 = 200;
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}
void EmergencyVehicleWarning::initialize(int stage)
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

void EmergencyVehicleWarning::check()
{
 
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    if (mEVWVehicle)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        auto message = createMessage();
        auto request = createRequest();
        mService->sendDenm(std::move(message), request);
    }
}

bool EmergencyVehicleWarning::checkConditions()
{
    return checkEgoSpeed() && checkEgoDeceleration();
}

bool EmergencyVehicleWarning::checkEgoSpeed() const
{
    return mVdp->speed() > mSpeedThreshold;
}

bool EmergencyVehicleWarning::checkEgoDeceleration() const
{
    const auto& samples = mAccelerationSampler.buffer();
    return samples.full() && std::all_of(samples.begin(), samples.end(),
        [this](const Sample<vanetza::units::Acceleration>& sample) -> bool {
            return sample.value < mDecelerationThreshold;
        });
}

vanetza::asn1::Denm EmergencyVehicleWarning::createMessage()
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
    msg->denm.situation->eventType.causeCode = CauseCodeType_emergencyVehicleApproaching;
    msg->denm.situation->eventType.subCauseCode = DangerousSituationSubCauseCode_emergencyElectronicBrakeEngaged;

    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    return msg;
}

vanetza::btp::DataRequestB EmergencyVehicleWarning::createRequest()
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
void EmergencyVehicleWarning::indicate(const artery::DenmObject& denm)
{
    /*auto cc = denm.situation_cause_code();
    if (den::CauseCode::EmergencyVehicleApproaching == cc)
    {
        std::cout<<"EmergencyVehicleWarning::EmergencyVehicleApproaching indicate"<<std::endl;
    }
    else if (den::CauseCode::DangerousSituation == cc)
    {
        std::cout<<"EmergencyVehicleWarning::DangerousSituation indicate"<<std::endl;
    }*/
    if (denm & CauseCode::EmergencyVehicleApproaching) {
        const vanetza::asn1::Denm& asn1 = denm.asn1();

        hvHeading = vanetza::units::GeoAngle { mVdp->heading() } / vanetza::units::degree;
        std::cout<<"HV heading "<<hvHeading<<" Station ID "<<mVdp->getStationId()<<std::endl;
        //std::cout<<asn1->header.stationID<<std::endl;
        //std::cout<<asn1->header.messageID<<std::endl;

        //std::cout<<"Cause code"<<asn1->denm.situation->eventType.causeCode<<std::endl;
        //std::cout<<"heading"<<asn1->denm.location->eventPositionHeading->headingValue<<std::endl;
        auto longitude = round(mVdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	    auto latitude = round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
        
        auto latit_1 = asn1->denm.management.eventPosition.latitude;
        auto longi_1 = asn1->denm.management.eventPosition.longitude;
    
        evwHeading = asn1->denm.location->eventPositionHeading->headingValue / 10;
        distance = utils->distance(static_cast<double>(latitude/anglePrecision), longitude/anglePrecision,
        latit_1/anglePrecision, longi_1/anglePrecision);
        std::cout<<"EVV heading "<<evwHeading<< "denm dist "<<distance<<std::endl;
        
        if(distance < prevDistance)
        {
            if(static_cast<uint16_t>(abs(hvHeading - evwHeading)) < HEADING_COMPENSATION_FOLLOW)
            {
                if(static_cast<uint16_t>(distance) < EEBL_LEVEL_2)
                {
                    std::cout<<"EmergencyVehicleWarning::Level_2 f"<<std::endl;
                }
                else if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"EmergencyVehicleWarning::Level_1 f"<<std::endl;
                }
            }
        }
        else if(distance > prevDistance)
        {
            if(static_cast<uint16_t>(abs(hvHeading - evwHeading)) < HEADING_COMPENSATION_OPPOSITE)
            {
                if(static_cast<uint16_t>(distance) < EEBL_LEVEL_2)
                {
                    std::cout<<"EmergencyVehicleWarning::Level_2 o"<<std::endl;
                }
                else if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"EmergencyVehicleWarning::Level_1 o"<<std::endl;
                }
            }
        }


        //std::cout<<"EmergencyVehicleWarning::EmergencyVehicleApproaching indicate_1 "<<id<<std::endl;
        /*
        if (asn1->denm.alacarte && asn1->denm.alacarte->impactReduction) {
            auto& indication = asn1->denm.alacarte->impactReduction->requestResponseIndication;
            if (indication == RequestResponseIndication_request) {
                transmitMessage(RequestResponseIndication_response);
            }
        }
        */
    }
    if (denm & CauseCode::DangerousSituation) {
            const vanetza::asn1::Denm& asn1 = denm.asn1();
            //std::cout<<"EmergencyVehicleWarning::DangerousSituation indicate_2"<<std::endl;
        }
    //auto denm = denmObject1.shared_ptr();
    //denmObject1.asn1()->denm.situation;

    
}
void EmergencyVehicleWarning::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "EVW") {
        mEVWVehicle = true;
        std::cout<<"EVW set \n";
    }
}
} // namespace den
} // namespace artery
