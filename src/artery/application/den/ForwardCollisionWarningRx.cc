/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/den/ForwardCollisionWarningRx.h"
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
    
Define_Module(ForwardCollisionWarningRx)
const double anglePrecision = 10000000.0;
const auto microdegree = vanetza::units::degree * boost::units::si::micro;
const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 20;
const uint16_t FCW_LEVEL_0 = 3;
const uint16_t FCW_LEVEL_1 = 5;
const uint16_t FCW_LEVEL_2 = 9;
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}
void ForwardCollisionWarningRx::initialize(int stage)
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

void ForwardCollisionWarningRx::check()
{
 /*
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    if (mFCWVehicle)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        auto message = createMessage();
        auto request = createRequest();
        mService->sendDenm(std::move(message), request);
    }
    */
}

bool ForwardCollisionWarningRx::checkConditions()
{
    return checkEgoSpeed() && checkEgoDeceleration();
}

bool ForwardCollisionWarningRx::checkEgoSpeed() const
{
    return mVdp->speed() > mSpeedThreshold;
}

bool ForwardCollisionWarningRx::checkEgoDeceleration() const
{
    const auto& samples = mAccelerationSampler.buffer();
    return samples.full() && std::all_of(samples.begin(), samples.end(),
        [this](const Sample<vanetza::units::Acceleration>& sample) -> bool {
            return sample.value < mDecelerationThreshold;
        });
}

vanetza::asn1::Denm ForwardCollisionWarningRx::createMessage()
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
    std::cout<<"message sended"<<std::endl;
    return msg;
}

vanetza::btp::DataRequestB ForwardCollisionWarningRx::createRequest()
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
void ForwardCollisionWarningRx::indicate(const artery::DenmObject& denm)
{
    if (denm & CauseCode::StationaryVehicle) {
        const vanetza::asn1::Denm& asn1 = denm.asn1();

        hvHeading = vanetza::units::GeoAngle { mVdp->heading() } / vanetza::units::degree;
        auto longitude = round(mVdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	    auto latitude = round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
        hvSpeed = mVdp->speed()/boost::units::si::meter_per_second;
        
        auto latit_1 = asn1->denm.management.eventPosition.latitude;
        auto longi_1 = asn1->denm.management.eventPosition.longitude;
    
        evwHeading = asn1->denm.location->eventPositionHeading->headingValue / 10;
        distance = utils->distance(latitude/anglePrecision, longitude/anglePrecision,
        latit_1/anglePrecision, longi_1/anglePrecision);
        /*
        std::cout<<"SV heading "<<evwHeading<< " host heading "<<hvHeading
        <<" distance "<<distance<<" speed "<<hvSpeed<<std::endl;
        */
        auto cordAngle = utils->CoordinatesToAngle(latitude/anglePrecision, longitude/anglePrecision,
                            latit_1/anglePrecision, longi_1/anglePrecision);

        
        //std::cout<<"RX: "<<std::endl<<"hvHeading: "<<hvHeading<<" longitude: "<<longitude<<" latitude: "<<latitude<<" hvSpeed: "<<hvSpeed<<"evwHeading: "<<evwHeading<<"distance: "<<distance<<" cordAngle: "<<cordAngle<<std::endl;
        
        //std::cout<<"FCW__TTC: "<<(distance/hvSpeed)<<std::endl;
        if(hvSpeed != 0 && static_cast<uint16_t>(distance/hvSpeed) < FCW_LEVEL_0 )//&& (distance > prevDistance))
        {
            std::cout<<"!!!!!!!!!!!!FCW::Level_0 "<<std::endl;
            mEVWFlag = true;
        }
        if(hvSpeed != 0 && static_cast<uint16_t>(abs(hvHeading - cordAngle)) < BEARING_COMPENSATION  &&
            static_cast<int16_t>(cordAngle) > 0)
        {
            //i32Ret = 1;
            if (!mEVWFlag)
            {
                if(static_cast<uint16_t>(distance) < FCW_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!FCW::Level_1 111111111"<<std::endl;
                    mEVWFlag = true;
                }
                else //if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!FCW::Level_2 11111111111"<<std::endl;
                    mEVWFlag = true;
                }
            }


        }
        printDenm(asn1);

        /*
        else if (static_cast<uint16_t>(abs(evwHeading- hvHeading)) < BEARING_COMPENSATION &&
        static_cast<int16_t>(abs(hvHeading - cordAngle)) > 155  && 
        static_cast<int16_t>(abs(hvHeading - cordAngle)) < 205)
        {
            std::cout<<"scast angle "<<static_cast<uint16_t>(cordAngle)<<"\n";
            if (!mEVWFlag)
            {
                if(static_cast<uint16_t>(distance) < EVW_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!EmergencyVehicleWarning::Level_1 2222222222222"<<std::endl;
                }
                else //if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!EmergencyVehicleWarning::Level_2 2222222222222"<<std::endl;
                }
            }

        }
        */
        mEVWFlag = false;
        //prevDistance = distance;
        //}
        /*
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

        */
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

    
}
void ForwardCollisionWarningRx::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "FCW") {
        mFCWVehicle = true;
        std::cout<<"FCW set \n";
    }
}

void ForwardCollisionWarningRx::printDenm(const vanetza::asn1::Denm& message)
{

    const ItsPduHeader_t& header = message->header;
    std::cout<< "RX: "<< "\n";    
    std::cout<< "ITS PDU Header: "<< "\n";
    std::cout<< " Protocol Version: " << header.protocolVersion << "\n";
    std::cout<< " Message ID: " << header.messageID << "\n";
    std::cout<< " Station ID: " << header.stationID << "\n";
    

    const DecentralizedEnvironmentalNotificationMessage_t& denm = message->denm;
    std::cout<< "Decentralized Environmental Notification Message" << "\n";

    //Management Container
    std::cout<< "Action ID:"<<std::endl;
    std::cout<< " Orginationg Station ID: " << static_cast<unsigned long> (denm.management.actionID.originatingStationID) << std::endl;
    std::cout<< " Sequence Number: " <<  static_cast<unsigned long> (denm.management.actionID.sequenceNumber) << std::endl;

    long decTime;
    asn_INTEGER2long(&message->denm.management.detectionTime, &decTime);
    std::cout<< "Detection Time: " << decTime << "\n";
    
    std::cout<< "Event Position: "<<std::endl;
    std::cout<< " Altitude: "<<std::endl;
    std::cout<< "  Altitude Confidence: " << static_cast<long> (denm.management.eventPosition.altitude.altitudeConfidence) << "\n";
    std::cout<< "  Altitude Value: " << static_cast<long> (denm.management.eventPosition.altitude.altitudeValue) << "\n";
    std::cout<< " Latitude: " << static_cast<long> (denm.management.eventPosition.latitude) << "\n";
    std::cout<< " Longitude: " << static_cast<long> (denm.management.eventPosition.longitude) << "\n";
    std::cout<< " Position Confidence Ellipse: " << std::endl;
    std::cout<< "  Semi Major Confidence: " << static_cast<long> (denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence) << "\n";
    std::cout<< "  Semi Major Orientation: " << static_cast<long> (denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation) << "\n";
    std::cout<< "  Semi Minor Confidence: " << static_cast<long> (denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence) << "\n";

    long refTime ;
    asn_INTEGER2long(&message->denm.management.referenceTime, &refTime);
    std::cout<< "Reference Time: " << refTime << std::endl;

    std::cout<< "Relevance Distance: " << denm.management.relevanceDistance << "\n";
    std::cout<< "Relevance Traffic Direction: " << denm.management.relevanceTrafficDirection << "\n";
    std::cout<< "Station Type: " << denm.management.stationType << "\n";
    std::cout<< "Termination: " << denm.management.termination << "\n";
    std::cout<< "Transmission Interval: " << denm.management.transmissionInterval << "\n";
    std::cout<< "Validity Duration: " << denm.management.validityDuration << "\n";

    //Situation Container
    std::cout<< "Event History: " << denm.situation->eventHistory << "\n";
    std::cout<< "Event Type: "<<std::endl;
    std::cout<< " Cause Code: " << denm.situation->eventType.causeCode << "\n";
    std::cout<< " Subcause Code: " << denm.situation->eventType.subCauseCode << "\n";

    std::cout<< "Information Quality: " << denm.situation->informationQuality << "\n";
    std::cout<< "Linked Cause: " << denm.situation->linkedCause << "\n";
    
    //LocationContainer
    std::cout<< "Event Position Heading:\n Heading Confidence: " << denm.location->eventPositionHeading->headingConfidence << "\n";
    std::cout<< " Heading Value: " << denm.location->eventPositionHeading->headingValue << "\n";
    std::cout<< "Event Speed: "<<std::endl;
    std::cout<< " Speed Confidence: " << denm.location->eventSpeed->speedConfidence << "\n";
    std::cout<< " Speed Value: " << denm.location->eventSpeed->speedValue << "\n";

    std::cout<< "Road Type: " << denm.location->roadType << "\n";
    std::cout<< "Traces: " << denm.location->traces.list.count << "\n";
    
    //AlacarteContainer
    if(denm.alacarte != nullptr){
    if(denm.alacarte->externalTemperature != nullptr)
        std::cout<< "External Temperature: " << denm.alacarte->externalTemperature << "\n";

    if(denm.alacarte->impactReduction != nullptr)
        std::cout<< "Impact Reduction: " << denm.alacarte->impactReduction << "\n";

    if(denm.alacarte->lanePosition != nullptr)
        std::cout<< "Lane Position: " << denm.alacarte->lanePosition << "\n";

    if(denm.alacarte->positioningSolution != nullptr)
        std::cout<< "Positioning Solution: " << denm.alacarte->positioningSolution << "\n";

    if(denm.alacarte->roadWorks != nullptr)
        std::cout<< "RoadWorks: " << denm.alacarte->roadWorks << "\n";

    if(denm.alacarte->stationaryVehicle != nullptr)
        std::cout<< "Stationary Vehicle: " << denm.alacarte->stationaryVehicle << "\n";
    }else{
        std::cout<< "ALACARTE CONTAINER IS NULL"<<std::endl;
    }

}
} // namespace den
} // namespace artery
