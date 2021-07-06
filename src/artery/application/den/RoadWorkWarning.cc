/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/den/RoadWorkWarning.h"
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

Define_Module(RoadWorkWarning)
const double anglePrecision = 10000000.0;
const auto microdegree = vanetza::units::degree * boost::units::si::micro;
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 45;
const uint16_t EVW_LEVEL_0 = 50;
const uint16_t EVW_LEVEL_1 = 200;
const uint16_t EVW_LEVEL_2 = 400;
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}
void RoadWorkWarning::initialize(int stage)
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

void RoadWorkWarning::check()
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

bool RoadWorkWarning::checkConditions()
{
    return checkEgoSpeed() && checkEgoDeceleration();
}

bool RoadWorkWarning::checkEgoSpeed() const
{
    return mVdp->speed() > mSpeedThreshold;
}

bool RoadWorkWarning::checkEgoDeceleration() const
{
    const auto& samples = mAccelerationSampler.buffer();
    return samples.full() && std::all_of(samples.begin(), samples.end(),
        [this](const Sample<vanetza::units::Acceleration>& sample) -> bool {
            return sample.value < mDecelerationThreshold;
        });
}

vanetza::asn1::Denm RoadWorkWarning::createMessage()
{
    auto msg = createMessageSkeleton();
    msg->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *msg->denm.management.relevanceDistance = RelevanceDistance_lessThan500m;
    msg->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *msg->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    msg->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *msg->denm.management.validityDuration = 2;
    msg->denm.management.stationType = StationType_unknown; // TODO retrieve type from SUMO
    
    msg->denm.management.eventPosition.longitude = 11029799 * Longitude_oneMicrodegreeEast;
    msg->denm.management.eventPosition.latitude =  49573085 * Latitude_oneMicrodegreeNorth;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;


    msg->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    msg->denm.situation->informationQuality = 1;
    msg->denm.situation->eventType.causeCode = CauseCodeType_roadworks;
    msg->denm.situation->eventType.subCauseCode = DangerousSituationSubCauseCode_collisionRiskWarningEngaged;

    //message->denm.location->eventPositionHeading = vanetza::asn1::allocate<Heading>();
    msg->denm.location->eventPositionHeading->headingValue = 463;//round(mVdp->heading(), decidegree);
    msg->denm.location->eventPositionHeading->headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    //std::cout<<"event heading "<<round(mVdp->heading(), decidegree)<<std::endl;
    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    //std::cout<<"RWW DENM sent\n";
    return msg;
}

vanetza::btp::DataRequestB RoadWorkWarning::createRequest()
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;

    vanetza::btp::DataRequestB request;
    request.gn.traffic_class.tc_id(0);
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 2 };

    geonet::Area destination;
    geonet::Rectangle destination_shape;
    destination_shape.a = 500.0 * meter;
    destination_shape.b = 20.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude  = 49573085 * vanetza::units::degree;//mVdp->latitude();
    destination.position.longitude = 11029799 * vanetza::units::degree;//mVdp->longitude();
    request.gn.destination = destination;
    //auto head = mVdp->heading().value() * 5729.58;
    //std::cout<<"EVW heading "<<head<<" Station ID "<<mVdp->getStationId()<<std::endl;
    //std::cout<<"lat "<<mVdp->latitude().value()<<" lon "<<mVdp->longitude().value()<<std::endl;

    return request;
}
void RoadWorkWarning::indicate(const artery::DenmObject& denm)
{
    //std::cout<<"denm\n";
    if (denm & CauseCode::EmergencyVehicleApproaching) {
       // std::cout<<"EVW denm\n";
    }
    if (denm & CauseCode::Roadworks) {
        //std::cout<<"RWW denm!!!!\n";
        const vanetza::asn1::Denm& asn1 = denm.asn1();

        hvHeading = vanetza::units::GeoAngle { mVdp->heading() } / vanetza::units::degree;
        //std::cout<<"HV heading "<<hvHeading<<" Station ID "<<mVdp->getStationId()<<std::endl;
        //std::cout<<asn1->header.stationID<<std::endl;
        //std::cout<<asn1->header.messageID<<std::endl;

        //std::cout<<"Cause code"<<asn1->denm.situation->eventType.causeCode<<std::endl;
        //std::cout<<"heading"<<asn1->denm.location->eventPositionHeading->headingValue<<std::endl;
        auto longitude = round(mVdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	    auto latitude = round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
        
        auto latit_1 = asn1->denm.management.eventPosition.latitude;
        auto longi_1 = asn1->denm.management.eventPosition.longitude;
    
        evwHeading = asn1->denm.location->eventPositionHeading->headingValue / 10;
        distance = utils->distance(latitude/anglePrecision, longitude/anglePrecision,
        latit_1/anglePrecision, longi_1/anglePrecision);
        /*
        std::cout<<"EVV heading "<<evwHeading<< " host heading "<<hvHeading
        <<" distance "<<distance<<"\nlati "<<latit_1<<" longi "<<longi_1
        <<"\nlati0 "<<latitude<<" longi0 "<<longitude<<std::endl;
        */
        //utils->boostLatLongtoXY(latitude/anglePrecision,longitude/anglePrecision,sX,sY);
        //std::cout<<"X "<<sX<<" Y "<<sY<<std::endl;
        //utils->boostLatLongtoXY(latit_1/anglePrecision,longi_1/anglePrecision,eX,eY);
        //std::cout<<"X "<<eX<<" Y "<<eY<<std::endl;

        /*utils->projLatLongtoXY(stGeoMerc, latitude/anglePrecision, longitude/anglePrecision, sX, sY);
        std::cout<<"X "<<sX<<" Y "<<sY<<std::endl;
        utils->projLatLongtoXY(stGeoMerc, latit_1/anglePrecision, longi_1/anglePrecision, eX, eY);
        std::cout<<"X "<<eX<<" Y "<<eY<<std::endl;
        */
        auto cordAngle = utils->CoordinatesToAngle(latitude/anglePrecision, longitude/anglePrecision,
                            latit_1/anglePrecision, longi_1/anglePrecision);

        //utils->get_line_heading_length(eX, eY, sX, sY, LineSegHd, LineSegLen);
        /*
        std::cout<<"CoordinatesToAngle "
        <<cordAngle<<" diff "<< static_cast<uint16_t>(abs(hvHeading - cordAngle))
        <<" 180diff "<< static_cast<uint16_t>(abs(180 - hvHeading - cordAngle)) <<std::endl;
        */
        //LineSegHd *= RAD_TO_DEG;
        //std::cout<<"LineSegHd "<<LineSegHd;
        //std::cout<<" hvHeading "<<hvHeading;
        /*std::cout<<"head diff "<<static_cast<uint16_t>(abs(evwHeading- hvHeading))
        <<" bear diff "<<static_cast<uint16_t>(abs(hvHeading - LineSegHd))
        <<" dist " <<distance<<std::endl;*/
        if(static_cast<uint16_t>(distance) < EVW_LEVEL_0 && static_cast<int16_t>(cordAngle) > 0 )//&& (distance > prevDistance))
        {
            std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_0 "<<std::endl;
            mEVWFlag = true;
        }
        //if(static_cast<uint16_t>(abs(evwHeading- hvHeading)) < HEADING_COMPENSATION)
        //{
        if(static_cast<uint16_t>(abs(hvHeading - cordAngle)) < BEARING_COMPENSATION  &&
            static_cast<int16_t>(cordAngle) > 0)
        {
            //i32Ret = 1;
            if (!mEVWFlag)
            {
                if(static_cast<uint16_t>(distance) < EVW_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_1 111111111"<<std::endl;
                    mEVWFlag = true;
                }
                else //if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_2 11111111111"<<std::endl;
                    mEVWFlag = true;
                }
            }


        }
        mEVWFlag = false;
    }
    if (denm & CauseCode::DangerousSituation) {
            const vanetza::asn1::Denm& asn1 = denm.asn1();
            //std::cout<<"RoadWorkWarning::DangerousSituation indicate_2"<<std::endl;
        }
    
}
void RoadWorkWarning::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "RWW") {
        mEVWVehicle = true;
        std::cout<<"RWW set \n";
    }
}
} // namespace den
} // namespace artery
