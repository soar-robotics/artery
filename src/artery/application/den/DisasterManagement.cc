/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/DenService.h"
#include "artery/application/den/DisasterManagement.h"
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

Define_Module(DisasterManagement)

const auto microdegree = vanetza::units::degree * boost::units::si::micro;
const auto DM_MESSAGE = "abcdefghi";
const auto MESSAGE_LENGTH_3 = 3;
const auto MESSAGE_LENGTH_6 = 6;
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}
void DisasterManagement::initialize(int stage)
{
    UseCase::initialize(stage);
    if (stage == 0)
    {
        using boost::units::si::meter_per_second;
        using boost::units::si::meter_per_second_squared;
    }
}

void DisasterManagement::check()
{
    if (mDiasterManagementVehicle)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        auto message = createMessage();
        auto request = createRequest();
        mService->sendDenm(std::move(message), request);
    }
    
}

vanetza::asn1::Denm DisasterManagement::createMessage()
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
    msg->denm.situation->eventType.causeCode = CauseCodeType_disasterManagement;
    msg->denm.situation->eventType.subCauseCode = DangerousSituationSubCauseCode_unavailable;
    
    msg->denm.alacarte = vanetza::asn1::allocate<AlacarteContainer_t>();
    msg->denm.alacarte->stationaryVehicle = vanetza::asn1::allocate<StationaryVehicleContainer_t>();

    msg->denm.alacarte->stationaryVehicle->stationaryCause = vanetza::asn1::allocate<CauseCode_t>();
    msg->denm.alacarte->stationaryVehicle->stationaryCause->causeCode = CauseCodeType_disasterManagement;
    msg->denm.alacarte->stationaryVehicle->stationaryCause->subCauseCode = DangerousSituationSubCauseCode_unavailable;
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification = vanetza::asn1::allocate<VehicleIdentification_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber = 
    vanetza::asn1::allocate<WMInumber_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(3));
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->size = MESSAGE_LENGTH_3;

    std::copy_n(DM_MESSAGE,MESSAGE_LENGTH_3,
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf);

    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS = vanetza::asn1::allocate<VDS_t>();
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(6));
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->size = MESSAGE_LENGTH_6;
    std::copy_n(DM_MESSAGE+MESSAGE_LENGTH_3,MESSAGE_LENGTH_6,
    msg->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf);

    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    return msg;
}


vanetza::btp::DataRequestB DisasterManagement::createRequest()
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
void DisasterManagement::indicate(const artery::DenmObject& denm)
{
    if (denm & CauseCode::DisasterManagement) 
    {
        const vanetza::asn1::Denm& asn1 = denm.asn1();
        std::cout<<"SV DENM received cc:"<<asn1->denm.alacarte->stationaryVehicle->stationaryCause->causeCode <<std::endl;

        std::cout<<"SV DENM received w0:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[0] <<std::endl;
        std::cout<<"SV DENM received w1:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[1] <<std::endl;
        std::cout<<"SV DENM received w2:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->wMInumber->buf[2] <<std::endl;

        std::cout<<"SV DENM received v0:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[0] <<std::endl;
        std::cout<<"SV DENM received v1:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[1] <<std::endl;
        std::cout<<"SV DENM received v2:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[2] <<std::endl;
        std::cout<<"SV DENM received v3:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[3] <<std::endl;
        std::cout<<"SV DENM received v4:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[4] <<std::endl;
        std::cout<<"SV DENM received v5:"<<asn1->denm.alacarte->stationaryVehicle->vehicleIdentification->vDS->buf[5] <<std::endl;
    }

    
}
void DisasterManagement::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "DM") {
        mDiasterManagementVehicle = true;
        std::cout<<"DisasterManagement set \n";
    }
}
} // namespace den
} // namespace artery
