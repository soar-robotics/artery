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
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 30;
const uint16_t EVW_LEVEL_0 = 50;
const uint16_t EVW_LEVEL_1 = 200;
const uint16_t EVW_LEVEL_2 = 400;
const long STATIONARY_SINCE = 4;
constexpr uint16_t SHM_KEY_RX = 0x5005;
constexpr uint16_t SHM_KEY_TX = 0x5006;
constexpr uint16_t SHM_KEY_CV = 0x6002;
//constexpr uint16_t SHM_KEY_OV = 0x6002;

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

        //IPC data transfer
        m_SHMIdRx = shmget(SHM_KEY_RX, 1024, 0644|IPC_CREAT);
        if (m_SHMIdRx == -1) 
        {
            perror("Shared memory");
        //return 1;
        }

        // Attach to the segment to get a pointer to it.
        m_pSHMSegmentRx = reinterpret_cast<SHMSegment_t *>(shmat(m_SHMIdRx, NULL, 0));
        if (m_pSHMSegmentRx == (void *) -1) 
        {
            perror("Shared memory attach");
        //return 1;
        }
        memset(m_pSHMSegmentRx, 0, 1024);
        m_pSHMSegmentRx->validData = false;
        std::cout<<"shmid Rx: "<<m_SHMIdRx<<std::endl;
        
        m_SHMIdTx = shmget(SHM_KEY_TX, 1024, 0644|IPC_CREAT);
        if (m_SHMIdTx == -1) 
        {
            perror("Shared memory");
        //return 1;
        }

        // Attach to the segment to get a pointer to it.
        m_pSHMSegmentTx = reinterpret_cast<SHMSegment_t *>(shmat(m_SHMIdTx, NULL, 0));
        if (m_pSHMSegmentTx == (void *) -1) 
        {
            perror("Shared memory attach");
        //return 1;
        }
        std::cout<<"shmid Tx: "<<m_SHMIdTx<<std::endl;
        memset(m_pSHMSegmentTx, 0, 1024);
        m_pSHMSegmentTx->validData = false;
        
        m_SHMIdCv = shmget(SHM_KEY_CV, 1024, 0644|IPC_CREAT);
        if (m_SHMIdCv == -1) 
        {
            perror("Shared memory");
        //return 1;
        }

        // Attach to the segment to get a pointer to it.
        m_pSHMSegmentCv = reinterpret_cast<SHMSegment_t *>(shmat(m_SHMIdCv, NULL, 0));
        if (m_pSHMSegmentCv == (void *) -1) 
        {
            perror("Shared memory attach");
        //return 1;
        }
        std::cout<<"shmid Cv: "<<m_SHMIdCv<<std::endl;
        memset(m_pSHMSegmentCv, 0, 1024);
        m_pSHMSegmentCv->validData = false;
/*
        m_SHMIdOv = shmget(SHM_KEY_OV, 1024, 0644|IPC_CREAT);
        if (m_SHMIdOv == -1) 
        {
            perror("Shared memory");
        //return 1;
        }

        // Attach to the segment to get a pointer to it.
        m_pSHMSegmentOv = reinterpret_cast<SHMSegment_t *>(shmat(m_SHMIdOv, NULL, 0));
        if (m_pSHMSegmentOv == (void *) -1) 
        {
            perror("Shared memory attach");
        //return 1;
        }
        std::cout<<"shmid Ov: "<<m_SHMIdOv<<std::endl;
        memset(m_pSHMSegmentOv, 0, 1024);
        m_pSHMSegmentOv->validData = false;
        */
    }
}

void StationaryVehicleTx::copyEVWVehicleData()
{
    //m_EVWDENMCv.header = 5678;
        //m_EVWDENMTx.payloadSize = sizeof(m_EVWDENMTx);
   // m_EVWDENMCv.payloadSize = sizeof(EVWTxData_t);
    mVehicleInfoCv.stationId = mVdp->station_id();
    //m_EVWDENMTx.packetCounter =  ;
    mVehicleInfoCv.latitude = round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    mVehicleInfoCv.longitude = round(mVdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    mVehicleInfoCv.speed = std::abs(round(mVdp->speed(), centimeter_per_second)) * SpeedValue_oneCentimeterPerSec;
    mVehicleInfoCv.heading = round(mVdp->heading(), decidegree);
    //std::cout<<"EVW Lat :"<<round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth<<" "<<
    //mVehicleInfoCv.latitude<<" "<<mVehicleInfoCv.longitude    <<std::endl;
    
    //std::cout<<"Writing copyEVWVehicleData: Complete "<<m_EVWDENMCv.EVWTxData.latitude<<
    //     " "<<m_EVWDENMCv.EVWTxData.latitude<<" "<<m_EVWDENMCv.EVWTxData.heading<<std::endl;
    

    m_pSHMSegmentCv->cnt = sizeof(mVehicleInfoCv);
    m_pSHMSegmentCv->complete = 0;
    //buffer = m_pSHMSegment->buf;
    m_pSHMSegmentCv->validData = true;
    memcpy(m_pSHMSegmentCv->buf,&mVehicleInfoCv,m_pSHMSegmentCv->cnt);
    //spaceavailable = BUF_SIZE;
    //printf("Writing copyEVWVehicleData: Shared Memory Write: Wrote %d bytes smhid %d\n", m_pSHMSegmentCv->cnt,m_SHMIdTx);
    m_pSHMSegmentCv->complete = 1;
    
    //std::cout<<"Writing copyEVWVehicleData: Complete "<<m_EVWDENMCv.EVWTxData.latitude<<
    //" "<<m_EVWDENMCv.EVWTxData.latitude<<" "<<m_EVWDENMCv.EVWTxData.heading<<std::endl;
    
}
void StationaryVehicleTx::check()
{
 
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    auto condi = checkConditions();
    copyEVWVehicleData();
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

    // TODO set road type in Location container
    // TODO set lane position in Alacarte container

    if(m_pSHMSegmentRx->validData == true)
    {
    	memcpy(&m_EVWDENMRx, m_pSHMSegmentRx->buf, sizeof(EVWDENM_t));
    	//std::cout << "RWW valid data\n";
    	//std::cout << m_EVWDENMRx.header<<" "<<m_EVWDENMRx.payloadSize<<" \n";

        //msg->header.messageID = m_EVWDENMRx.EVWTxData.stationID;
        msg->denm.management.actionID.sequenceNumber = m_EVWDENMRx.packetCounter;
        msg->denm.management.eventPosition.latitude = m_EVWDENMRx.EVWTxData.latitude;
        msg->denm.management.eventPosition.longitude = m_EVWDENMRx.EVWTxData.longitude;
        //m_EVWDENMTx.EVWTxData.speed = 4;
        //m_EVWDENMTx.EVWTxData.heading = msg->denm.location->eventPositionHeading->headingValue;
        msg->denm.situation->eventType.causeCode = m_EVWDENMRx.EVWTxData.causeCode;
        msg->denm.situation->eventType.subCauseCode = m_EVWDENMRx.EVWTxData.subCauseCode;
        m_pSHMSegmentRx->validData = false;
        //std::cout<<"SV lat: "<<m_EVWDENMRx.EVWTxData.latitude * Latitude_oneMicrodegreeNorth<<std::endl;
        //std::cout<<"SV lat: "<<m_EVWDENMRx.EVWTxData.latitude <<std::endl;

    }

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
