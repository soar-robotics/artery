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
const auto decidegree = vanetza::units::degree * boost::units::si::deci;
const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 20;
const uint16_t FCW_LEVEL_0 = 3;
const uint16_t FCW_LEVEL_1 = 5;
const uint16_t FCW_LEVEL_2 = 9;

constexpr uint16_t SHM_KEY_RX = 0x5005;
constexpr uint16_t SHM_KEY_TX = 0x5006;
//constexpr uint16_t SHM_KEY_CV = 0x6001;
constexpr uint16_t SHM_KEY_OV = 0x6002;

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
        /*
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
*/
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
        
    }
}

/*
void ForwardCollisionWarningRx::copyEVWVehicleData()
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
    std::cout<<"EVW Lat :"<<round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth<<" "<<
    mVehicleInfoCv.latitude<<" "<<mVehicleInfoCv.longitude    <<std::endl;
    
    std::cout<<"Writing copyEVWVehicleData: Complete "<<m_EVWDENMCv.EVWTxData.latitude<<
         " "<<m_EVWDENMCv.EVWTxData.latitude<<" "<<m_EVWDENMCv.EVWTxData.heading<<std::endl;
    

    m_pSHMSegmentCv->cnt = sizeof(mVehicleInfoCv);
    m_pSHMSegmentCv->complete = 0;
    //buffer = m_pSHMSegment->buf;
    m_pSHMSegmentCv->validData = true;
    memcpy(m_pSHMSegmentCv->buf,&mVehicleInfoCv,m_pSHMSegmentCv->cnt);
    //spaceavailable = BUF_SIZE;
    //printf("Writing copyEVWVehicleData: Shared Memory Write: Wrote %d bytes smhid %d\n", m_pSHMSegmentCv->cnt,m_SHMIdTx);
    m_pSHMSegmentCv->complete = 1;
    
    std::cout<<"Writing copyEVWVehicleData: Complete "<<m_EVWDENMCv.EVWTxData.latitude<<
    " "<<m_EVWDENMCv.EVWTxData.latitude<<" "<<m_EVWDENMCv.EVWTxData.heading<<std::endl;
    
}
*/

void ForwardCollisionWarningRx::check()
{
 /*
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    if (mFCWVehicle)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        copyEVWVehicleData();
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

void ForwardCollisionWarningRx::copyOVData()
{
    //m_EVWDENMOv.header = 5678;
        //m_EVWDENMTx.payloadSize = sizeof(m_EVWDENMTx);
    //m_EVWDENMOv.payloadSize = sizeof(EVWTxData_t);
    mVehicleInfoOv.stationId = mVdp->station_id();
    //m_EVWDENMTx.packetCounter =  ;
    mVehicleInfoOv.latitude = round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
    mVehicleInfoOv.longitude = round(mVdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
    mVehicleInfoOv.speed = std::abs(round(mVdp->speed(), centimeter_per_second)) * SpeedValue_oneCentimeterPerSec;
    mVehicleInfoOv.heading = round(mVdp->heading(), decidegree);
    
    //std::cout<<"OV Lat :"<<round(mVdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth<<" "<<
    //m_EVWDENMOv.EVWTxData.latitude<<" "<<m_EVWDENMOv.EVWTxData.longitude    <<std::endl;

    m_pSHMSegmentOv->cnt = sizeof(mVehicleInfoOv);
    m_pSHMSegmentOv->complete = 0;
    //buffer = m_pSHMSegment->buf;
    m_pSHMSegmentOv->validData = true;
    memcpy(m_pSHMSegmentOv->buf,&mVehicleInfoOv,m_pSHMSegmentOv->cnt);
    //spaceavailable = BUF_SIZE;
    //printf("Writing copyOVehicleData: Shared Memory Write: Wrote %d bytes smhid %d\n", m_pSHMSegmentOv->cnt,m_SHMIdTx);
    m_pSHMSegmentOv->complete = 1;
    //std::cout<<"Writing copyOVehicleData: Complete\n";
}

void ForwardCollisionWarningRx::indicate(const artery::DenmObject& denm)
{
    if (denm & CauseCode::StationaryVehicle) {
        const vanetza::asn1::Denm& asn1 = denm.asn1();
        copyOVData();
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

        /*
        std::cout<<"CoordinatesToAngle "
        <<cordAngle<<" diff "<< static_cast<uint16_t>(abs(hvHeading - cordAngle))
        <<" 180diff "<< static_cast<uint16_t>(abs(180 - hvHeading - cordAngle)) <<std::endl;
        */
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
       m_EVWDENMTx.header = 1234;
        //m_EVWDENMTx.payloadSize = sizeof(m_EVWDENMTx);
        m_EVWDENMTx.payloadSize = sizeof(EVWTxData_t);
        m_EVWDENMTx.EVWTxData.stationID = asn1->header.messageID;
        m_EVWDENMTx.packetCounter = asn1->denm.management.actionID.sequenceNumber ;
        m_EVWDENMTx.EVWTxData.latitude = asn1->denm.management.eventPosition.latitude;
        m_EVWDENMTx.EVWTxData.longitude = asn1->denm.management.eventPosition.longitude;
        m_EVWDENMTx.EVWTxData.speed = 4;
        m_EVWDENMTx.EVWTxData.heading = asn1->denm.location->eventPositionHeading->headingValue;
        m_EVWDENMTx.EVWTxData.causeCode = asn1->denm.situation->eventType.causeCode;
        m_EVWDENMTx.EVWTxData.subCauseCode = asn1->denm.situation->eventType.subCauseCode;
            
        m_pSHMSegmentTx->cnt = sizeof(m_EVWDENMTx);
        m_pSHMSegmentTx->complete = 0;
        //buffer = m_pSHMSegment->buf;
        m_pSHMSegmentTx->validData = true;
        memcpy(m_pSHMSegmentTx->buf,&m_EVWDENMTx,m_pSHMSegmentTx->cnt);
        //spaceavailable = BUF_SIZE;
        //printf("Writing Process: Shared Memory Write: Wrote %d bytes smhid %d\n", m_pSHMSegmentTx->cnt,m_SHMIdTx);
        m_pSHMSegmentTx->complete = 1;
    }

    
}
void ForwardCollisionWarningRx::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "FCW") {
        mFCWVehicle = true;
        std::cout<<"FCW set \n";
    }
}
} // namespace den
} // namespace artery
