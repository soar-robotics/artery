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
const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

//const uint16_t HEADING_COMPENSATION= 90;
const uint16_t BEARING_COMPENSATION = 45;
const uint16_t RWW_LEVEL_0 = 50;
const uint16_t RWW_LEVEL_1 = 200;
const uint16_t RWW_LEVEL_2 = 400;
const long RWW_LATITUDE = 17594071;
const long RWW_LONGITUDE = 78125249;
const long RWW_HEADING = 180;
const long SPEED_LIMIT = 20;

constexpr uint16_t SHM_KEY_RX = 0x5003;
constexpr uint16_t SHM_KEY_TX = 0x5004;
//constexpr uint16_t SHM_KEY_CV = 0x6001;
constexpr uint16_t SHM_KEY_OV = 0x6002;

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
void RoadWorkWarning::copyEVWVehicleData()
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
void RoadWorkWarning::check()
{
 
    //mAccelerationSampler.feed(mVdp->acceleration(), mVdp->updated());
    if (mRWWVehicle)//!isDetectionBlocked() && checkConditions())
    {
        //blockDetection();
        //copyEVWVehicleData();
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
    
    msg->denm.management.eventPosition.longitude =  RWW_LONGITUDE * Longitude_oneMicrodegreeEast;
    msg->denm.management.eventPosition.latitude = RWW_LATITUDE* Latitude_oneMicrodegreeNorth;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    msg->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;


    msg->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    msg->denm.situation->informationQuality = 1;
    msg->denm.situation->eventType.causeCode = CauseCodeType_roadworks;
    msg->denm.situation->eventType.subCauseCode = DangerousSituationSubCauseCode_collisionRiskWarningEngaged;

    //message->denm.location->eventPositionHeading = vanetza::asn1::allocate<Heading>();
    msg->denm.location->eventPositionHeading->headingValue = RWW_HEADING;//round(mVdp->heading(), decidegree);
    msg->denm.location->eventPositionHeading->headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    //std::cout<<"event heading "<<round(mVdp->heading(), decidegree)<<std::endl;
    // TODO set road type in Location container
    // TODO set lane position in Alacarte container
    msg->denm.alacarte = vanetza::asn1::allocate<AlacarteContainer_t>();
    msg->denm.alacarte->roadWorks =  vanetza::asn1::allocate<RoadWorksContainerExtended_t>();
    msg->denm.alacarte->roadWorks->lightBarSirenInUse = vanetza::asn1::allocate<LightBarSirenInUse_t>();
    msg->denm.alacarte->roadWorks->lightBarSirenInUse->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	msg->denm.alacarte->roadWorks->lightBarSirenInUse->size = 1;
	msg->denm.alacarte->roadWorks->lightBarSirenInUse->buf[0] |= 1 << (7 - LightBarSirenInUse_sirenActivated);

    msg->denm.alacarte->roadWorks->closedLanes = vanetza::asn1::allocate<ClosedLanes_t>();
    msg->denm.alacarte->roadWorks->closedLanes->outerhardShoulderStatus = vanetza::asn1::allocate<HardShoulderStatus_t>();
    *msg->denm.alacarte->roadWorks->closedLanes->outerhardShoulderStatus = HardShoulderStatus_closed;
    
    msg->denm.alacarte->roadWorks->speedLimit = vanetza::asn1::allocate<SpeedLimit_t>();
    *msg->denm.alacarte->roadWorks->speedLimit = SPEED_LIMIT * SpeedLimit_oneKmPerHour;
    msg->denm.alacarte->roadWorks->incidentIndication =  vanetza::asn1::allocate<CauseCode_t>();
    msg->denm.alacarte->roadWorks->incidentIndication->causeCode = CauseCodeType_roadworks;
    msg->denm.alacarte->roadWorks->incidentIndication->subCauseCode = 0;
    msg->denm.alacarte->roadWorks->trafficFlowRule =  vanetza::asn1::allocate<TrafficRule_t>();
    *msg->denm.alacarte->roadWorks->trafficFlowRule = TrafficRule_noPassing;
    //std::cout<<"RWW DENM sent\n";

    
    if(m_pSHMSegmentRx->validData == true)
    {
    	memcpy(&m_EVWDENMRx, m_pSHMSegmentRx->buf, sizeof(EVWDENM_t));
    	//std::cout << "RWW valid data\n";
    	//std::cout << m_EVWDENMRx.header<<" "<<m_EVWDENMRx.payloadSize<<" \n";

        //msg->header.messageID = m_EVWDENMRx.EVWTxData.stationID;
        msg->denm.management.actionID.sequenceNumber = m_EVWDENMRx.packetCounter;
        msg->denm.management.eventPosition.longitude = m_EVWDENMRx.EVWTxData.latitude * Latitude_oneMicrodegreeNorth;
        msg->denm.management.eventPosition.longitude = m_EVWDENMRx.EVWTxData.longitude * Longitude_oneMicrodegreeEast;
        //m_EVWDENMTx.EVWTxData.speed = 4;
        //m_EVWDENMTx.EVWTxData.heading = msg->denm.location->eventPositionHeading->headingValue;
        msg->denm.situation->eventType.causeCode = m_EVWDENMRx.EVWTxData.causeCode;
        msg->denm.situation->eventType.subCauseCode = m_EVWDENMRx.EVWTxData.subCauseCode;
        m_pSHMSegmentRx->validData = false;

    }
    

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
    destination.position.latitude  = RWW_LATITUDE * vanetza::units::degree;//mVdp->latitude();
    destination.position.longitude = RWW_LONGITUDE * vanetza::units::degree;//mVdp->longitude();
    request.gn.destination = destination;
    //auto head = mVdp->heading().value() * 5729.58;
    //std::cout<<"EVW heading "<<head<<" Station ID "<<mVdp->getStationId()<<std::endl;
    //std::cout<<"lat "<<mVdp->latitude().value()<<" lon "<<mVdp->longitude().value()<<std::endl;

    return request;
}

void RoadWorkWarning::copyOVData()
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

void RoadWorkWarning::indicate(const artery::DenmObject& denm)
{
    //std::cout<<"denm\n";
    if (denm & CauseCode::Roadworks) {
        //std::cout<<"RWW denm!!!!\n";
        const vanetza::asn1::Denm& asn1 = denm.asn1();
        copyOVData();
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
    
        //rwHeading = asn1->denm.location->eventPositionHeading->headingValue / 10;
        distance = utils->distance(latitude/anglePrecision, longitude/anglePrecision,
        latit_1/anglePrecision, longi_1/anglePrecision);
        /*
        std::cout<<"RWW heading "<<rwHeading<< " host heading "<<hvHeading
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
        <<std::endl;
        */
        //LineSegHd *= RAD_TO_DEG;
        //std::cout<<"LineSegHd "<<LineSegHd;
        //std::cout<<" hvHeading "<<hvHeading;
        /*std::cout<<"head diff "<<static_cast<uint16_t>(abs(evwHeading- hvHeading))
        <<" bear diff "<<static_cast<uint16_t>(abs(hvHeading - LineSegHd))
        <<" dist " <<distance<<std::endl;*/
        if(static_cast<uint16_t>(distance) < RWW_LEVEL_0 && static_cast<int16_t>(cordAngle) > 0 )//&& (distance > prevDistance))
        {
            std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_0 !!!!!!!!!!!!!!!!!!!!00 "<<std::endl;
            mRWWFlag = true;
        }
        //if(static_cast<uint16_t>(abs(evwHeading- hvHeading)) < HEADING_COMPENSATION)
        //{
        if(static_cast<uint16_t>(abs(hvHeading - cordAngle)) < BEARING_COMPENSATION  &&
            static_cast<int16_t>(cordAngle) > 0)
        {
            //i32Ret = 1;
            if (!mRWWFlag)
            {
                if(static_cast<uint16_t>(distance) < RWW_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_1 111111111"<<std::endl;
                    mRWWFlag = true;
                }
                else //if(static_cast<uint16_t>(distance) < EEBL_LEVEL_1)
                {
                    std::cout<<"!!!!!!!!!!!!RoadWorkWarning::Level_2 11111111111"<<std::endl;
                    mRWWFlag = true;
                }
            }


        }
        mRWWFlag = false;
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
    if (denm & CauseCode::DangerousSituation) {
            const vanetza::asn1::Denm& asn1 = denm.asn1();
            //std::cout<<"RoadWorkWarning::DangerousSituation indicate_2"<<std::endl;
        }
    
}
void RoadWorkWarning::handleStoryboardTrigger(const StoryboardSignal& signal)
{
    if (signal.getCause() == "RWW") {
        mRWWVehicle = true;
        std::cout<<"RWW set \n";
    }
}
} // namespace den
} // namespace artery
