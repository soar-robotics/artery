/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2020 Raphael Riebl et al.
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_RSUDENSERVICE_H_
#define ARTERY_RSUDENSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
//#include <vanetza/asn1/cam.hpp>
#include <omnetpp/simtime.h>
#include <boost/optional/optional.hpp>
#include <vanetza/asn1/denm.hpp>
#include "artery/application/den/Memory.h"
#include "artery/application/den/UseCase.h"
#include <list>

namespace artery
{

class GeoPosition;
class Identity;
class LocalDynamicMap;
class NetworkInterfaceTable;
class Timer;

class RsuDenService : public ItsG5BaseService
{
    public:
        void initialize() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;
/*
        struct ProtectedCommunicationZone
        {
            boost::optional<ProtectedZoneID_t> id;
            ProtectedZoneType_t type = ProtectedZoneType_permanentCenDsrcTolling;
            double latitude_deg = 0.0;
            double longitude_deg = 0.0;
            unsigned radius_m = 0;
        };
        */

        //static std::list<ProtectedCommunicationZone> parseProtectedCommunicationZones(omnetpp::cXMLElement*);
        ActionID_t requestActionID();
        void sendDENM(vanetza::asn1::Denm&&, vanetza::btp::DataRequestB&);
        void fillRequest(vanetza::btp::DataRequestB&);
        vanetza::btp::DataRequestB createRequest();
        using ItsG5BaseService::getFacilities;
        const Timer* getTimer() const;
        //std::shared_ptr<const artery::den::Memory> getMemory() const;
        //void fillRequest(vanetza::btp::DataRequestB& request);

    private:
        
        vanetza::asn1::Denm createMessage() const;

        ChannelNumber mPrimaryChannel = channel::CCH;
        const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
        const Timer* mTimer = nullptr;
        const Identity* mIdentity = nullptr;
        const GeoPosition* mGeoPosition = nullptr;
        LocalDynamicMap* mLocalDynamicMap = nullptr;
        omnetpp::SimTime mGenerationInterval;
        omnetpp::SimTime mLastDENMTimestamp;
        std::list<ProtectedCommunicationZone> mProtectedCommunicationZones;
        uint16_t mSequenceNumber;
        std::shared_ptr<artery::den::Memory> mMemory;
        std::list<artery::den::UseCase*> mUseCases;
        
};

} // namespace artery

#endif /* ARTERY_RSUDENSERVICE_H_ */
