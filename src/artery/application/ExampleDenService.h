//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef EXAMPLEDENSERVICE_H_
#define EXAMPLEDENSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"
#include <omnetpp/cobject.h>
#include <omnetpp.h>
#include <cassert>
#include <vanetza/asn1/cam.hpp>
#include <memory>
//#include <artery/application/CaObject.h>


namespace artery
{

class ExampleDenService : public ItsG5Service
{
    public:
        ExampleDenService();
        ~ExampleDenService();
        const vanetza::asn1::Cam& asn1() const;


        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, const NetworkInterface&) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
        //void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* cObject_1, cObject* details) override;
    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        omnetpp::cMessage* m_self_msg;
};

} // namespace artery

#endif /* EXAMPLEDENSERVICE_H_ */
