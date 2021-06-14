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

#include "ExampleDenService.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "artery/networking/PositionProvider.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/asn1/denm.hpp>

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");

static const simsignal_t denmReceivedSignal = cComponent::registerSignal("DenmReceived");
static const simsignal_t denmSentSignal = cComponent::registerSignal("DenmSent");

Define_Module(ExampleDenService)

ExampleDenService::ExampleDenService()
{
}

ExampleDenService::~ExampleDenService()
{
	cancelAndDelete(m_self_msg);
}

void ExampleDenService::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
	Enter_Method("indicate");

	if (packet->getByteLength() == 42) {
		EV_INFO << "packet indication on channel " << net.channel << "\n";
	}

	delete(packet);
}

void ExampleDenService::initialize()
{
	ItsG5Service::initialize();
	m_self_msg = new cMessage("Example Service");
	subscribe(denmReceivedSignal);
	subscribe(denmSentSignal);
	//subscribe(scSignalCamSent);
	//subscribe(scSignalCamReceived);

	scheduleAt(simTime() + 3.0, m_self_msg);
}

void ExampleDenService::finish()
{
	// you could record some scalars at this point
	ItsG5Service::finish();
}

void ExampleDenService::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "self message\n";
	}
}

void ExampleDenService::trigger()
{
			ActionID_t id;
	Enter_Method("trigger");
	//checkTriggeringConditions(simTime());
	/*
	if(1)//cam
	{
		// use an ITS-AID reserved for testing purposes
	static const vanetza::ItsAid example_its_aid = 16480;

	auto& mco = getFacilities().get_const<MultiChannelPolicy>();
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();

	for (auto channel : mco.allChannels(example_its_aid)) {
		auto network = networks.select(channel);
		if (network) {
			btp::DataRequestB req;
			// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(channel));
			req.gn.transport_type = geonet::TransportType::SHB;
			req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(42);

			// send packet on specific network interface
			request(req, packet, network.get());
			auto& vehicle = getFacilities().get_const<traci::VehicleController>();
			//std::cout<<" cam sent "<<vehicle.getVehicleId()<<std::endl;
		}
	}
	}
	*/
	if(1) //denm
	{
	// use an ITS-AID reserved for testing purposes
	static const vanetza::ItsAid example_its_aid = 16480;

	auto& mco = getFacilities().get_const<MultiChannelPolicy>();
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();

	for (auto channel : mco.allChannels(example_its_aid)) {
		auto network = networks.select(channel);
		if (network) {
			btp::DataRequestB req;
			// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(channel));
			//req.gn.transport_type = geonet::TransportType::SHB;
			req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			//request.destination_port = btp::ports::DENM;
			//request.gn.its_aid = aid::DEN;
    		req.gn.transport_type = geonet::TransportType::GBC;
    		//request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			geonet::Area destination;
			geonet::Circle destination_shape;
			using vanetza::units::si::meter;
			const PositionProvider* mPositionProvider = &getFacilities().get_const<PositionProvider>();
			destination.position.latitude = mPositionProvider->getGeodeticPosition().latitude;
        	destination.position.longitude = mPositionProvider->getGeodeticPosition().longitude;
			destination_shape.r = 1000.0 * meter;
			destination.shape = destination_shape;
			//destination.position.latitude = 0.0;
			//destination.position.longitude = 0.0;
			req.gn.destination = destination;


			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(42);

			// send packet on specific network interface
			request(req, packet, network.get());
			auto& vehicle = getFacilities().get_const<traci::VehicleController>();
			std::cout<<" denm sent "<<vehicle.getVehicleId()<<"  "<<id.originatingStationID<<std::endl;
		} else {
			EV_ERROR << "No network interface available for channel " << channel << "\n";
		}
	}
	}
}

void ExampleDenService::receiveSignal(cComponent* source, simsignal_t signal, cObject* cObject_1, cObject*)
//void ExampleDenService::receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* cObject_1, cObject* details) override
    
{
			ActionID_t id;
	Enter_Method("receiveSignal");
	//td::cout<<"  received sig \n";

	if (signal == denmReceivedSignal) {
		
		auto& vehicle = getFacilities().get_const<traci::VehicleController>();
		EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a DENM in sibling serivce\n";
		using namespace vanetza::asn1;
		//auto& vehicle = getFacilities().get_const<traci::VehicleController>();
		//EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
		std::cout<<" cam received "<<vehicle.getVehicleId()<<std::endl;

		id.originatingStationID = getFacilities().get_const<VehicleDataProvider>().station_id();
		//const auto cam = dynamic_cast<cObject*>(cObject_1);
		
		//const auto genDeltaTime = cam->asn1()->cam.generationDeltaTime;
		//const auto stdID = cam->asn1()->header.stationID
		std::cout<<" DENM received "<<vehicle.getVehicleId()<<"  "<<id.originatingStationID<<std::endl;
		//fire(this, t, genDeltaTime, details);
		//camParameters.basicContainer.stationType
	}
	/*
	//else if (signal == scSignalCamReceived) 
	if (signal == scSignalCamReceived) {
		
		auto& vehicle = getFacilities().get_const<traci::VehicleController>();
		EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
		//std::cout<<" cam received "<<vehicle.getVehicleId()<<std::endl;
	}*/
}

} // namespace artery
