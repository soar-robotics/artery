/*
 * Artery V2X Simulation Framework
 * Copyright 2020 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_DISASTER_MANAGEMENT_H_QRTLCYIY
#define ARTERY_DISASTER_MANAGEMENT_H_QRTLCYIY

#include "artery/application/den/SuspendableUseCase.h"
#include "artery/application/Sampling.h"
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/velocity.hpp>
#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"

#include "artery/application/den/DMData.h"

#include<sys/ipc.h>
#include<sys/shm.h>
#include<sys/types.h>
#include "artery/application/den/EVWData.h"
#include "artery/application/den/VehicleInfo.h"

constexpr uint16_t BUF_SIZE = 4096;
struct SHMSegment_t {
   int cnt;
   int complete;
   bool validData;
   char buf[BUF_SIZE];
};

namespace artery
{
namespace den
{

class DisasterManagement : public SuspendableUseCase//, public ItsG5Service
{
public:
    void check() override;
    void indicate(const artery::DenmObject&) override;
    void handleStoryboardTrigger(const StoryboardSignal&) override;

protected:
    void initialize(int) override;

    vanetza::asn1::Denm createMessage();
    vanetza::btp::DataRequestB createRequest();

private:
    bool mDiasterManagementVehicle = false;

    int m_SHMIdTx;
    SHMSegment_t * m_pSHMSegmentTx;
    int m_SHMIdRx;
    SHMSegment_t * m_pSHMSegmentRx;
    DMDENM_t mDMDENMTx;
	DMDENM_t mDMDENMRx;
    //EVWDENM_t mDMDENMTx;
	//EVWDENM_t mDMDENMRx;
    int m_SHMIdCv;
    SHMSegment_t * m_pSHMSegmentCv;
    VehicleInfo_t mVehicleInfoCv;
    void copyEVWVehicleData();
    int m_SHMIdOv;
    SHMSegment_t * m_pSHMSegmentOv;
    VehicleInfo_t mVehicleInfoOv;
    void copyOVData();
};

} // namespace den
} // namespace artery

#endif /* ARTERY_DISASTER_MANAGEMENT_H_QRTLCYIY */
