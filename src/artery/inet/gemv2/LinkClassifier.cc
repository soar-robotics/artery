/*
 * Artery V2X Simulation Framework
 * Copyright 2017 Thiago Vieira, Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/inet/gemv2/LinkClassifier.h"
#include "artery/inet/gemv2/ObstacleIndex.h"
#include "artery/inet/gemv2/VehicleIndex.h"
#include <inet/common/ModuleAccess.h>

namespace artery
{
namespace gemv2
{

Define_Module(LinkClassifier)

void LinkClassifier::initialize()
{
    mObstacleIndex = inet::findModuleFromPar<ObstacleIndex>(par("obstacleIndexModule"), this);
    mVehicleIndex = inet::findModuleFromPar<VehicleIndex>(par("vehicleIndexModule"), this);
}

LinkClassifier::Link LinkClassifier::classifyLink(const Position& tx, const Position& rx) const
{
    Link link = Link::LOS;
    if (mObstacleIndex->anyBlockage(tx, rx)) {
        link = Link::NLOSb;
    } else if (mVehicleIndex->anyBlockage(tx, rx)) {
        link = Link::NLOSv;
    }
    return link;
}

} // namespace gemv2
} // namespace artery