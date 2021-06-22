// Copyright (C) 2020 SoarRobotics
// @author Amith
#include "V2XUtils.h"
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <iostream>


//namespace sr
//{
const double PI = boost::math::constants::pi<double>();
const long double RADIUS = 6371000;// Radius of Earth in Meter, R = 6371000, R = 3956 for miles

// Utility function for converting degrees to radians
double V2XUtils::toRad(const double degree)
{
    return (degree * (PI) / 180);
}

double V2XUtils::distance(double lat1, double lon1,
                        double lat2, double lon2)
{
    // distance between latitudes and longitudes
    double dLat = toRad(abs(lat2 - lat1));
    double dLon = toRad(abs(lon2 - lon1));
    /*std::cout<< "ddlat "<<lat1<<std::endl;
    std::cout<< "lon "<<lon1<<std::endl;
    std::cout<< "ddlat "<<lat2<<std::endl;
    std::cout<< "lon "<<lon2<<std::endl;*/
    // convert to radians
    lat1 = toRad(lat1);
    lat2 = toRad(lat2);

    // apply formulae
    double a = pow(sin(dLat / 2), 2) +
                pow(sin(dLon / 2), 2) *
                cos(lat1) * cos(lat2);
    double c = 2 * asin(sqrt(a));
    return RADIUS * c;
}


//} //namespace sr