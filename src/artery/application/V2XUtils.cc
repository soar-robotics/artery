// Copyright (C) 2020 SoarRobotics
// @author Amith
#include "V2XUtils.h"
#include <cmath>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/core/coordinate_system.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/srs/epsg.hpp>
#include <boost/geometry/srs/projection.hpp>
//namespace sr
//{

const long double RADIUS = 6371000;// Radius of Earth in Meter, R = 6371000, R = 3956 for miles
const double anglePrecision = 10000000.0;
namespace bg  = boost::geometry;
namespace bm  = bg::model::d2;
namespace srs = bg::srs;

using LongLat = bm::point_xy<double, bg::cs::geographic<bg::degree>>;
using UTM     = bm::point_xy<double /*, srs::static_epsg<3043>*/>;

double V2XUtils::distance(double lat1, double lon1,
                        double lat2, double lon2)
{
    // distance between latitudes and longitudes
    double dLat = (abs(lat2 - lat1)) * SR_DEG_TO_RAD;
    double dLon = (abs(lon2 - lon1)) * SR_DEG_TO_RAD;
#if 0
    std::cout<< "ddlat0 "<< std::fixed << std::setprecision(6)<<lat1;
    std::cout<< " lon0 "<< std::fixed << std::setprecision(6)<<lon1<<std::endl;
    std::cout<< "ddlat1 "<< std::fixed << std::setprecision(6)<<lat2;
    std::cout<< " lon1 "<< std::fixed << std::setprecision(6)<<lon2<<std::endl;
#endif
    // convert to radians
    lat1 = (lat1) * SR_DEG_TO_RAD;
    lat2 = (lat2) * SR_DEG_TO_RAD;

    // apply formulae
    double a = pow(sin(dLat / 2), 2) +
                pow(sin(dLon / 2), 2) *
                cos(lat1) * cos(lat2);
    double c = 2 * asin(sqrt(a));
    return RADIUS * c;
}
void V2XUtils::latLongtoXY(double lat, double lon, float & X, float &Y)
{
    X = RADIUS / anglePrecision * lon* cos((lat+lon)/2);
    Y = RADIUS / anglePrecision * lat;
}

void V2XUtils::boostLatLongtoXY(double lat, double lon, float & x, float & y)
{
	UTM xy{};
	LongLat latlon{lon, lat};
	srs::projection<> zone32 = srs::proj4("+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");
	//report(Amsterdam, zone33);
	zone32.forward(latlon, xy);
	//std::cout << bg::get<0>(xy)<<bg::get<1>(xy)<<"\n";
	x = static_cast<float>(bg::get<0>(xy));
	y = static_cast<float>(bg::get<1>(xy));
}

	//double V2XUtils::degreeToRadian(double degree) { return (degree * PI / 180); };
	//double V2XUtils::radianToDegree(double radian) { return (radian * 180 / PI); };

	double V2XUtils::CoordinatesToAngle(const double latitude1,const double longitude1,
										const double latitude2,const double longitude2)
	{
#if 0
    std::cout<< "cclat0 "<<latitude1;
    std::cout<< " lon0 "<<longitude1<<std::endl;
    std::cout<< "cclat1 "<<latitude2;
    std::cout<< " lon2 "<<longitude2<<std::endl;
#endif
	const auto longitudeDifferenceRadians = (longitude2 - longitude1) * SR_DEG_TO_RAD;
	auto latitude1Radian = (latitude1) * SR_DEG_TO_RAD;
	auto latitude2Radian = (latitude2) * SR_DEG_TO_RAD;

	const auto x = std::cos(latitude1Radian) * std::sin(latitude2Radian) -
					std::sin(latitude1Radian) * std::cos(latitude2Radian) *
					std::cos(longitudeDifferenceRadians);
	const auto y = std::sin(longitudeDifferenceRadians) * std::cos(latitude2Radian);

	return (std::atan2(y, x) * SR_RAD_TO_DEG);
	}

//} //namespace sr