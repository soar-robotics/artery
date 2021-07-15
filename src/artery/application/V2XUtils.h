// Copyright (C) 2020 SoarRobotics 
// @author Amith
#ifndef ARTERY_V2XUTILS_H_
#define ARTERY_V2XUTILS_H_
#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <boost/math/constants/constants.hpp>

#define DELTA_ANGLE_DEV		0.1 //in degree

const double ZERO_DELTA	 = 0.00001;
#define C_180_BY_PI	(57.2957795131)
#define ALLOWED_HEADING_DEV	(10)	//degree
const double SR_PI = boost::math::constants::pi<double>();
const double SR_DEG_TO_RAD = (SR_PI/180.0);
const double SR_RAD_TO_DEG = (1.0/SR_DEG_TO_RAD);

//namespace sr
//{

class V2XUtils
{
public:
    double distance( double lat1,  double long1, double lat2,  double long2);
    void latLongtoXY(double lat, double lon, float& X, float &Y);
    void boostLatLongtoXY(double lat, double lon, float & X, float &Y);

	  double CoordinatesToAngle(const double latitude1,const double longitude1,
                              const double latitude2,const double longitude2);
};



//} //namespace sr

#endif /* ARTERY_V2XUTILS_H_*/