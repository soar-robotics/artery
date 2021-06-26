// Copyright (C) 2020 SoarRobotics 
// @author Amith
#ifndef ARTERY_V2XUTILS_H_
#define ARTERY_V2XUTILS_H_
#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include "proj_api.h"
#include <boost/math/constants/constants.hpp>
#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG (1.0/DEG_TO_RAD)
#define PI_BY_180  (0.01745329252)
#define DELTA_ANGLE_DEV		0.1 //in degree
#define FLOAT_PREC			0.00001
#define PI				(3.1415926536)
#define ZERO_DELTA	0.00001
#define C_180_BY_PI	(57.2957795131)
#define ALLOWED_HEADING_DEV	(10)	//degree
//const double PI = boost::math::constants::pi<double>();
typedef struct _stGeoMerc_
{
	projPJ stPJmerc;
	projPJ stPJlatlong;
}stGeoMerc_t;


//namespace sr
//{

class V2XUtils
{
public:
    //V2XUtils();
    //~V2XUtils();
    double toRad(const double degree);
    double distance( double lat1,  double long1, double lat2,  double long2);
    void latLongtoXY(double lat, double lon, float& X, float &Y);
    void projLatLongtoXY(stGeoMerc_t & stGeoMerc, float f32Lat, float f32Long,float & pf32X, float & pf32Y);
    void init(stGeoMerc_t & stGeoMerc);
    void get_line_heading_length(float f32X1, float f32Y1, \
		float f32X2, float f32Y2, float & pf32Heading, float & pf32Length);
    void boostLatLongtoXY(double lat, double lon, float & X, float &Y);
    double degreeToRadian(double degree);// { return (degree * PI / 180); };
	  double radianToDegree(double radian);// { return (radian * 180 / PI); };

	  double CoordinatesToAngle(const double latitude1,const double longitude1,
                              const double latitude2,const double longitude2);
    //stGeoMerc_t stGeoMerc;
};



//} //namespace sr

#endif /* ARTERY_V2XUTILS_H_*/