// Copyright (C) 2020 SoarRobotics
// @author Amith
#include "V2XUtils.h"
#include <cmath>
#include <iostream>




//namespace sr
//{

const long double RADIUS = 6371000;// Radius of Earth in Meter, R = 6371000, R = 3956 for miles
const double anglePrecision = 10000000.0;
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
    std::cout<< "ddlat "<<lat1<<std::endl;
    std::cout<< "lon "<<lon1<<std::endl;
    std::cout<< "ddlat "<<lat2<<std::endl;
    std::cout<< "lon "<<lon2<<std::endl;
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
void V2XUtils::latLongtoXY(double lat, double lon, float & X, float &Y)
{
    X = RADIUS / anglePrecision * lon* cos((lat+lon)/2);
    Y = RADIUS / anglePrecision * lat;
}

void V2XUtils::projLatLongtoXY(stGeoMerc_t & stGeoMerc, float f32Lat, float f32Long, \
		float & pf32X, float & pf32Y)
{
    //stProjHdl_t* pstProjHdl = (stProjHdl_t*) pvProjHdl;
	double d64X,d64Y;
	int ret = 0;

	d64X = f32Long*DEG_TO_RAD;
	d64Y = f32Lat*DEG_TO_RAD;
	//ret = pj_transform(stGeoMerc.stPJlatlong,stGeoMerc.stPJmerc, 1, 1, &d64X, &d64Y, NULL );

	pf32X = (float)d64X;
	pf32Y = (float)d64Y;
#if 0
	printf("Conv Code: %s\n",pj_strerrno(ret));
	printf("Conv:%f,%f,%f,%f\n",f32Lat,f32Long,*pf32X,*pf32Y);
#endif
	//return ret;
}
void V2XUtils::init(stGeoMerc_t & stGeoMerc)
{
    stGeoMerc.stPJlatlong = NULL;

	//if (!(stGeoMerc.stPJmerc = pj_init_plus("+proj=utm +zone=43 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")) )
	{
		fprintf(stderr,"Projection library initialization failed\n");
		exit(0);
	}

	//if (!(stGeoMerc.stPJlatlong = pj_init_plus("+proj=latlong +ellps=WGS84")) )
	{
		fprintf(stderr,"Projection library initialization failed\n");
		exit(0);
	}
}
void V2XUtils::get_line_heading_length(float f32X1, float f32Y1, \
		float f32X2, float f32Y2, float & pf32Heading, float & pf32Length)
{
	float f32NrTemp, f32DrTemp;
	float f32Slope,f32Theta;

	f32NrTemp = (f32Y2-f32Y1);
	f32DrTemp = (f32X2-f32X1);
	if(f32DrTemp > -ZERO_DELTA && f32DrTemp < ZERO_DELTA)//vertical lane
	{
		// eqn of lane: x = x1
		// intersection point (x1,yp)
		// lane heading 0/180 degree, if y1 > y2, then 180 : if y2 > y1, then 0
		pf32Length = fabs(f32NrTemp);
		if(f32Y2 > f32Y1)
		{
			pf32Heading = 0;
		}
		else
		{
			pf32Heading = PI;
		}
	}
	else if(f32NrTemp > -ZERO_DELTA && f32NrTemp < ZERO_DELTA)//horizontal lane
	{
		// eqn of lane: y = y1
		// intersection point (xp,y1)
		// lane heading 90/270 degree, if x1 > x2, then 270 : if x2 > x1, then 90
		pf32Length = fabs(f32DrTemp);
		if(f32X2 > f32X1)
		{
			pf32Heading = 90*(PI/180);
		}
		else
		{
			pf32Heading = 270*(PI/180);
		}
	}
	else//general case
	{
		// theta = invtan(slope)
		// lane heading first quandrant = 90 - theta : second quandrant = 360 - (theta - 90) : third quandrant = 360 - (theta-90) : fourth quandrant = 360 - (theta-90)
		f32Slope = f32NrTemp/f32DrTemp;
		//f32SlopeAbs=fabs(f32Slope);
		f32Theta = fabs(atan(f32Slope));
		if((f32NrTemp<0) && (f32DrTemp>0))//4th quadrant
		{
			pf32Heading=( 90*PI_BY_180 + f32Theta);
		}
		else if((f32NrTemp<0) && (f32DrTemp<0))//3rd quadrant
		{
			pf32Heading=( 180*PI_BY_180 - f32Theta) + 90*PI_BY_180;
		}
		else if((f32NrTemp>0) && (f32DrTemp<0))//2nd quadrant
		{
			pf32Heading=( 270*PI_BY_180 + f32Theta); // (360-(180-theta-90))
		}
		else//1st quadrant
		{
			pf32Heading = ( 90*PI_BY_180 - f32Theta);
		}
		pf32Length = sqrt(pow((f32X1-f32X2),2) + pow((f32Y1-f32Y2),2));	// Length of the line segment
	}

}

//} //namespace sr