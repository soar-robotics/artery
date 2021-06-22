// Copyright (C) 2020 SoarRobotics 
// @author Amith
#ifndef ARTERY_V2XUTILS_H_
#define ARTERY_V2XUTILS_H_



//namespace sr
//{

class V2XUtils
{
public:
    //V2XUtils();
    //~V2XUtils();
    double toRad(const double degree);
    double distance( double lat1,  double long1, double lat2,  double long2);
};



//} //namespace sr

#endif /* ARTERY_V2XUTILS_H_*/