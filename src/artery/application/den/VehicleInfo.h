/*
 * VehicleInfo.h
 *
 *  Created on: 06-Aug-2021
 *      Author: v2x
 */

#ifndef V2XAPPS_INCLUDES_VEHICLEINFO_H_
#define V2XAPPS_INCLUDES_VEHICLEINFO_H_


struct VehicleInfo_t
{
	uint32_t stationId;
	uint16_t stationType;
	int64_t latitude;
	int64_t longitude;
	uint16_t speed;
	int16_t heading; // from north, clockwise
	float yawRate;
	float acceleration;

}__attribute__((packed));


#endif /* V2XAPPS_INCLUDES_VEHICLEINFO_H_ */
