/*
 * EVWData.h
 *
 *  Created on: 02-Aug-2021
 *      Author: amith
 */

#ifndef V2XAPPS_INCLUDES_EVWDATA_H_
#define V2XAPPS_INCLUDES_EVWDATA_H_

struct EVWTxData_t{

	uint64_t stationID;
	int64_t latitude;
	int64_t longitude;
	uint16_t speed;
	uint16_t heading;
	uint32_t causeCode;
	uint32_t subCauseCode;
}__attribute__((packed));

struct EVWDENM_t
{
	uint16_t header;
	uint16_t payloadSize;
	uint64_t packetCounter;
	EVWTxData_t EVWTxData;

}__attribute__((packed));

#endif /* V2XAPPS_INCLUDES_EVWDATA_H_ */
