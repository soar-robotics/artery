/*
 * DMData.h
 *
 *  Created on: 08-Aug-2021
 *      Author: v2x
 */

#ifndef V2XAPPS_FEATURES_DM_DMDATA_H_
#define V2XAPPS_FEATURES_DM_DMDATA_H_

const auto MESSAGE_LENGTH_3 = 3;
const auto MESSAGE_LENGTH_6 = 6;
const auto MESSAGE_LENGTH_24 = 24;

struct DMData_t{

	uint64_t stationID;
	int64_t latitude;
	int64_t longitude;
	//uint16_t speed;
	//uint16_t heading;
	uint32_t causeCode;
	uint32_t subCauseCode;
	uint8_t wMInumber[MESSAGE_LENGTH_3];
	uint8_t vDS[MESSAGE_LENGTH_3];
	uint8_t emergencyActionCode[MESSAGE_LENGTH_24];
	uint8_t companyName[MESSAGE_LENGTH_24];

}__attribute__((packed));

struct DMDENM_t
{
	uint16_t header;
	uint16_t payloadSize;
	uint64_t packetCounter;
	DMData_t DMData;

}__attribute__((packed));

#endif /* V2XAPPS_FEATURES_DM_DMDATA_H_ */
