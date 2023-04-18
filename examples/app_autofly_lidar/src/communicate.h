#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
#include "config_autofly.h"
#include "auxiliary_tool.h"

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5
#define MOVE_DELAY 600

#define RESPONSE_PAYLOAD_LENGTH 5
#define DESTINATION_ID 0x00

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct
{
    coordinate_t startPoint;
    coordinate_t endPoint;
} coordinate_pair_t;

typedef struct
{
    coordinate_pair_t coordinatePair;
    uint8_t mergedNums;
} mapping_req_payload_t;

typedef struct
{
    coordinate_t startPoint;
    example_measure_t measurement;
} explore_req_payload_t;

typedef struct
{
    coordinate_t endPoint;
} explore_resp_payload_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    uint8_t mappingRequestPayloadLength;
    mapping_req_payload_t mappingRequestPayload[4];
} mapping_req_packet_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_resp_payload_t exploreResponsePayload;
} explore_resp_packet_t;

void ListeningInit();
bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq);
bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq);
#endif
