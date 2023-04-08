#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"
#include "config_autofly.h"
typedef enum {
    ExploreReq = 1,
    PathReq = 2,
    MappingReq = 5
} ReqType;

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

void ListeningInit();
bool SendReq(coordinate_t* coords, ReqType mode, uint16_t seq);
bool sendMappingRequest(coordinate_pair_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq);
bool sendExploreRequest(coordinate_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq);
bool sendPathRequest(coordinate_pair_t* pathRequestPayloadPtr, uint16_t pathRequestSeq);
#endif
