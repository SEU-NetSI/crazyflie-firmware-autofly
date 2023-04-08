#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"

typedef enum {
    MappingReq = 1,
    ExploreReq = 2,
    PathReq = 3,
    ExploreResp = 4,
    PathResp = 5,
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

void P2PListeningInit();
#endif
