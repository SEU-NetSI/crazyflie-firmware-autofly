#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#define DEBUG_MODULE "P2P"

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5

#define CRTP_RECEIVE_PAYLOAD_LENGTH 5

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
    float data[6];
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

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
    uint8_t destinationId;
    coordinate_t endPoint;
} explore_resp_payload_t;

void CPXForwardInit();
void P2PListeningInit();
#endif
