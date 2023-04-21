#include <string.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "debug.h"
#include "app.h"
#include "param.h"
#include "range.h"
#include "log.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"
#include "config_autofly.h"

#define DEBUG_PRINT_ENABLED 1
#define RxQueueSize 10

// handle mapping request
mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
uint8_t mappingRequestPayloadCur = 0;
uint16_t mappingRequestSeq = 0;

bool isSameNode(coordinate_t* coordinate1, coordinate_t* coordinate2)
{
    return (coordinate1->x / TREE_RESOLUTION == coordinate2->x / TREE_RESOLUTION) && 
        (coordinate1->y / TREE_RESOLUTION == coordinate2->y / TREE_RESOLUTION) && 
        (coordinate1->z / TREE_RESOLUTION == coordinate2->z / TREE_RESOLUTION);
}

void appendMappingRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint, uint8_t payloadLengthAdaptive)
{
    // merge coordinatePair
    for (int i = 0; i < mappingRequestPayloadCur; i++)
    {
        if (isSameNode(startPoint, &mappingRequestPayload[i].startPoint) && 
            isSameNode(endPoint, &mappingRequestPayload[i].endPoint))
        {
            mappingRequestPayload[i].mergedNums++;
            if (mappingRequestPayload[i].mergedNums < (LOG_ODDS_OCCUPIED - LOG_ODDS_FREE) / LOG_ODDS_DIFF_STEP) {
                return;
            }
        }
    }

    // append coordinatePair to payload
    mappingRequestPayload[mappingRequestPayloadCur].startPoint = *startPoint;
    mappingRequestPayload[mappingRequestPayloadCur].endPoint = *endPoint;
    mappingRequestPayload[mappingRequestPayloadCur].mergedNums = 1;
    mappingRequestPayloadCur++;

    if (mappingRequestPayloadCur >= MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT || mappingRequestPayloadCur >= payloadLengthAdaptive)
    {
        // package explore request
        mappingRequestSeq++;
        bool flag = sendMappingRequest(mappingRequestPayload, mappingRequestPayloadCur, mappingRequestSeq);
        mappingRequestPayloadCur = 0;

        // print debug info
        DEBUG_PRINT("[LiDAR-STM32]P2P: Send mapping request %s, seq: %d, payloadLength: %d\n", 
            flag == false ? "Failed" : "Successfully", mappingRequestSeq, mappingRequestPayloadCur);
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[LiDAR-STM32]P2P: Mapping request payload: \n");
            for (int i = 0; i < mappingRequestPayloadCur; i++)
            {
                DEBUG_PRINT("[LiDAR-STM32]P2P: coordinatePair %d: (%d, %d, %d), (%d, %d, %d), mergedNums: %d\n", 
                    i, 
                    mappingRequestPayload[i].startPoint.x, 
                    mappingRequestPayload[i].startPoint.y, 
                    mappingRequestPayload[i].startPoint.z,
                    mappingRequestPayload[i].endPoint.x, 
                    mappingRequestPayload[i].endPoint.y, 
                    mappingRequestPayload[i].endPoint.z,
                    mappingRequestPayload[i].mergedNums);
                vTaskDelay(50);
            }
            DEBUG_PRINT("\n");
        }
    }
}

// handle explore request
uint16_t exploreRequestSeq = 0;
void setExploreRequestPayload(coordinate_t* startPoint, example_measure_t* measurement)
{
    // package explore request
    exploreRequestSeq++;
    explore_req_payload_t exploreRequestPayload = {*startPoint, *measurement};
    bool flag = sendExploreRequest(&exploreRequestPayload, exploreRequestSeq);

    // print debug info
    DEBUG_PRINT("[LiDAR-STM32]P2P: Send explore request %s, seq: %d\n", 
        flag == false ? "Failed" : "Successfully", exploreRequestSeq);
    if (DEBUG_PRINT_ENABLED)
    {
        DEBUG_PRINT("[Edge-STM32]P2P: Explore request payload: \n");
        DEBUG_PRINT("[Edge-STM32]P2P: startPoint: (%d, %d, %d)\n", 
            exploreRequestPayload.startPoint.x, 
            exploreRequestPayload.startPoint.y, 
            exploreRequestPayload.startPoint.z);
        DEBUG_PRINT("[Edge-STM32]P2P: data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
            (double)exploreRequestPayload.measurement.data[0], 
            (double)exploreRequestPayload.measurement.data[1], 
            (double)exploreRequestPayload.measurement.data[2], 
            (double)exploreRequestPayload.measurement.data[3], 
            (double)exploreRequestPayload.measurement.data[4], 
            (double)exploreRequestPayload.measurement.data[5]);
        DEBUG_PRINT("[Edge-STM32]P2P: roll: %.2f, pitch: %.2f, yaw: %.2f\n\n", 
            (double)exploreRequestPayload.measurement.roll, 
            (double)exploreRequestPayload.measurement.pitch, 
            (double)exploreRequestPayload.measurement.yaw);
        vTaskDelay(50);
    }
}

void appMain()
{
    vTaskDelay(M2T(10000));
    example_measure_t measurement;
    coordinate_t startPoint = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinateF_t startPointF = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinate_t endPoint;
    coordinateF_t endPointF;

    uint8_t payloadLengthAdaptive = MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC;
    while (1) {
        vTaskDelay(M2T(100));
        // 如果收到了探索响应报文
        if (true) {
            // 根据响应结果，运动（从 xQueue 读取）
            // crtpCommanderHighLevelGoTo();
            // 将建图数量改为1
            payloadLengthAdaptive = MAPPING_REQUEST_PAYLOAD_LENGTH_MOVING;
            vTaskDelay(M2T(80));

            // 将建图数量改为4
            payloadLengthAdaptive = MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC;
            // 发送下一个探索请求报文

        } else {
            // 如果如果超时了
            if (true) {
                continue;
                // 重新发送探索请求报文
                // 更新关注的seq
            }
            // 发送建图信息请求报文
        }
    }
    
    // circularly get measurement and send to edge-computing uav
    while (1) 
    {
        vTaskDelay(M2T(1000));
        // set start point
        startPointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
        startPointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
        startPointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
        startPoint.x = startPointF.x;
        startPoint.y = startPointF.y;
        startPoint.z = startPointF.z;
        
        // set measurement
        get_measurement(&measurement);
        if (startPointF.z < TOP) {
            measurement.data[4] = TOP - startPointF.z;
        } else {
            measurement.data[4] = 0;
        }
        if (startPointF.z > BOTTOM) {
            measurement.data[5] = startPointF.z - BOTTOM;
        } else {
            measurement.data[5] = 0;
        }
        
        // set end point
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            if (cal_Point(&measurement, &startPointF, dir, &endPointF))
            {
                endPoint.x = endPointF.x;
                endPoint.y = endPointF.y;
                endPoint.z = endPointF.z;

                // add (startPoint, endPoint) to mappingRequestPayload
                appendMappingRequestPayload(&startPoint, &endPoint, MAPPING_REQUEST_PAYLOAD_LENGTH_STATIC);
            }
        }
    }
}
