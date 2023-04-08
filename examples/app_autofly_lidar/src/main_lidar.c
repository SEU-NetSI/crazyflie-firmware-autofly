#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "param.h"
#include "range.h"
#include "log.h"

#include <string.h>
#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "auxiliary_tool.h"
#include "config_autofly.h"
#include "communicate.h"

static bool lidarUavWorking = false;

// handle mapping request
coordinate_pair_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH];
uint8_t mappingRequestPayloadCur = 0;
uint16_t mappingRequestSeq = 0;
void appendMappingRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint)
{
    mappingRequestPayload[mappingRequestPayloadCur].startPoint = *startPoint;
    mappingRequestPayload[mappingRequestPayloadCur].endPoint = *endPoint;
    mappingRequestPayloadCur++;

    if (mappingRequestPayloadCur == MAPPING_REQUEST_PAYLOAD_LENGTH)
    {
        mappingRequestSeq++;
        bool flag = sendMappingRequest(mappingRequestPayload, MAPPING_REQUEST_PAYLOAD_LENGTH, mappingRequestSeq);
        mappingRequestPayloadCur = 0;
        DEBUG_PRINT("[app-autofly-lidar]Send mapping request %s, seq: %d, payloadLength: %d\n",flag == false ? "Failed" : "Successfully", mappingRequestSeq, MAPPING_REQUEST_PAYLOAD_LENGTH);
        DEBUG_PRINT("[app-autofly-lidar]First coordinate pair: (%d, %d, %d), (%d, %d, %d)\n", 
            mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
            mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);
    }
}

// handle explore request
uint16_t exploreRequestSeq = 0;
void setExploreRequestPayload(coordinate_t* startPoint)
{
    exploreRequestSeq++;
    bool flag = sendExploreRequest(startPoint, exploreRequestSeq);
    DEBUG_PRINT("[app-autofly-lidar]Send explore request %s, seq: %d\n",flag == false ? "Failed" : "Successfully", exploreRequestSeq);
    DEBUG_PRINT("[app-autofly-lidar]startPoint coordinate: (%d, %d, %d)\n", startPoint[0].x, startPoint[0].y, startPoint[0].z);
}

// handle path request
uint16_t pathRequestSeq = 0;
void setPathRequestPayload(coordinate_t* startPoint, coordinate_t* endPoint)
{
    pathRequestSeq++;
    coordinate_pair_t pathRequestPayload;
    pathRequestPayload.startPoint = *startPoint;
    pathRequestPayload.endPoint = *endPoint;
    bool flag = sendPathRequest(&pathRequestPayload, pathRequestSeq);
    DEBUG_PRINT("[app-autofly-lidar]Send path request %s, seq: %d\n",flag == false ? "Failed" : "Successfully", pathRequestSeq);
    DEBUG_PRINT("[app-autofly-lidar]startPoint: (%d, %d, %d), endPoint: (%d, %d, %d)\n", 
        startPoint[0].x, startPoint[0].y, startPoint[0].z, 
        endPoint[0].x, endPoint[0].y, endPoint[0].z);
}

void appMain()
{
    example_measure_t measurement;
    coordinate_t startPoint = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinateF_t startPointF = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinate_t endPoint;
    coordinateF_t endPointF;
    
    // circularly get measurement and send to edge-computing uav
    while (1) 
    {
        vTaskDelay(M2T(500));
        if (lidarUavWorking) {
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
                    appendMappingRequestPayload(&startPoint, &endPoint);
                }
            }
        }
    }
}

// void appMain()
// {
//     //create test coords
//     coordinate_t coords1[5];
//     for(int i=0;i<5;i++){
//         coords1[i].x=i;
//         coords1[i].y=i+1;
//         coords1[i].z=i+2;
//     }
//     coordinate_t coords2[2];
//     for(int i=0;i<2;i++){
//         coords2[i].x=i+1;
//         coords2[i].y=i+2;
//         coords2[i].z=i+3;
//     }
//     coordinate_t coord3;
//     coord3.x=3;
//     coord3.y=4;
//     coord3.z=5;
//     bool flag=0;
//     uint16_t seq=0;
//     while(1){
//         flag=SendReq(coords1,MappingReq,seq);
//         DEBUG_PRINT("sent MappingReq %s\n",flag==false?"Failed":"Success");
//         seq++;
//         vTaskDelay(M2T(1000));
//         flag=SendReq(coords2,PathReq,seq);
//         DEBUG_PRINT("sent PathReq %s\n",flag==false?"Failed":"Success");
//         seq++;
//         vTaskDelay(M2T(1000));
//         flag=SendReq(&coord3,ExploreReq,seq);
//         DEBUG_PRINT("sent ExploreReq %s\n",flag==false?"Failed":"Success");
//         seq++;
//         vTaskDelay(M2T(1000));
//     }
// }

PARAM_GROUP_START(autofly)
PARAM_ADD(PARAM_UINT8, lidarUavWorking, &lidarUavWorking)
PARAM_GROUP_STOP(autofly)
