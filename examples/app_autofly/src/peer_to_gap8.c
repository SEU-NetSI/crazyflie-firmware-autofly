//
// Created by Raven on 2023/3/14.
//
//#include "pmsis.h"

#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"
#include "param.h"

#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"
#include "rrtConnect.h"

static bool octotree_Flying = false;
/*
make cload:
    CLOAD_CMDS="-w radio://0/43/2M/E7E7E7E72B" make cload
*/

void sendMsgToGAP8()
{
    return;
}

void appMain()
{
    DEBUG_PRINT("appMain start\n");
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    // coordinate_t start_point = {TREE_CENTER_X, TREE_CENTER_Y, 0};
    coordinateF_t item_start = {TREE_CENTER_X, TREE_CENTER_Y, 0};
    coordinateF_t item_end = {0, 0, 0};
    coordinate_t end_point;
    // coordinate_t target_point = {200,200,200};
    int seqnumber = 0;
    // bool flag = true;

    while (1)
    {
        vTaskDelay(200);
        if(octotree_Flying){
            ++seqnumber;
            // test rrtConnect
            // if(flag){
            //     array_t path;
            //     path.len = 0;
            //     DEBUG_PRINT("[app]starting planning\n");
            //     planning(&start_point,&target_point,octoMap.octoTree,&octoMap,&path);
            //     DEBUG_PRINT("[app]path:");
            //     for(int i=0;i<path.len;i++){
            //         vTaskDelay(100);
            //         DEBUG_PRINT("(%d,%d,%d)\n",path.arr[i].loc.x,path.arr[i].loc.y,path.arr[i].loc.z);
            //     }
            //     flag = false;
            // }
            item_start.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + TREE_CENTER_X;
            item_start.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + TREE_CENTER_Y;
            item_start.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
            DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_start.x, (double)item_start.y, (double)item_start.z, seqnumber);

            // continue;

            //float -> int
            // start_point.x = item_start.x;
            // start_point.y = item_start.y;
            // start_point.z = item_start.z;

            get_measurement(&measurement);
            // DEBUG_PRINT("[appMain]Ranger Front: %f, Back: %f, Up: %f, Left: %f, Right: %f\n", (double)measurement.front, (double)measurement.back, (double)measurement.up, (double)measurement.left, (double)measurement.right);
            // DEBUG_PRINT("[appMain]Pitch: %f, Roll: %f, Yaw: %f\n", (double)measurement.pitch, (double)measurement.roll, (double)measurement.yaw);

            if (cal_Point(&measurement, item_start, rangeFront, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;

                    // octoTreeInsertPoint(octoTree,octoMap,start_point,LOG_ODDS_FREE_FLAG);
                    DEBUG_PRINT("[app]M_F:%.2f,seq:%d\n", (double)measurement.front, seqnumber);
                    DEBUG_PRINT("[app]EP_F:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                    // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                    octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                    //  DEBUG_PRINT("[appMain]front: (%f, %f, %f)\n\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
                }

                // continue;

            if (cal_Point(&measurement, item_start, rangeBack, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                DEBUG_PRINT("[app]M_B: %.2f,seq:%d\n", (double)measurement.back, seqnumber);
                DEBUG_PRINT("[app]EP_B:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                //DEBUG_PRINT("[appMain]back: (%f, %f, %f)\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
            }

            /*
            if (cal_Point(&measurement, item_start, rangeUp, &item_end)){
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                DEBUG_PRINT("up: x: %f, y: %f, z: %f\n\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
            }*/

            if (cal_Point(&measurement, item_start, rangeLeft, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_L: %.2f,seq:%d\n", (double)measurement.left, seqnumber);
                DEBUG_PRINT("[app]EP_L:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
            }

            if (cal_Point(&measurement, item_start, rangeRight, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_R: %.2f,seq:%d\n", (double)measurement.right, seqnumber);
                DEBUG_PRINT("[app]EP_R:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
            }
            
            /*if (cal_Point(&measurement, item_start, rangeRight, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_R: %.2f,seq:%d\n", (double)measurement.right, seqnumber);
                DEBUG_PRINT("[app]EP_R:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
            }*/
        }
    }
}

// Test Communicate
// void appMain()
// {
//     //For receiving
// //    DEBUG_PRINT("appMain start\n");
// //    AdP2PListeningInit();
// //    while(1){
// //        vTaskDelay(M2T(2000));
// //    }
//     //wait for cpx init
//     vTaskDelay(M2T(20000));
//     //create test coords
//     coordinate_t coords[5];
//     DEBUG_PRINT("Coords created\n");
//     for(int i=0;i<5;i++){
//         coords[i].x=i;
//         coords[i].y=i+1;
//         coords[i].z=i+2;
//     }
//     /* For Test
//     //cpx send
//     cpxInternalRouterInit();
//     while(1)
//     {
//         CPXPacket_t cpxPacket;
//         cpxPacket.route.source=1;
//         cpxPacket.route.destination=4;
//         cpxPacket.route.function=5;
//         cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
//         memcpy(&cpxPacket, coords, cpxPacket.dataLength);
//         bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
//         DEBUG_PRINT("Send %s\n",flag==false?"failed":"success");
//         //if(flag==false) break;
//         vTaskDelay(M2T(10000));
//     }
//      */

//     //For sending
// //    while(1){
// //        bool flag=SendCoords(coords);
// //        DEBUG_PRINT("sent %s\n",flag==false?"Failed":"Success");
// //        vTaskDelay(M2T(2000));
// //    }
// //}
PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_GROUP_STOP(octotree)