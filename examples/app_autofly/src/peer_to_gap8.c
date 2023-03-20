//
// Created by Raven on 2023/3/14.
//
// #include "pmsis.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"

#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"

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
    //coordinate_t start_point = {TREE_CENTER_X, TREE_CENTER_Y, TREE_CENTER_Z};
    coordinateF_t item_start = {TREE_CENTER_X, TREE_CENTER_Y, TREE_CENTER_Z};
    coordinateF_t item_end = {0, 0, 0};
    coordinate_t end_point;
    while (1)
    {
        vTaskDelay(2000);
        item_start.x = 100 * logGetFloat(logGetVarId("stateEstimate", "y"));
        item_start.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y"));
        item_start.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
        DEBUG_PRINT("[appMain]start point:(%f, %f, %f)\n",(double)item_start.x,(double)item_start.y,(double)item_start.z);

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

            //octoTreeInsertPoint(octoTree,octoMap,start_point,LOG_ODDS_FREE_FLAG);
            DEBUG_PRINT("[appMain]front measurement: %f\n", (double)measurement.front);
            DEBUG_PRINT("[appMain]front end_point:(%d,%d,%d)\n\n", end_point.x, end_point.y, end_point.z);
            octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
            //  DEBUG_PRINT("[appMain]front: (%f, %f, %f)\n\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
        }

        continue;

        if (cal_Point(&measurement, item_start, rangeBack, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
            DEBUG_PRINT("[appMain]back: (%f, %f, %f)\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
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
            // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
            DEBUG_PRINT("[appMain]left: (%.2f, %.2f, %.2f)\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
        }

        if (cal_Point(&measurement, item_start, rangeRight, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
            DEBUG_PRINT("[appMain]right: (%.2f, %.2f, %.2f)\n\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);
        }
    }
}