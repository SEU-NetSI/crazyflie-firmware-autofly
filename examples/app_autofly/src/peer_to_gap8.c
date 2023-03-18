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

#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"

void sendMsgToGAP8()
{
    return ;
}

void appMain()
{
    DEBUG_PRINT("appMain start\n");
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    coordinate_t start_point = {0,0,0};
    coordinateF_t item_start = {0,0,0};
    coordinateF_t item_end = {0,0,0};
    coordinate_t end_point;
    while(1){
        vTaskDelay(2000);
        measurement = get_measurement();
        DEBUG_PRINT("Ranger Front: %f, Back: %f, Up: %f, Left: %f, Right: %f\n", (double)measurement.front, (double)measurement.back, (double)measurement.up, (double)measurement.left, (double)measurement.right);
        DEBUG_PRINT("Pitch: %f, Roll: %f, Yaw: %f\n", (double)measurement.pitch, (double)measurement.roll, (double)measurement.yaw);

        item_end = cal_Point(&measurement,item_start,rangeRight);
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
        DEBUG_PRINT("right: x: %f, y: %f, z: %f\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);

        item_end = cal_Point(&measurement,item_start,rangeLeft);
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
        DEBUG_PRINT("left: x: %f, y: %f, z: %f\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);

        item_end = cal_Point(&measurement,item_start,rangeFront);
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
        DEBUG_PRINT("front: x: %f, y: %f, z: %f\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);

        item_end = cal_Point(&measurement,item_start,rangeBack);
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
        DEBUG_PRINT("back: x: %f, y: %f, z: %f\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);

        /*
        item_end = cal_Point(&measurement,item_start,rangeUp);
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
        DEBUG_PRINT("up: x: %f, y: %f, z: %f\n\n", (double)item_end.x, (double)item_end.y, (double)item_end.z);*/
    }
}