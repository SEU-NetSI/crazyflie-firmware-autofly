//
// Created by Raven on 2023/3/14.
//
//#include "pmsis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "auxiliary_tool.h"

void sendMsgToGAP8()
{
    return ;
}

void appMain()
{
    example_measure_t measurement;
    coordinate_t start_point = {0,0,0};
    coordinate_t end_point;
    while(1){
        vTaskDelay(2000);
        measurement = get_measurement();
        DEBUG_PRINT("Ranger Front: %f, Back: %f, Up: %f, Left: %f, Right: %f\n", (double)measurement.front, (double)measurement.back, (double)measurement.up, (double)measurement.left, (double)measurement.right);
        DEBUG_PRINT("Pitch: %f, Roll: %f, Yaw: %f\n", (double)measurement.pitch, (double)measurement.roll, (double)measurement.yaw);

        end_point = cal_Point(&measurement,start_point,rangeRight);
        DEBUG_PRINT("right: x: %f, y: %f, z: %f\n", (double)end_point.x, (double)end_point.y, (double)end_point.z);
        end_point = cal_Point(&measurement,start_point,rangeLeft);
        DEBUG_PRINT("left: x: %f, y: %f, z: %f\n", (double)end_point.x, (double)end_point.y, (double)end_point.z);
        end_point = cal_Point(&measurement,start_point,rangeFront);
        DEBUG_PRINT("front: x: %f, y: %f, z: %f\n", (double)end_point.x, (double)end_point.y, (double)end_point.z);
        end_point = cal_Point(&measurement,start_point,rangeBack);
        DEBUG_PRINT("back: x: %f, y: %f, z: %f\n", (double)end_point.x, (double)end_point.y, (double)end_point.z);
        end_point = cal_Point(&measurement,start_point,rangeUp);
        DEBUG_PRINT("up: x: %f, y: %f, z: %f\n", (double)end_point.x, (double)end_point.y, (double)end_point.z);
    }
}