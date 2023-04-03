// #include "pmsis.h"

#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"

#include "auxiliary_tool.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"
#include "rrtConnect.h"
#include "config_autofly.h"
#include "circularQueue.h"

#define DEBUG_DELAY 75

static bool octotree_Flying = false;
int seqnumber = 0;
/*
make cload:
    CLOAD_CMDS="-w radio://0/2M/86E7E7E7E7" make cload
*/

void UpdateMap(octoMap_t *octoMap, example_measure_t *measurement, coordinateF_t *current_F, coordinate_t *current_I)
{
    DEBUG_PRINT("[app]UpdateMap,seq:%d\n", seqnumber);
    vTaskDelay(200);
    coordinate_t end_point;
    coordinateF_t item_end;
    if (cal_Point(measurement, current_F, rangeFront, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_F:%.2f,seq:%d\n", (double)measurement->front, seqnumber);
        DEBUG_PRINT("[app]EP_F:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }

    // return;
    if (cal_Point(measurement, current_F, rangeBack, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_B: %.2f,seq:%d\n", (double)measurement->back, seqnumber);
        DEBUG_PRINT("[app]EP_B:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }

    if (cal_Point(measurement, current_F, rangeLeft, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_L: %.2f,seq:%d\n", (double)measurement->left, seqnumber);
        DEBUG_PRINT("[app]EP_L:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }

    if (cal_Point(measurement, current_F, rangeRight, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_R: %.2f,seq:%d\n", (double)measurement->right, seqnumber);
        DEBUG_PRINT("[app]EP_R:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }

    if (cal_Point(measurement, current_F, rangeUp, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_U: %.2f,seq:%d\n", (double)measurement->up, seqnumber);
        DEBUG_PRINT("[app]EP_U:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }

    if (cal_Point(measurement, current_F, rangeDown, &item_end))
    {
        end_point.x = item_end.x;
        end_point.y = item_end.y;
        end_point.z = item_end.z;
        DEBUG_PRINT("[app]M_D: %.2f,seq:%d\n", (double)measurement->down, seqnumber);
        DEBUG_PRINT("[app]EP_D:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
        vTaskDelay(300);
        octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
    }
}

void CalCandidates(coordinate_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    coordinateF_t item_point;
    if (measurement->front > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeFront, &item_point);
        candidates[0].x = (int)item_point.x;
        candidates[0].y = (int)item_point.y;
        candidates[0].z = (int)item_point.z;
    }
    else
    {
        candidates[0].x = 30000;
        candidates[0].y = 30000;
        candidates[0].z = 30000;
    }
    if (measurement->back > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeBack, &item_point);
        candidates[1].x = (int)item_point.x;
        candidates[1].y = (int)item_point.y;
        candidates[1].z = (int)item_point.z;
    }
    else
    {
        candidates[1].x = 30000;
        candidates[1].y = 30000;
        candidates[1].z = 30000;
    }
    if (measurement->left > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeLeft, &item_point);
        candidates[2].x = (int)item_point.x;
        candidates[2].y = (int)item_point.y;
        candidates[2].z = (int)item_point.z;
    }
    else
    {
        candidates[2].x = 30000;
        candidates[2].y = 30000;
        candidates[2].z = 30000;
    }
    if (measurement->right > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeRight, &item_point);
        candidates[3].x = (int)item_point.x;
        candidates[3].y = (int)item_point.y;
        candidates[3].z = (int)item_point.z;
    }
    else
    {
        candidates[3].x = 30000;
        candidates[3].y = 30000;
        candidates[3].z = 30000;
    }
    if (measurement->up > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeUp, &item_point);
        candidates[4].x = (int)item_point.x;
        candidates[4].y = (int)item_point.y;
        candidates[4].z = (int)item_point.z;
    }
    else
    {
        candidates[4].x = 30000;
        candidates[4].y = 30000;
        candidates[4].z = 30000;
    }
    if (measurement->down > TREE_RESOLUTION)
    {
        cal_PointByLength(TREE_RESOLUTION, pitch, roll, yaw, current_F, rangeDown, &item_point);
        candidates[5].x = (int)item_point.x;
        candidates[5].y = (int)item_point.y;
        candidates[5].z = (int)item_point.z;
    }
    else
    {
        candidates[5].x = 30000;
        candidates[5].y = 30000;
        candidates[5].z = 30000;
    }
}

float GetDirValueFromMeasurement(example_measure_t *measurement, rangeDirection_t dir)
{
    switch (dir)
    {
    case rangeFront:
        return measurement->front;
        break;
    case rangeBack:
        return measurement->back;
        break;
    case rangeLeft:
        return measurement->left;
        break;
    case rangeRight:
        return measurement->right;
        break;
    case rangeUp:
        return measurement->up;
        break;
    case rangeDown:
        return measurement->down;
        break;
    default:
        return -1;
        break;
    }
}

int MoveTo(float x, float y, float z)
{
    return crtpCommanderHighLevelGoTo((x - OFFSET_X) / 100, (y - OFFSET_Y) / 100, (z - OFFSET_Z) / 100, 0, 0.5, 0);
}

void appMain()
{
    DEBUG_PRINT("appMain start\n");
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    coordinate_t start_point = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinateF_t item_start = {OFFSET_X, OFFSET_Y, OFFSET_Z};

    coordinate_t candinates[6];
    double direction_weight[6];
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_cost, item_sum;
    short dir_next = -1;
    int dir_last = 0;
    for (int i = 0; i < 6; ++i)
    {
        direction_weight[i] = 1;
    }

    Queue_t queue;
    initQueue(&queue);
    short loops[WINDOW_SIZE];
    short index_loop = 0;

    for (int i = 0; i < WINDOW_SIZE; ++i)
    {
        loops[i] = 0;
    }
    while (1)
    {
        vTaskDelay(666);
        if (octotree_Flying)
        {
            ++seqnumber;
            item_start.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
            item_start.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
            item_start.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
            DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_start.x, (double)item_start.y, (double)item_start.z, seqnumber);
            vTaskDelay(200);
            get_measurement(&measurement);
            if (item_start.z < COVER)
                measurement.up = COVER - item_start.z;
            else
                measurement.up = 0;
            measurement.down = item_start.z;

            // Update the octoMap
            // float -> int
            start_point.x = item_start.x;
            start_point.y = item_start.y;
            start_point.z = item_start.z;

            index_loop = ((start_point.x + start_point.y + start_point.z) / TREE_RESOLUTION) % WINDOW_SIZE;
            ++loops[index_loop];
            if (loops[index_loop] < MAX_LOOP)
            {
                push(&queue, index_loop);
                if (queue.len == WINDOW_SIZE)
                {
                    index_loop = pop(&queue);
                    --loops[index_loop];
                }
            }
            else
            {
                DEBUG_PRINT("[app]Loop detected!\n");
                // clear loop count
                initQueue(&queue);
                for (int i = 0; i < WINDOW_SIZE; ++i)
                {
                    loops[i] = 0;
                }
                rangeDirection_t dir = (rangeDirection_t)(rand() % 6);
                short k = 0;
                float length = GetDirValueFromMeasurement(&measurement, dir);
                while (length < TREE_RESOLUTION && k < 20)
                {
                    dir = (rangeDirection_t)(rand() % 6);
                    length = GetDirValueFromMeasurement(&measurement, dir);
                    ++k;
                }
                float item_roll = measurement.roll, item_pitch = measurement.pitch, item_yaw = measurement.yaw;
                coordinateF_t item_point = item_start;
                coordinateF_t midTarget;
                for (k = 1; length > TREE_RESOLUTION; ++k)
                {
                    cal_PointByLength(TREE_RESOLUTION * k, item_pitch, item_roll, item_yaw, &item_point, dir, &midTarget);
                    MoveTo(midTarget.x, midTarget.y, midTarget.z);
                    // UpdateMap
                    item_start.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + TREE_CENTER_X;
                    item_start.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + TREE_CENTER_Y;
                    item_start.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
                    DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_start.x, (double)item_start.y, (double)item_start.z, seqnumber);

                    get_measurement(&measurement);
                    if (item_start.z < COVER)
                        measurement.up = COVER - item_start.z;
                    else
                        measurement.up = 0;
                    measurement.down = item_start.z;

                    start_point.x = item_start.x;
                    start_point.y = item_start.y;
                    start_point.z = item_start.z;

                    UpdateMap(&octoMap, &measurement, &item_start, &start_point);
                }
                //
                DEBUG_PRINT("[app]Loop End!\n");
                continue;
            }

            UpdateMap(&octoMap, &measurement, &item_start, &start_point);

            // cal income of the candidate
            // 计算下一位置代价
            CalCandidates(candinates, &measurement, &item_start);
            DEBUG_PRINT("candidates[i]:(%d,%d,%d)\n", candinates[0].x, candinates[0].y, candinates[0].z);
            max_candinateCost = 0;
            for (int i = 0; i < 6; ++i)
            {
                item_candinateCost = 0;
                item_sum.cost_prune = 0;
                item_sum.income_info = 0;
                // printf("candinates[%d]:(%d,%d,%d)\n",i,candinates[i].x,candinates[i].y,candinates[i].z);
                if (candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000)
                {
                    continue;
                }
                for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
                {
                    item_cost = Cost_Sum(octoMap.octoTree, &octoMap, &candinates[i], dir);
                    item_sum.cost_prune += item_cost.cost_prune;
                    item_sum.income_info += item_cost.income_info;
                }
                if (item_sum.income_info == 0)
                {
                    item_sum.income_info = DISCIPLINE;
                }
                // printf("item_sum.cost_prune:%f,item_sum.income_info:%f\n",item_sum.cost_prune,item_sum.income_info);
                // printf("direction_weight[%d]:%.2f\n",i,direction_weight[i]);
                item_candinateCost = direction_weight[i] * (((double)octoMap.octoNodeSet->length / NODE_SET_SIZE) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                            (1.0 - (double)octoMap.octoNodeSet->length / NODE_SET_SIZE) * item_sum.income_info * INCOME_INFO_TIMES);
                // printf("fullqueueentry:%d\n",octoMap->octoNodeSet->fullQueueEntry);
                // printf("candinates_cost[%d]:%f\n",i,candinates_cost[i]);
                if (item_candinateCost > max_candinateCost)
                    dir_next = i;
            }
            if (dir_next != -1)
            {
                DEBUG_PRINT("nextdir:%d\n",dir_next);
                direction_weight[dir_next] = DIRECTION_AWARD;
                direction_weight[dir_last] = 1;
                dir_last = dir_next;
                // Move
                MoveTo((float)candinates[dir_next].x, (float)candinates[dir_next].y, (float)candinates[dir_next].z);
            }
            else
            {
                DEBUG_PRINT("no next point\n");
                break;
            }
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