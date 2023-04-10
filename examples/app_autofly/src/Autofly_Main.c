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

#define MOVE_DELAY 600
#define PROBABILITY_MEM(octomap) (double)octomap->octoNodeSet->length / NODE_SET_SIZE

static bool octotree_Flying = false;
static bool octotree_Print = false;
int seqnumber = 0;

Queue_t queue;
short loops[WINDOW_SIZE];
/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/

void UpdateMap(octoMap_t *octoMap, example_measure_t *measurement, coordinateF_t *current_F, coordinate_t *current_I)
{
    // DEBUG_PRINT("[app]UpdateMap,seq:%d\n", seqnumber);
    // vTaskDelay(200);
    coordinate_t end_point;
    coordinateF_t item_end;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(measurement, current_F, dir, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            // DEBUG_PRINT("[app]M_%d:%.2f,seq:%d\n", (short)dir, (double)measurement->data[dir], seqnumber);
            // DEBUG_PRINT("[app]EP_%d:(%.2f,%.2f,%.2f),seq:%d\n", (short)dir, (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
            // vTaskDelay(300);
            octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
            // octoTreeInsertPoint(octoMap->octoTree, octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
        }
    }
}

void CalCandidates(coordinate_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    coordinateF_t item_point;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &item_point);
            candidates[dir].x = (int)item_point.x;
            candidates[dir].y = (int)item_point.y;
            candidates[dir].z = (int)item_point.z;
        }
        else
        {
            candidates[dir].x = 30000;
            candidates[dir].y = 30000;
            candidates[dir].z = 30000;
        }
    }
}

int MoveTo(float x, float y, float z)
{
    return crtpCommanderHighLevelGoTo((x - OFFSET_X) / 100, (y - OFFSET_Y) / 100, (z - OFFSET_Z) / 100, 0, 0.4, 0);
}

rangeDirection_t GetdirByCost(octoTree_t *octoTree, octoMap_t *octoMap,example_measure_t *measurement, coordinateF_t *current_F){
    rangeDirection_t dir = -1;
    float maxcost = 0;
    float cost = 0;
    Cost_C_t item_cost = {0,0}, item_sum = {0,0};
    coordinateF_t item_pointF;
    coordinate_t item_pointI;
    for (rangeDirection_t i = rangeFront; i <= rangeDown; ++i)
    {
        if (measurement->data[i] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(measurement->data[i]- AVOID_DISTANCE - STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, current_F, i, &item_pointF);
            item_pointI.x = item_pointF.x;
            item_pointI.y = item_pointF.y;
            item_pointI.z = item_pointF.z;
            for (rangeDirection_t j = rangeFront; j <= rangeDown; ++j)
            {
                item_cost = Cost_Sum(octoTree, octoMap, &item_pointI, j);
                item_sum.cost_prune += item_cost.cost_prune;
                item_sum.income_info += item_cost.income_info;
            }
            cost = (PROBABILITY_MEM(octoMap)) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                            (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES;
            if (cost > maxcost)
            {
                maxcost = cost;
                dir = i;
            }
        }
    }
    return dir;
}

rangeDirection_t GetRandomDir(example_measure_t *measurement)
{
    // Randomly sample twice to choose the larger
    rangeDirection_t dir = (rangeDirection_t)rand() % 6;
    rangeDirection_t maxdir = (rangeDirection_t)rand() % 6;
    int i = 0;
    // Guaranteed to get a feasible direction
    while (measurement->data[maxdir] < STRIDE + AVOID_DISTANCE && i < 20)
    {
        maxdir = (rangeDirection_t)rand() % 6;
        ++i;
    }
    // Try to get a better and feasible direction
    dir = (rangeDirection_t)rand() % 6;
    ++i;
    if (i == 20)
        return -1;
    if (measurement->data[dir] > measurement->data[maxdir])
        maxdir = dir;
    return maxdir;
}

void JumpDeath(example_measure_t* measurement,octoMap_t* octoMap, coordinateF_t* start_pointF,coordinate_t* start_pointI){
    DEBUG_PRINT("[app]Loop detected!\n");
    // clear loop count
    initQueue(&queue);
    for (int i = 0; i < WINDOW_SIZE; ++i)
    {
        loops[i] = 0;
    }
    rangeDirection_t dir = GetdirByCost(octoMap->octoTree, octoMap, measurement, start_pointF);
    if(dir == -1)
        dir = GetRandomDir(measurement);
    if (dir == -1)
    {
        DEBUG_PRINT("[app]No feasible direction!\n");
        octotree_Flying = false;
        return;
    }
    // else{
    //     DEBUG_PRINT("[app]Random direction:%d\n", dir);
    // }
    // float length = measurement->data[dir];
    short k;
    // float item_roll = measurement->roll, item_pitch = -1 * measurement->pitch, item_yaw = measurement->yaw;
    // coordinateF_t item_point = *start_pointF;
    coordinateF_t midTarget;
    for (k = 1; measurement->data[dir] > STRIDE + AVOID_DISTANCE; ++k)
    {
        cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, start_pointF, dir, &midTarget);
        // cal_PointByLength(STRIDE * k, item_pitch, item_roll, item_yaw, &item_point, dir, &midTarget);
        MoveTo(midTarget.x, midTarget.y, midTarget.z);
        // DEBUG_PRINT("[app]dir:%d,MidTarget:(%.2f,%.2f,%.2f)\n", dir, (double)midTarget.x, (double)midTarget.y, (double)midTarget.z);
        vTaskDelay(M2T(MOVE_DELAY));
        // UpdateMap
        start_pointF->x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
        start_pointF->y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
        start_pointF->z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
        DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)start_pointF->x, (double)start_pointF->y, (double)start_pointF->z, seqnumber);

        get_measurement(measurement);
        // DEBUG_PRINT("[app]F:%.2f,B:%.2f,L:%.2f,R:%.2f,U:%.2f,D:%.2f\n", (double)measurement->data[0], (double)measurement->data[1], (double)measurement->data[2],
                        // (double)measurement->data[3], (double)measurement->data[4], (double)measurement->data[5]);
        if (start_pointF->z < TOP)
            measurement->data[4] = TOP - start_pointF->z;
        else
            measurement->data[4] = 0;
        if (start_pointF->z > BOTTOM)
            measurement->data[5] = start_pointF->z - BOTTOM;
        else
            measurement->data[5] = 0;

        start_pointI->x = start_pointF->x;
        start_pointI->y = start_pointF->y;
        start_pointI->z = start_pointF->z;

        UpdateMap(octoMap, measurement, start_pointF, start_pointI);
        DEBUG_PRINT("[app]NN:%d,%d,%d,seq:%d\n",octoMap->octoNodeSet->length,octoMap->octoNodeSet->numFree,octoMap->octoNodeSet->numOccupied,seqnumber);
        ++seqnumber;
        // length -= STRIDE;
        if(seqnumber > MAXRUN)
            return;
    }
    --seqnumber;
    //
    DEBUG_PRINT("[app]Loop End!\n");
}

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    coordinate_t start_pointI = {OFFSET_X, OFFSET_Y, OFFSET_Z};
    coordinateF_t start_pointF = {OFFSET_X, OFFSET_Y, OFFSET_Z};

    coordinate_t candinates[6];
    double direction_weight[6];
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_cost, item_sum;
    short dir_next = -1;
    short dir_last = 0;
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

    bool hasprint = false;

    while (1)
    {
        vTaskDelay(M2T(MOVE_DELAY));
        // DEBUG_PRINT("[app]octotree.octotree_Flying:%d\n",octotree_Flying);
        // DEBUG_PRINT("[app]octotree.octotree_Print:%d\n",octotree_Print);
        if (octotree_Flying && seqnumber < MAXRUN)
        {
            ++seqnumber;
            start_pointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
            start_pointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
            start_pointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
            DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)start_pointF.x, (double)start_pointF.y, (double)start_pointF.z, seqnumber);
            // vTaskDelay(200);
            get_measurement(&measurement);
            if (start_pointF.z < TOP)
                measurement.data[4] = TOP - start_pointF.z;
            else
                measurement.data[4] = 0;
            if (start_pointF.z > BOTTOM)
                measurement.data[5] = start_pointF.z - BOTTOM;
            else
                measurement.data[5] = 0;

            // print measurement
            // DEBUG_PRINT("[app]F:%.2f,B:%.2f,L:%.2f,R:%.2f,U:%.2f,D:%.2f\n", (double)measurement.data[0], (double)measurement.data[1], (double)measurement.data[2],
            //             (double)measurement.data[3], (double)measurement.data[4], (double)measurement.data[5]);
            // // float -> int
            start_pointI.x = start_pointF.x;
            start_pointI.y = start_pointF.y;
            start_pointI.z = start_pointF.z;

            // Check if it is in a local optimum
            index_loop = ((start_pointI.x + start_pointI.y + start_pointI.z) / TREE_RESOLUTION) % WINDOW_SIZE;
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
                JumpDeath(&measurement ,&octoMap, &start_pointF, &start_pointI);
                continue;
            }

            UpdateMap(&octoMap, &measurement, &start_pointF, &start_pointI);
            DEBUG_PRINT("[app]NN:%d,%d,%d,seq:%d\n",octoMap.octoNodeSet->length,octoMap.octoNodeSet->numFree,octoMap.octoNodeSet->numOccupied,seqnumber);
            // cal income of the candidate
            // 计算下一位置代价
            CalCandidates(candinates, &measurement, &start_pointF);
            max_candinateCost = 0;
            dir_next = -1;
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
                item_candinateCost = direction_weight[i] * (PROBABILITY_MEM((&octoMap)) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                            (1.0 - PROBABILITY_MEM((&octoMap))) * item_sum.income_info * INCOME_INFO_TIMES);
                // printf("fullqueueentry:%d\n",octoMap->octoNodeSet->fullQueueEntry);
                // printf("candinates_cost[%d]:%f\n",i,candinates_cost[i]);
                if (item_candinateCost > max_candinateCost){
                    dir_next = i;
                    max_candinateCost = item_candinateCost;
                }
            }
            if (dir_next != -1)
            {
                // DEBUG_PRINT("nextdir:%d\n",dir_next);
                direction_weight[dir_next] = DIRECTION_AWARD;
                direction_weight[dir_last] = 1;
                dir_last = dir_next;
                // Move
                // DEBUG_PRINT("candidates:(%d,%d,%d)\n", candinates[dir_next].x, candinates[dir_next].y, candinates[dir_next].z);
                // vTaskDelay(400);
                MoveTo((float)candinates[dir_next].x, (float)candinates[dir_next].y, (float)candinates[dir_next].z);
            }
            else
            {
                DEBUG_PRINT("no next point\n");
                JumpDeath(&measurement, &octoMap, &start_pointF, &start_pointI);
                continue;
            }
        }
        if ((octotree_Print || seqnumber >= MAXRUN) && !hasprint)
        {
            octotree_Flying = false;
            crtpCommanderHighLevelLand(0, 0.5);
            DEBUG_PRINT("start to print the octotree");
            recursiveExportOctoMap(&octoMap, octoMap.octoTree->root, octoMap.octoTree->origin, octoMap.octoTree->width);
            DEBUG_PRINT("print the octotree end");
            hasprint = true;
            octotree_Print = false;
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
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_GROUP_STOP(octotree)