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
#include "config_autofly.h"
#include "circularQueue.h"

static bool octotree_Flying = false;
/*
make cload:
    CLOAD_CMDS="-w radio://0/43/2M/E7E7E7E72B" make cload
*/

void appMain()
{
    DEBUG_PRINT("appMain start\n");
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    coordinate_t start_point = {TREE_CENTER_X, TREE_CENTER_Y, 0};
    coordinateF_t item_start = {TREE_CENTER_X, TREE_CENTER_Y, 0};
    coordinateF_t item_end = {0, 0, 0};
    coordinate_t end_point;
    // coordinate_t target_point = {200,200,200};
    int seqnumber = 0;

    coordinate_t candinates[6];
    double direction_weight[6];
    double item_candinateCost = 0,max_candinateCost = 0;
    Cost_C_t item_cost,item_sum;
    short dir_next = -1;
    int dir_last = 0;
    for(int i = 0;i<6;++i){
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
        vTaskDelay(200);

        get_measurement(&measurement);
            if(measurement.up + item_start.z > COVER){
                if(COVER > item_start.z)
                    measurement.up = COVER - item_start.z;
                else
                    measurement.up = 0;
            }
        measurement.down = item_start.z;

        if(octotree_Flying){
            ++seqnumber;
            item_start.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + TREE_CENTER_X;
            item_start.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + TREE_CENTER_Y;
            item_start.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
            DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_start.x, (double)item_start.y, (double)item_start.z, seqnumber);

            // continue;
            
            // Update the octoMap

            //float -> int
            start_point.x = item_start.x;
            start_point.y = item_start.y;
            start_point.z = item_start.z;

            index_loop = ((start_point.x+start_point.y+start_point.z)/TREE_RESOLUTION) % WINDOW_SIZE;
            ++loops[index_loop];
            if(loops[index_loop] < MAX_LOOP){
                push(&queue,index_loop);
                if(queue.len == WINDOW_SIZE){
                    index_loop = pop(&queue);
                    --loops[index_loop];
                }
            }
            else{
                //clear loop count
                initQueue(&queue);
                for (int i = 0; i < WINDOW_SIZE; ++i)
                {
                    loops[i] = 0;
                }
                while (1)
                {
                    //rangeDirection_t dir = rand()%6;
                    
                    //MOVE

                }
            }

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
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeFront].x = start_point.x + TREE_RESOLUTION;
                candinates[rangeFront].y = start_point.y;
                candinates[rangeFront].z = start_point.z;
            }
            else{
                candinates[rangeFront].x = 30000;
                candinates[rangeFront].y = 30000;
                candinates[rangeFront].z = 30000;
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
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeBack].x = start_point.x - TREE_RESOLUTION;
                candinates[rangeBack].y = start_point.y;
                candinates[rangeBack].z = start_point.z;
            }
            else{
                candinates[rangeBack].x = 30000;
                candinates[rangeBack].y = 30000;
                candinates[rangeBack].z = 30000;
            }

            if (cal_Point(&measurement, item_start, rangeLeft, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_L: %.2f,seq:%d\n", (double)measurement.left, seqnumber);
                DEBUG_PRINT("[app]EP_L:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeLeft].x = start_point.x;
                candinates[rangeLeft].y = start_point.y + TREE_RESOLUTION;
                candinates[rangeLeft].z = start_point.z;
            }
            else{
                candinates[rangeLeft].x = 30000;
                candinates[rangeLeft].y = 30000;
                candinates[rangeLeft].z = 30000;
            }

            if (cal_Point(&measurement, item_start, rangeRight, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_R: %.2f,seq:%d\n", (double)measurement.right, seqnumber);
                DEBUG_PRINT("[app]EP_R:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeRight].x = start_point.x;
                candinates[rangeRight].y = start_point.y - TREE_RESOLUTION;
                candinates[rangeRight].z = start_point.z;
            }
            else{
                candinates[rangeRight].x = 30000;
                candinates[rangeRight].y = 30000;
                candinates[rangeRight].z = 30000;
            }
            
            if (cal_Point(&measurement, item_start, rangeUp, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                // octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                DEBUG_PRINT("[app]M_U: %.2f,seq:%d\n", (double)measurement.up, seqnumber);
                DEBUG_PRINT("[app]EP_U:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeUp].x = start_point.x;
                candinates[rangeUp].y = start_point.y;
                candinates[rangeUp].z = start_point.z + TREE_RESOLUTION;
            }
            else{
                candinates[rangeUp].x = 30000;
                candinates[rangeUp].y = 30000;
                candinates[rangeUp].z = 30000;
            }

            if (cal_Point(&measurement, item_start, rangeDown, &item_end))
            {
                end_point.x = item_end.x;
                end_point.y = item_end.y;
                end_point.z = item_end.z;
                DEBUG_PRINT("[app]M_D: %.2f,seq:%d\n", (double)measurement.down, seqnumber);
                DEBUG_PRINT("[app]EP_D:(%.2f,%.2f,%.2f),seq:%d\n", (double)item_end.x, (double)item_end.y, (double)item_end.z, seqnumber);
                octoTreeRayCasting(octoMap.octoTree,&octoMap,&start_point,&end_point);
                // octoTreeInsertPoint(octoMap.octoTree, &octoMap, &end_point, LOG_ODDS_OCCUPIED_FLAG);
                candinates[rangeDown].x = start_point.x;
                candinates[rangeDown].y = start_point.y;
                candinates[rangeDown].z = start_point.z - TREE_RESOLUTION;
            }
            else{
                candinates[rangeDown].x = 30000;
                candinates[rangeDown].y = 30000;
                candinates[rangeDown].z = 30000;
            }

            // cal income of the candidate
            //计算下一位置代价
            max_candinateCost = 0;
            for(int i = 0;i<6;++i){
                item_candinateCost = 0;
                item_sum.cost_prune = 0;
                item_sum.income_info = 0;
                // printf("candinates[%d]:(%d,%d,%d)\n",i,candinates[i].x,candinates[i].y,candinates[i].z);
                if(candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000){
                    continue;
                }
                for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
                {
                    item_cost = Cost_Sum(octoMap.octoTree,&octoMap,&candinates[i],dir);
                    item_sum.cost_prune += item_cost.cost_prune;
                    item_sum.income_info += item_cost.income_info;
                }
                if(item_sum.income_info == 0){
                    item_sum.income_info = DISCIPLINE;
                }
                // printf("item_sum.cost_prune:%f,item_sum.income_info:%f\n",item_sum.cost_prune,item_sum.income_info);
                // printf("direction_weight[%d]:%.2f\n",i,direction_weight[i]);
                item_candinateCost = direction_weight[i] * (((double)octoMap.octoNodeSet->length / NODE_SET_SIZE) * item_sum.cost_prune * COST_PRUNE_TIMES + 
                                        (1.0 - (double)octoMap.octoNodeSet->length / NODE_SET_SIZE) * item_sum.income_info * INCOME_INFO_TIMES);
                // printf("fullqueueentry:%d\n",octoMap->octoNodeSet->fullQueueEntry);
                // printf("candinates_cost[%d]:%f\n",i,candinates_cost[i]);
                if(item_candinateCost > max_candinateCost)
                    dir_next = i;
            }
            if(dir_next != -1){
                direction_weight[dir_next] = DIRECTION_AWARD;
                direction_weight[dir_last] = 1;
                dir_last = dir_next;
                //Move

            }
            else{
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