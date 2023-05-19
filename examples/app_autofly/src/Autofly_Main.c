// #include "pmsis.h"

#include "stdlib.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"

#include "config_autofly.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"
#include "rrtConnect.h"
#include "circularQueue.h"
#include "auxiliary_tool.h"
#include "coordinateQueue.h"

#define MOVE_DELAY 800
#define RE_DELAY 300
#define PROBABILITY_MEM(octomap) (double)octomap->octoNodeSet->length / NODE_SET_SIZE

static bool octotree_Flying = false;
static bool octotree_Print = false;
int seqnumber = 0;
bool flag_JumpLoc = false;
rangeDirection_t Jump_Dir = 0;
Queue_t queue;
short loops[WINDOW_SIZE];
/*
make cload:
    CLOAD_CMDS="-w radio://0/86/2M/86E7E7E7E7" make cload
*/
void UpdateMap(octoMap_t *octoMap, example_measure_t *measurement, coordinateF_t *current_F, coordinate_t *current_I);
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
rangeDirection_t GetRandomDir(example_measure_t *measurement);
bool CalBestCandinates(octoMap_t *octoMap,example_measure_t *measurement, coordinateF_t *current_point, float* direction_weight, short* lastdir, CoordinateQueue_t* paths);
bool JumpLocalOp(coordinateF_t *current_point, example_measure_t* measurement,CoordinateQueue_t* paths);
void printOctomap(octoMap_t *octoMap);
void MoveTo(float x, float y, float z);
bool MoveToNextPoint(CoordinateQueue_t* paths);
bool ReliabilityTest(coordinateF_t* last, coordinateF_t* cur);

void UpdateMap(octoMap_t *octoMap, example_measure_t *measurement, coordinateF_t *current_F, coordinate_t *current_I)
{
    coordinate_t end_point;
    coordinateF_t item_end;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (cal_Point(measurement, current_F, dir, &item_end))
        {
            end_point.x = item_end.x;
            end_point.y = item_end.y;
            end_point.z = item_end.z;
            octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, &end_point);
        }
    }
}

void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &candidates[dir]);
        }
        else
        {
            candidates[dir].x = 30000;
            candidates[dir].y = 30000;
            candidates[dir].z = 30000;
        }
    }
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

bool CalBestCandinates(octoMap_t *octoMap,example_measure_t *measurement, coordinateF_t *current_point, float* direction_weight, short* lastdir, CoordinateQueue_t* paths){
    coordinateF_t candinates[6];
    coordinate_t item_point;
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_sum,item_cost;
    CalCandidates(candinates, measurement, current_point);
    max_candinateCost = 0;
    short dir_next = -1;
    for(int i = 0;i<6;++i){
        item_candinateCost = 0;
        item_sum.cost_prune = 0;
        item_sum.income_info = 0;
        if (candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000)
        {
            continue;
        }
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            item_point.x = candinates[i].x;
            item_point.y = candinates[i].y;
            item_point.z = candinates[i].z;
            item_cost = Cost_Sum(octoMap->octoTree, octoMap, &item_point, dir);
            item_sum.cost_prune += item_cost.cost_prune;
            item_sum.income_info += item_cost.income_info;
        }
        if (item_sum.income_info == 0)
        {
            item_sum.income_info = DISCIPLINE;
        }
        item_candinateCost = (double)direction_weight[i] * (PROBABILITY_MEM(octoMap) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                    (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES);
        if (item_candinateCost > max_candinateCost){
            dir_next = i;
            max_candinateCost = item_candinateCost;
        }
    }
    if(dir_next != -1){
        direction_weight[dir_next] = DIRECTION_AWARD;
        direction_weight[(*lastdir)] = 1;
        (*lastdir) = dir_next;
        push_CoordinateQueue(paths, candinates[dir_next]);
        return true;
    }
    else{
        DEBUG_PRINT("no next point\n");
        return false;
    }
}

bool JumpLocalOp(coordinateF_t *current_point, example_measure_t* measurement,CoordinateQueue_t* paths){
    // rangeDirection_t dir = rand()%6;
    float length = fmin(measurement->data[Jump_Dir],300);
    // coordinateF_t item_start_point = {current_point->x,current_point->y,current_point->z};
    coordinateF_t item_end_point;
    if(length > STRIDE + AVOID_DISTANCE){
        // cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, &item_start_point, Jump_Dir, &item_end_point);
        cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, current_point, Jump_Dir, &item_end_point);
        push_CoordinateQueue(paths, item_end_point);
        return true;
        // item_start_point = item_end_point;
        // length -= STRIDE;
    }
    else{
        flag_JumpLoc = false;
        return false;
    }
}

void printOctomap(octoMap_t* octoMap){
    int Free = 0;
    int Occupied = 0;
    for(int i=0;i<NODE_SET_SIZE;++i){
        for(int j=0;j<8;j++){
            if(octoMap->octoNodeSet->setData[i].data[j].isLeaf){
                if(octoMap->octoNodeSet->setData[i].data[j].logOdds == LOG_ODDS_OCCUPIED){
                    ++Occupied;
                }
                else if(octoMap->octoNodeSet->setData[i].data[j].logOdds == LOG_ODDS_FREE){
                    ++Free;
                }
            }
        }
    }
    DEBUG_PRINT("Free:%d,Occupied:%d\n",Free,Occupied);
}

void MoveTo(float x, float y, float z)
{   
    crtpCommanderHighLevelGoTo((x - OFFSET_X) / 100, (y - OFFSET_Y) / 100, (z - OFFSET_Z) / 100, 0, 0.5, 0);
    vTaskDelay(M2T(MOVE_DELAY));
}

bool MoveToNextPoint(CoordinateQueue_t* paths){
    if(isCoordinateQueueEmpty(paths)){
        return false;
    }
    else{
        coordinateF_t next_point = pop_CoordinateQueue(paths);
        MoveTo(next_point.x,next_point.y,next_point.z);
        return true;
    }
}

bool (coordinateF_t* last, coordinateF_t* cur){
    return abs(last.x-cur.x)+abs(last.y-cur.y)+abs(last.z-cur.z) < RELIABILITY_DISTANCE;
}

void appMain()
{
    // DEBUG_PRINT("appMain start\n");
    vTaskDelay(M2T(1000));
    octoMap_t octoMap;
    octoMapInit(&octoMap);
    example_measure_t measurement;
    float direction_weight[6];
    for (int i = 0; i < 6; ++i)
    {
        direction_weight[i] = 1;
    }
    coordinateF_t start_pointF,item_pointF;
    coordinate_t start_pointI;
    CoordinateQueue_t* paths;
    paths = (CoordinateQueue_t*)malloc(sizeof(CoordinateQueue_t));
    //DEBUG_PRINT("CoordinateQueue_t malloc success\n");
    // DEBUG_PRINT("sizeof(CoordianteQueue_t):%d\n", sizeof(CoordinateQueue_t)); // 608
    initCoordinateQueue(paths);
    short lastdir = 0;

    Queue_t queue;
    // DEBUG_PRINT("sizeof(Queue_t):%d\n", sizeof(Queue_t)); //166
    initQueue(&queue);
    short loops[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; ++i)
    {
        loops[i] = 0;
    }
    short index_loop = 0;
    DEBUG_PRINT("init success\n");
    while (1)
    {
        if (octotree_Flying)
        {
            DEBUG_PRINT("seq:%d\n", seqnumber);

            item_pointF.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x")) + OFFSET_X;
            item_pointF.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y")) + OFFSET_Y;
            item_pointF.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z")) + OFFSET_Z;
            if(seqnumber == 1 || ReliabilityTest(start_pointF,item_pointF)){
                start_pointF = item_pointF;
            }
            else{
                DEBUG_PRINT("ReliabilityTest failed\n");
                vTaskDelay(M2T(RE_DELAY));
                continue;
            }
            ++seqnumber;
            DEBUG_PRINT("[app]SP:(%.2f,%.2f,%.2f),seq:%d\n", (double)start_pointF.x, (double)start_pointF.y, (double)start_pointF.z, seqnumber);

            get_measurement(&measurement);
            if (start_pointF.z < TOP)
                measurement.data[4] = TOP - start_pointF.z;
            else
                measurement.data[4] = 0;
            if (start_pointF.z > BOTTOM)
                measurement.data[5] = start_pointF.z - BOTTOM;
            else
                measurement.data[5] = 0;

            start_pointI.x = start_pointF.x;
            start_pointI.y = start_pointF.y;
            start_pointI.z = start_pointF.z;

            UpdateMap(&octoMap, &measurement, &start_pointF, &start_pointI);
            DEBUG_PRINT("[app]NN:%d,%d,%d,%d,%d,seq:%d\n",octoMap.octoNodeSet->length,
                                                        octoMap.octoNodeSet->numFree,octoMap.octoNodeSet->numOccupied,
                                                        octoMap.octoNodeSet->volumeFree,octoMap.octoNodeSet->volumeOccupied,
                                                        seqnumber);
            if(seqnumber % MAXRUN == 0){
                vTaskDelay(M2T(MOVE_DELAY));
                octotree_Flying = false;
                crtpCommanderHighLevelLand(0, 0.5);
                vTaskDelay(M2T(MOVE_DELAY));
            }
            // Check if it is in a local optimum
            index_loop = ((start_pointI.x + start_pointI.y + start_pointI.z) / TREE_RESOLUTION) % WINDOW_SIZE;
            ++loops[index_loop];
            if (loops[index_loop] < MAX_LOOP)
            {
                push(&queue, index_loop);
                if (queue.len >= WINDOW_SIZE)
                {
                    index_loop = pop(&queue);
                    --loops[index_loop];
                }
                if(!flag_JumpLoc && isCoordinateQueueEmpty(paths) && !CalBestCandinates(&octoMap, &measurement, &start_pointF, direction_weight, &lastdir, paths)){
                    initCoordinateQueue(paths);
                    Jump_Dir = GetRandomDir(&measurement);
                    if(Jump_Dir == -1){
                        DEBUG_PRINT("no next dir\n");
                        return;
                    }
                    flag_JumpLoc = true;
                }
            }
            else
            {
                initCoordinateQueue(paths);
                initQueue(&queue);
                for (int i = 0; i < WINDOW_SIZE; ++i)
                {
                    loops[i] = 0;
                }
                Jump_Dir = GetRandomDir(&measurement);
                if(Jump_Dir == -1){
                    DEBUG_PRINT("no next dir\n");
                    return;
                }
                flag_JumpLoc = true;
            }
            if(flag_JumpLoc){
                if(!JumpLocalOp(&start_pointF, &measurement, paths))
                    continue;
            }
            DEBUG_PRINT("flag_JumpLoc:%d,Jump_dir:%d \n", flag_JumpLoc,Jump_Dir);
            if(!MoveToNextPoint(paths)){
                DEBUG_PRINT("error\n");
                break;
            }
        }
        else{
            vTaskDelay(M2T(300));
        }
        if (octotree_Print)
        {
            octotree_Flying = false;
            crtpCommanderHighLevelLand(0, 0.5);
            vTaskDelay(M2T(MOVE_DELAY));
            DEBUG_PRINT("start to print the octotree\n");
            recursiveExportOctoMap(&octoMap, octoMap.octoTree->root, octoMap.octoTree->origin, octoMap.octoTree->width);
            DEBUG_PRINT("print the octotree end\n");
            octotree_Print = false;
            // printOctomap(&octoMap);
        }
    }
}

PARAM_GROUP_START(octotree)
PARAM_ADD(PARAM_UINT8, octotree_Flying, &octotree_Flying)
PARAM_ADD(PARAM_UINT8, octotree_Print, &octotree_Print)
PARAM_GROUP_STOP(octotree)