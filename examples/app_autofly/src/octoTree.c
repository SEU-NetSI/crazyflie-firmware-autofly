#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "octoTree.h"
#include "octoNodeSet.h"

#include "debug.h"

/**
 * @brief initialize an octoTree
 *
 * @param octoTree
 */
octoTree_t g_OctoTree;
octoNode_t g_OctoNode;
octoTree_t *octoTreeInit(octoNodeSet_t *nodeSet) {
    // init octoTree rootNode
    octoNode_t *root = &g_OctoNode;
    root->logOdds = 3;
    root->isLeaf = TRUE;
    // init octoTree
    g_OctoTree.center.x = TREE_CENTER_X;
    g_OctoTree.center.y = TREE_CENTER_Y;
    g_OctoTree.center.z = TREE_CENTER_Z;
    g_OctoTree.origin.x = TREE_CENTER_X - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.origin.y = TREE_CENTER_Y - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.origin.z = TREE_CENTER_Z - TREE_RESOLUTION * (1 << TREE_MAX_DEPTH) / 2;
    g_OctoTree.resolution = TREE_RESOLUTION;
    g_OctoTree.maxDepth = TREE_MAX_DEPTH;
    g_OctoTree.width = TREE_RESOLUTION * pow(2, TREE_MAX_DEPTH);
    g_OctoTree.root = root;
    return &g_OctoTree;
}

/**
 * @brief Add an observation to the octomap.
 *
 * @param octoTree self
 * @param point the coordinate of the observation lidar point --- (x,y,z): tuple
 * @param diffLogOdds the difference value of logodds, 0: free, 1: occupied
 */
void octoTreeInsertPoint(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds) {
    octoNodeUpdate(octoTree->root, octoMap, point, diffLogOdds, octoTree->origin, octoTree->width, octoTree->maxDepth);
}

/**
 * @brief Add the probability of the grid occupied by the ray path to the tree
 *
 * @param octoTree self
 * @param startPoint the coordinate of the sensor --- (x,y,z): tuple
 * @param endPoint the coordinate of the observation point  --- (x,y,z): tuple
 * @param diffLogOdds the difference value of logodds, 0: free, 1: occupied
 */
void octoTreeRayCasting(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint) {
    // print parameters
//    printf("[OctoTree.ray_casting] start_point: (%d, %d, %d) end_point: (%d, %d, %d)\n", startPoint->x,
//           startPoint->y, startPoint->z, endPoint->x, endPoint->y, endPoint->z);
    // Insert occupancy voxel
    octoTreeInsertPoint(octoTree, octoMap, endPoint, LOG_ODDS_OCCUPIED_FLAG);
    // call bresenham algorithm to insert free voxel
    bresenham3D(octoTree, octoMap, startPoint, endPoint);
}

/**
 * @brief Return the occupancy probability of the voxel at a given point coordinate.
 *
 * @param octoTree self
 * @param point coordinate of some voxel to get probability --- (x,y,z): tuple
 * @return uint8_t occupancy probability of the corresponding voxel
 */
uint8_t octoTreeGetLogProbability(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point) {
    return octoNodeLogOddsAt(octoTree->root, octoMap, point, octoTree->origin, octoTree->width);
}

/**
 * @brief bresenham algorithm for 3D ray casting
 *
 * @param octoTree self
 * @param octoMap self
 * @param start start point of the ray
 * @param end end point of the ray
 */
void bresenham3D(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, coordinate_t *end)
{
    uint16_t steepXY = (abs(end->y - start->y) > abs(end->x - start->x));
    if (steepXY) {
        uint16_t temp = start->x;
        start->x = start->y;
        start->y = temp;
        temp = end->x;
        end->x = end->y;
        end->y = temp;
    }
    uint16_t steepXZ = (abs(end->z - start->z) > abs(end->x - start->x));
    if (steepXZ) {
        uint16_t temp = start->x;
        start->x = start->z;
        start->z = temp;
        temp = end->x;
        end->x = end->z;
        end->z = temp;
    }

    uint16_t deltaX = abs(end->x - start->x);
    uint16_t deltaY = abs(end->y - start->y);
    uint16_t deltaZ = abs(end->z - start->z);

    int errorXY = deltaX / 2;
    int errorXZ = deltaX / 2;

    uint16_t stepX = start->x < end->x ? TREE_RESOLUTION : -TREE_RESOLUTION;
    uint16_t stepY = start->y < end->y ? TREE_RESOLUTION : -TREE_RESOLUTION;
    uint16_t stepZ = start->z < end->z ? TREE_RESOLUTION : -TREE_RESOLUTION;

    uint16_t x = start->x;
    uint16_t y = start->y;
    uint16_t z = start->z;
    while (abs(x - end->x) > TREE_RESOLUTION) {
        coordinate_t pointCoordinate = {x, y, z};
        coordinate_t* point = &pointCoordinate;
        if (steepXZ) {
            int temp = point->x;
            point->x = point->z;
            point->z = temp;
        }
        if (steepXY) {
            int temp = point->x;
            point->x = point->y;
            point->y = temp;
        }

        errorXY -= deltaY;
        errorXZ -= deltaZ;
        if (errorXY < 0) {
            y += stepY;
            errorXY += deltaX;
        }
        if (errorXZ < 0) {
            z += stepZ;
            errorXZ += deltaX;
        }

        octoTreeInsertPoint(octoTree, octoMap, point, LOG_ODDS_FREE_FLAG);
        x += stepX;
    }
}


double caldistance(coordinate_t* A,coordinate_t* B){
    return sqrt( pow(A->x - B->x,2) + pow(A->y - B->y,2) + pow(A->z - B->z,2) );
}

/**
 * @brief find the target node's parent node and and save the maxDepth by reference
 * @param octoNode self
 * @param point the point coordinate of the observation --- (x,y,z): tuple
 * @param origin origin of this node --- (x,y,z): tuple
 * @param width width of this node --- int
 * @param maxDepth maximum depth this node can be branched --- int
 */
octoNode_t *findTargetParent(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t origin, uint16_t* width, uint8_t* maxDepth)
{
    if (octoNode->isLeaf == 1)
    {
        return NULL;
    }
    else
    {
        uint8_t index = octoNodeIndex(point, origin, *width);
        // if the node is leaf node, return its parent node
        if (octoMap->octoNodeSet->setData[octoNode->children].data[index].isLeaf)
            return octoNode;
        coordinate_t newOrigin = calOrigin(index, origin, *width);
        *width = *width/2;
        *maxDepth = *maxDepth - 1;
        return findTargetParent(&octoMap->octoNodeSet->setData[octoNode->children].data[index], octoMap, point, newOrigin, width, maxDepth);
    }
}

/**
 * @brief calculate the node's cost,while the node contains the point
 * @param point the calculating point coordinate --- (x,y,z): tuple
 * @param octoTree self
 * @param octoMap self
 */
costParameter_t Cost(coordinate_t *point, octoTree_t *octoTree, octoMap_t *octoMap,octoNode_t *LastoctoNode)
{
    costParameter_t costParameter;
    uint8_t Depth = octoTree->maxDepth;
    uint16_t Width = octoTree->width;
    octoNode_t *octoNode = findTargetParent(octoTree->root, octoMap, point, octoTree->origin, &Width, &Depth);
    if (octoNode == NULL){ // if the node is root, just return the cost
        costParameter.node = octoTree->root;
        costParameter.cost = 8 * Depth;
        costParameter.p_not_occupied = 1-P_GLOBAL;
        return costParameter;
    }
    uint8_t index = octoNodeIndex(point, octoNode->origin, Width);
    costParameter.node = &octoMap->octoNodeSet->setData[octoNode->children].data[index];
    //Duplicate node, no contribution
    if(costParameter.node == LastoctoNode){
        costParameter.node = LastoctoNode;
        costParameter.cost = 0;
        costParameter.p_not_occupied = 1;
        return costParameter;
    }
    //if the node is known to be not occupied, continue
    if(costParameter.node->logOdds <= MAX_NOT_OCCUPIED){
        costParameter.cost = 0;
        costParameter.p_not_occupied = 1;
        return costParameter;
    }
    //if the node is known to be occupied, break
    else if(costParameter.node->logOdds >= MIN_OCCUPIED){
        costParameter.cost = 0;
        costParameter.p_not_occupied = 0;
        return costParameter;
    }
    // calculate the cost
    double cost = 8 * (Depth - 1);
    // calculate the cost_prune
    double cost_prune = 0;
    int allsame = -1;
    double p;
    int n = 0;
    int i;
    for (i = 0; i < 8; ++i)
    {
        octoNode_t *temp = &octoMap->octoNodeSet->setData[octoNode->children].data[i];
        // if the node is not leaf, return 0
        if (!temp->isLeaf)
        {
            break;
        }
        // if the node is leaf, check its occupy
        if (allsame == -1)
        {
            if (temp->logOdds >= MIN_OCCUPIED)
                allsame = 1;
            else if (temp->logOdds <= MAX_NOT_OCCUPIED)
                allsame = 0;
        }
        // the occupy is not same
        else if (allsame == 1 && temp->logOdds <= MAX_NOT_OCCUPIED){
            costParameter.cost = ALPHA * cost;
            costParameter.p_not_occupied = 1-P_GLOBAL;
            return costParameter;
        }
        else if (allsame == 0 && temp->logOdds >= MIN_OCCUPIED){
            costParameter.cost = ALPHA * cost;
            costParameter.p_not_occupied = 1-P_GLOBAL;
            return costParameter;
        }
        // the occupy is same
        else if (!(temp->logOdds <= MIN_OCCUPIED && temp->logOdds >= MAX_NOT_OCCUPIED))
            ++n;
    }
    if(allsame == 1){ // occupied
        p = P_GLOBAL + (1 - P_GLOBAL) * (double)n / 8;
        costParameter.p_not_occupied = 1-p;
    }
    else if(allsame == 0){ // not occupied
        p = (1 - P_GLOBAL) + P_GLOBAL * (double)n / 8;
        costParameter.p_not_occupied = p;
    }
    else{ // unknown
        p = 1 - P_GLOBAL;
        costParameter.p_not_occupied = 1-P_GLOBAL;
    }
    cost_prune = 8 * pow(p, 8-n); // 根据p_global值固定与否可以考虑数组存值替代幂运算
    costParameter.cost = ALPHA * cost - BETA * cost_prune;
    return costParameter;
    // return cost，octonode，p_not_occupied
}

double Cost_Sum(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, coordinate_t *end)
{
    int16_t dx = (int16_t)abs(end->x - start->x);
    int16_t dy = (int16_t)abs(end->y - start->y);
    int16_t dz = (int16_t)abs(end->z - start->z);
    int16_t sx = start->x < end->x ? 1 : -1;
    int16_t sy = start->y < end->y ? 1 : -1;
    int16_t sz = start->z < end->z ? 1 : -1;
    int16_t err = (int16_t)(dx - dy - dz);
    int16_t e2;

    double cost_sum = 0;
    double income_info = 0;
    double p_iter = 1;
    costParameter_t item;
    octoNode_t* LastoctoNode = NULL;
    while (1)
    {
        if (start->x == end->x && start->y == end->y && start->z == end->z)
        {
            break;
        }
        e2 = (int16_t)(2 * err);
        if (e2 > -dx)
        {
            err = (int16_t)(err - dy - dz);
            start->x = start->x + sx;
        }
        if (e2 < dx)
        {
            err = (int16_t)(err + dy + dz);
            start->y = start->y + sy;
        }
        if (e2 < dy)
        {
            err = (int16_t)(err + dy + dz);
            start->z = start->z + sz;
        }
        item = Cost(start,octoTree,octoMap,LastoctoNode);
        if(item.node->logOdds >= MIN_OCCUPIED) //the node is occupied, break
            break;
        cost_sum += p_iter * item.cost;
        if(LastoctoNode != NULL)
            income_info += p_iter * caldistance(&(LastoctoNode->origin),&(item.node->origin));
        p_iter *= item.p_not_occupied;
        if(item.node == octoTree->root)
            break;
        LastoctoNode = item.node;
    }
    return cost_sum - income_info;
}
