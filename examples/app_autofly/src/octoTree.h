#ifndef __OCTOTREE_H__
#define __OCTOTREE_H__
#pragma once
#include <stdint.h>
#include "octoMap.h"
#include "octoNode.h"

//#define BOOL int
#define TRUE 1
#define FALSE 0

#define P_GLOBAL 0.5
#define ALPHA 1
#define BETA 1
#define MIN_OCCUPIED 5
#define MAX_NOT_OCCUPIED 2

octoTree_t* octoTreeInit(octoNodeSet_t* nodeSet);
void octoTreeInsertPoint(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds);
void octoTreeRayCasting(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);
uint8_t octoTreeGetLogProbability(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point);
void bresenham3D(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);

double caldistance(coordinate_t* A,coordinate_t* B);
octoNode_t *findTargetParent(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t origin, uint16_t* width, uint8_t* maxDepth);
costParameter_t Cost(coordinate_t *Point,octoTree_t *octoTree, octoMap_t *octoMap, octoNode_t *LastoctoNode);
double Cost_Sum(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *start, coordinate_t *end);
#endif
