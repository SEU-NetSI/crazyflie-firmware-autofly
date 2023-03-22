#include "rrtConnect.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "debug.h"
void planning(coordinate_t X_start, coordinate_t X_end, octoTree_t *octoTree, octoMap_t *octoMap,array_t* result)
{
    //DEBUG_PRINT("Start planning\n");
    //DEBUG_PRINT("X_start %d,%d,%d\n", X_start.x, X_start.y, X_start.z);
    //DEBUG_PRINT("X_end %d,%d,%d\n", X_end.x, X_end.y, X_end.z);
    //DEBUG_PRINT("ITER_MAX %d\n", ITER_MAX);
    //DEBUG_PRINT("MIN_DISTANCE %d\n", MIN_DISTANCE);
    //DEBUG_PRINT("STRIDE %d\n", STRIDE);
    //char* filename = "../assets/rrtPath.csv";
    //FILE *fp = fopen(filename, "w");
    // fprintf(fp, "%d,%d,%d,", X_start.x, X_start.y, X_start.z);
    // fprintf(fp, "%d,%d,%d,0\n", X_end.x, X_end.y, X_end.z);
    result->len = 0;
    if (caldistance(&X_start, &X_end) < MIN_DISTANCE)
        return;
    array_t current1,current2;
    current1.len = 0;
    current2.len = 0;
    addToArray_coordinate(&current1, X_start, NULL);
    addToArray_coordinate(&current2, X_end, NULL);
    vertex_t X_rand, X_new_1,X_new_2;
    int near_index_1 = -1,near_index_2 = -1;
    vertex_t *X_near_1,*X_near_2;
    vertex_t *X_connect_1 = &current1.arr[0],*X_connect_2 = &current2.arr[1];
    for (int i = 0; i < ITER_MAX; ++i)
    {
        // printf("i:%d\n",i);
        //printf("probability_end: %d",octoTreeGetLogProbability(octoTree, octoMap, &X_end));
        do
        {
            if (rand() % 100 < 100 * PROBABILITY_THRESHOLD)
                generate_random_node(&X_rand.loc);
            else
                X_rand.loc = X_end;
        } while (octoTreeGetLogProbability(octoTree, octoMap, &X_rand.loc) >= MAX_PROBABILITY);
        // printf("X_rand %d,%d,%d\n",X_rand.loc.x,X_rand.loc.y,X_rand.loc.z);
        near_index_1 = find_nearest_neighbor(&X_rand.loc, &current1);
        near_index_2 = find_nearest_neighbor(&X_rand.loc, &current2);
        X_near_1 = &current1.arr[near_index_1];
        X_near_2 = &current2.arr[near_index_2];
        if(caldistance(&X_near_1->loc,&X_rand.loc)<0.5 || caldistance(&X_near_2->loc,&X_rand.loc)<0.5)
            continue;
        // printf("X_near %d,%d,%d\n",X_near->loc.x,X_near->loc.y,X_near->loc.z);
        X_new_1 = steer(&X_near_1->loc, &X_rand.loc);
        X_new_2 = steer(&X_near_2->loc, &X_rand.loc);
        // printf("X_end %d,%d,%d\n",X_end.x,X_end.y,X_end.z);
        if (obstaclefree(octoTree, octoMap, X_near_1->loc, X_new_1.loc))
        {
            // printf("obstaclefree true\n");
            X_new_1.parent = X_near_1;
            // fprintf(fp, "%d,%d,%d,", X_new_1.loc.x, X_new_1.loc.y, X_new_1.loc.z);
            // fprintf(fp, "%d,%d,%d,1\n", X_new_1.parent->loc.x, X_new_1.parent->loc.y, X_new_1.parent->loc.z);
            if (!addToArray_vertex(&current1, X_new_1))
                break;
            X_connect_2 = &current2.arr[find_nearest_neighbor(&X_new_1.loc, &current2)];
            if (caldistance(&X_new_1.loc, &X_connect_2->loc) <= 2*MIN_DISTANCE)
            {
                X_connect_1 = &X_new_1;
                break;
            }
        }
        else
        {
            DEBUG_PRINT("[rrtC]obstaclefree false\n");
        }

        if (obstaclefree(octoTree, octoMap, X_near_2->loc, X_new_2.loc))
        {
            // printf("obstaclefree true\n");
            X_new_2.parent = X_near_2;
            // fprintf(fp, "%d,%d,%d,", X_new_2.loc.x, X_new_2.loc.y, X_new_2.loc.z);
            // fprintf(fp, "%d,%d,%d,-1\n", X_new_2.parent->loc.x, X_new_2.parent->loc.y, X_new_2.parent->loc.z);
            if (!addToArray_vertex(&current2, X_new_2))
                break;
            X_connect_1 = &current1.arr[find_nearest_neighbor(&X_new_2.loc, &current1)];
            if (caldistance(&X_new_2.loc, &X_connect_1->loc) <= 2*MIN_DISTANCE)
            {
                X_connect_2 = &X_new_2;
                break;
            }
        }
        else
        {
            DEBUG_PRINT("[rrtC]obstaclefree false\n");
        }
    }
    if (caldistance(&X_connect_1->loc, &X_connect_2->loc) <= 2*MIN_DISTANCE)
    {
        array_t item;
        vertex_t *p = X_connect_1;
        while (p != NULL)
        {
            addToArray_vertex(&item, *p);
            p = p->parent;
        }
        for (int i = item.len - 1; i >= 0; i--)
        {
            addToArray_vertex(result, item.arr[i]);
            // fprintf(fp,"%d,%d,%d,2\n",item.arr[i].loc.x,item.arr[i].loc.y,item.arr[i].loc.z);
        }
        p = X_connect_2;
        while (p != NULL)
        {
            addToArray_vertex(result, *p);
            // fprintf(fp,"%d,%d,%d,-2\n",p->loc.x,p->loc.y,p->loc.z);
            p = p->parent;
        }
        for(int i=1;i<result->len;i++)
            result->arr[i].parent = &result->arr[i-1];
    }
    //writefile(&current1,filename,1);
    //writefile(&current2,filename,-1);
    //writefile(&result,filename,2);
    //fclose(fp);
    //return result;
}

void generate_random_node(coordinate_t *X_rand)
{
    X_rand->x = rand() % MAXRAND;
    X_rand->y = rand() % MAXRAND;
    X_rand->z = rand() % MAXRAND;
}

int find_nearest_neighbor(coordinate_t *X_rand, array_t *current)
{
    int len = current->len;
    int min = 0;
    double min_distance = caldistance(X_rand, &(current->arr[0].loc));
    for (int i = 1; i < len; i++)
    {
        double distance = caldistance(X_rand, &(current->arr[i].loc));
        if (distance < min_distance)
        {
            min_distance = distance;
            min = i;
        }
    }
    //printf("len:%d,min:%d\n", len, min);
    return min;
}

vertex_t steer(coordinate_t *X_near, coordinate_t *X_rand)
{
    double length = caldistance(X_near, X_rand);
    vertex_t X_new;
    X_new.loc.x = fmax(0,fmin(X_near->x + (X_rand->x - X_near->x) * (double)STRIDE / length,WIDTH));
    X_new.loc.y = fmax(0,fmin(X_near->y + (X_rand->y - X_near->y) * (double)STRIDE / length,WIDTH));
    X_new.loc.z = fmax(0,fmin(X_near->z + (X_rand->z - X_near->z) * (double)STRIDE / length,WIDTH));
    return X_new;
}

bool obstaclefree(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t start, coordinate_t end)
{
    float d = caldistance(&start, &end);
    float dx = TREE_RESOLUTION * (end.x - start.x) / d;
    float dy = TREE_RESOLUTION * (end.y - start.y) / d;
    float dz = TREE_RESOLUTION * (end.z - start.z) / d;
    int sgl_x = dx > 0 ? 1 : -1;
    int sgl_y = dy > 0 ? 1 : -1;
    int sgl_z = dz > 0 ? 1 : -1;
    int dx_i = dx;
    int dy_i = dy;
    int dz_i = dz;
    float dif_x = (dx - dx_i);
    float dif_y = (dy - dy_i);
    float dif_z = (dz - dz_i);
    //printf("dx:%f,dy:%f,dz:%f\n", dx, dy, dz);
    //printf("sgl_x %d,sgl_y %d,sgl_z %d\n", sgl_x, sgl_y, sgl_z);
    //printf("dx_i %d,dy_i %d,dz_i %d\n", dx_i, dy_i, dz_i);
    //printf("dif_x %f,dif_y %f,dif_z %f\n", dif_x, dif_y, dif_z);
    float error_x = 0;
    float error_y = 0;
    float error_z = 0;

    uint8_t probability = 3;
    coordinate_t point = start;
    // for x in range(startPoint[0], endPoint[0], step[0]):
    while (caldistance(&point, &end) > MIN_DISTANCE)
    {
        probability = 3;
        point.x += dx_i;
        error_x += dif_x;
        if (error_x >=(float)0.5 || error_x <= (float)-0.5)
        {
            point.x += sgl_x;
            error_x -= sgl_x;
        }
        point.y += dy_i;
        error_y += dif_y;
        if (error_y >= (float)0.5 || error_y <= (float)-0.5)
        {
            point.y += sgl_y;
            error_y -= sgl_y;
        }
        point.z += dz_i;
        error_z += dif_z;
        if (error_z >= (float)0.5 || error_z <= (float)-0.5)
        {
            point.z += sgl_z;
            error_z -= sgl_z;
        }

        probability = octoTreeGetLogProbability(octoTree, octoMap, &point);
        //printf("d:%f,dx:%f,dy:%f,dz:%f\n",d,dx,dy,dz);
        // printf("point:%d,%d,%d , probability:%d \n",point.x,point.y,point.z,probability);
        // printf("start:%d,%d,%d , end:%d,%d,%d \n",start.x,start.y,start.z,end.x,end.y,end.z);
        if (probability >= MAX_PROBABILITY)
            return false;
    }
    return true;
}

BOOL addToArray_vertex(array_t *array, vertex_t element)
{
    if (array->len >= MAX_ARRAY_SIZE)
    {
        DEBUG_PRINT("[rrtC]Array is full\n");
        return FALSE;
    }
    array->arr[array->len] = element;
    array->len++;
    return TRUE;
}

BOOL addToArray_coordinate(array_t *array, coordinate_t element, vertex_t *parent)
{
    if (array->len >= MAX_ARRAY_SIZE)
    {
        DEBUG_PRINT("[rrtC]Array is full\n");
        return FALSE;
    }
    array->arr[array->len].loc = element;
    array->arr[array->len].parent = parent;
    array->len++;
    return TRUE;
}
/*
void writefile(array_t* array, char* filename,int flag){
    FILE* fp = fopen(filename, "a");
    fprintf(fp,"%d\n",array->len);
    for(int i=1;i<array->len;i++){
        fprintf(fp, "%d,%d,%d", array->arr[i].loc.x, array->arr[i].loc.y, array->arr[i].loc.z);
        fprintf(fp, ",%d,%d,%d", array->arr[i].parent->loc.x, array->arr[i].parent->loc.y, array->arr[i].parent->loc.z);  
        fprintf(fp,",%d\n",flag);
    }
    fclose(fp);
    return;
}*/