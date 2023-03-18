#include "octoMap.h"
#include "octoTree.h"

#define ITER_MAX 10000
#define MAX_ARRAY_SIZE 10000
#define MAXRAND 500
#define MIN_DISTANCE TREE_RESOLUTION
#define STRIDE 8
#define MAX_PROBABILITY 4
#define PROBABILITY_THRESHOLD 0.8

typedef struct vertex_t{
    coordinate_t loc;
    struct vertex_t *parent;
}vertex_t;

typedef struct array_t{
    vertex_t arr[MAX_ARRAY_SIZE];
    int len;
}array_t;

array_t planning(coordinate_t X_start,coordinate_t X_end,octoTree_t *octoTree,octoMap_t *octoMap);
void generate_random_node(coordinate_t* X_rand);
int find_nearest_neighbor(coordinate_t* X_rand,array_t *current);
vertex_t steer(coordinate_t* X_near,coordinate_t* X_rand);
bool obstaclefree(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t start, coordinate_t end);
BOOL addToArray_vertex(array_t* array, vertex_t element);
BOOL addToArray_coordinate(array_t* array, coordinate_t element,vertex_t *parent);

void writefile(array_t* array, char* filename,int flag);
