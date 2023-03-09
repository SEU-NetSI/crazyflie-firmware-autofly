#include <stdio.h>
#include <stdlib.h>
typedef struct
{
    float x;
    float y;
    float z;
} coordinate_t;
typedef struct
{
    float front;
    float back;
    float left;
    float right;
    float up;
    float roll;
    float pitch;
    float yaw;
} example_measure_t;

void exampleTaskInit();
bool exampleTaskTest();

coordinate_t rot(float roll, float pitch, float yaw, coordinate_t origin, coordinate_t point);
void determine_threshold(coordinate_t *point);
void dot(float A[][3], float B[][1]);
void rotate_and_create_points(example_measure_t *measurement, coordinate_t start_point, coordinate_t *res);
