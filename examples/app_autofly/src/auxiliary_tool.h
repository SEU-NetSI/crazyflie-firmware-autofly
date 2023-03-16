#pragma once
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

example_measure_t get_measurement();
coordinate_t cal_Point(example_measure_t* get_measurement,coordinate_t start_point,rangeDirection_t dir); 

coordinate_t rot(float roll, float pitch, float yaw, coordinate_t origin, coordinate_t point);
void determine_threshold(coordinate_t *point);
void dot(float A[][3], float B[][1]);