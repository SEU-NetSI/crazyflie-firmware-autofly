#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
#include "range.h"
#include "log.h"
#include "math.h"
#include "example.h"
#include "stdlib.h"

#define WIDTH 500
#define SENSOR_TH 500

static void exampleTask(void *);
STATIC_MEM_TASK_ALLOC(exampleTask, EXAMPLE_TASK_STACKSIZE);

static bool isInit = false;

void exampleTaskInit()
{
  if (isInit)
  {
    return;
  }
  isInit = true;
  STATIC_MEM_TASK_CREATE(exampleTask, exampleTask, EXAMPLE_TASK_NAME, NULL, EXAMPLE_TASK_PRI);
}

bool exampleTaskTest()
{
  return true;
}

static void exampleTask(void *parameters)
{
  DEBUG_PRINT("Example task main function is running!\n");
  example_measure_t measurement;
  float range_front = 0;
  float range_back = 0;
  float range_up = 0;
  float range_left = 0;
  float range_right = 0;
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  coordinate_t start_point = {0, 0, 0};
  coordinate_t *points = (coordinate_t *)malloc(4 * sizeof(coordinate_t));
  int i = 0;
  while (true)
  {
    vTaskDelay(M2T(5000));
    ++i;
    DEBUG_PRINT("Iteration: %d\n",i);
    range_front = rangeGet(rangeFront);
    range_back = rangeGet(rangeBack);
    range_up = rangeGet(rangeUp);
    range_left = rangeGet(rangeLeft);
    range_right = rangeGet(rangeRight);
    DEBUG_PRINT("Ranger Front: %f, Back: %f, Up: %f, Left: %f, Right: %f\n", (double)range_front, (double)range_back, (double)range_up, (double)range_left, (double)range_right);

    static int pitchid, rollid, yawid = -1;
    pitchid = logGetVarId("stabilizer", "pitch");
    rollid = logGetVarId("stabilizer", "roll");
    yawid = logGetVarId("stabilizer", "yaw");
    pitch = logGetFloat(pitchid);
    roll = logGetFloat(rollid);
    yaw = logGetFloat(yawid);
    DEBUG_PRINT("Pitch: %f, Roll: %f, Yaw: %f\n", (double)pitch, (double)roll, (double)yaw);

    measurement.front = range_front;
    measurement.back = range_back;
    measurement.left = range_left;
    measurement.right = range_right;
    measurement.up = range_up;
    measurement.pitch = pitch;
    measurement.roll = roll;
    measurement.yaw = yaw;

    rotate_and_create_points(&measurement, start_point, points);
    DEBUG_PRINT("point_front x:%f,y:%f,z:%f\n", (double)points[0].x, (double)points[0].y, (double)points[0].z);
    DEBUG_PRINT("point_back x:%f,y:%f,z:%f\n", (double)points[1].x, (double)points[1].y, (double)points[1].z);
    DEBUG_PRINT("point_left x:%f,y:%f,z:%f\n", (double)points[2].x, (double)points[2].y, (double)points[2].z);
    DEBUG_PRINT("point_right x:%f,y:%f,z:%f\n\n", (double)points[3].x, (double)points[3].y, (double)points[3].z);
  }
}

coordinate_t rot(float roll, float pitch, float yaw, coordinate_t origin, coordinate_t point)
{
  float cosr = cos((double)roll*M_PI/180);
  float cosp = cos((double)pitch*M_PI/180);
  float cosy = cos((double)yaw*M_PI/180);

  float sinr = sin((double)roll*M_PI/180);
  float sinp = sin((double)pitch*M_PI/180);
  float siny = sin((double)yaw*M_PI/180);

  float roty[3][3];
  float rotp[3][3];
  float rotr[3][3];

  roty[0][0] = cosy;
  roty[0][1] = -siny;
  roty[0][2] = 0;
  roty[1][0] = siny;
  roty[1][1] = cosy;
  roty[1][2] = 0;
  roty[2][0] = 0;
  roty[2][1] = 0;
  roty[2][2] = 1;

  rotp[0][0] = cosp;
  rotp[0][1] = 0;
  rotp[0][2] = sinp;
  rotp[1][0] = 0;
  rotp[1][1] = 1;
  rotp[1][2] = 0;
  rotp[2][0] = -sinp;
  rotp[2][1] = 0;
  rotp[2][2] = cosp;

  rotr[0][0] = 1;
  rotr[0][1] = 0;
  rotr[0][2] = 0;
  rotr[1][0] = 0;
  rotr[1][1] = cosr;
  rotr[1][2] = -sinr;
  rotr[2][0] = 0;
  rotr[2][1] = sinr;
  rotr[2][2] = cosr;
  float tmp[3][1];
  tmp[0][0] = point.x - origin.x;
  tmp[1][0] = point.y - origin.y;
  tmp[2][0] = point.z - origin.z;

  dot(roty,tmp);
  dot(rotp,tmp);
  dot(rotr,tmp);
  coordinate_t tmp2 = {tmp[0][0] + origin.x, tmp[1][0] + origin.y, tmp[2][0] + origin.z};

  determine_threshold(&tmp2);
  return tmp2;
}

void determine_threshold(coordinate_t *point)
{
  point->x = fmax(fmin(point->x, WIDTH / 2), -WIDTH / 2);
  point->y = fmax(fmin(point->y, WIDTH / 2), -WIDTH / 2);
  point->z = fmax(fmin(point->z, WIDTH / 2), -WIDTH / 2);
}

void dot(float A[][3], float B[][1])
{
  float C[3][1];
  for (int i = 0; i < 3; i++)
  {
    C[i][0] = 0;
    for (int k = 0; k < 3; k++)
    {
      C[i][0] += A[i][k] * B[k][0];
    }
  }
  for (int i = 0; i < 3; i++)
  {
    B[i][0] = C[i][0];
  }
}

void rotate_and_create_points(example_measure_t *measurement, coordinate_t start_point, coordinate_t *res)
{
  float pitch = -1 * measurement->pitch;
  float roll = measurement->roll;
  float yaw = measurement->yaw;
  if (measurement->front < SENSOR_TH)
  {
    coordinate_t point = {start_point.x + measurement->front, start_point.y, start_point.z};
    res[0] = rot(roll, pitch, yaw, start_point, point);
  }
  else
  {
    res[0].x = start_point.x;
    res[0].y = start_point.y;
    res[0].z = start_point.z;
  }
  if (measurement->back < SENSOR_TH)
  {
    coordinate_t point = {start_point.x - measurement->back, start_point.y, start_point.z};
    res[1] = rot(roll, pitch, yaw, start_point, point);
  }
  else
  {
    res[1].x = start_point.x;
    res[1].y = start_point.y;
    res[1].z = start_point.z;
  }
  if (measurement->left < SENSOR_TH)
  {
    coordinate_t point = {start_point.x, start_point.y + measurement->left, start_point.z};
    res[2] = rot(roll, pitch, yaw, start_point, point);
  }
  else
  {
    res[2].x = start_point.x;
    res[2].y = start_point.y;
    res[2].z = start_point.z;
  }
  if (measurement->right < SENSOR_TH)
  {
    coordinate_t point = {start_point.x, start_point.y - measurement->right, start_point.z};
    res[3] = rot(roll, pitch, yaw, start_point, point);
  }
  else
  {
    res[3].x = start_point.x;
    res[3].y = start_point.y;
    res[3].z = start_point.z;
  }
}
