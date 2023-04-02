#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"

#include <string.h>
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"
#define COORDS_LENGTH 5

void appMain()
{
    //For receiving
    DEBUG_PRINT("[P2P] Listening...\n");
    P2PListeningInit();
    while(1){
        vTaskDelay(M2T(2000));
    }
}