#include <string.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "app.h"
#include "range.h"
#include "log.h"

#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"
#define COORDS_LENGTH 5

void appMain()
{
    // Listening for forward packets
    P2PListeningInit();
    DEBUG_PRINT("[STM32-Edge]Listening P2P...\n");
    while(1){
        vTaskDelay(M2T(2000));
    }
}