#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"

#include "communicate.h"

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse p2p request packet from a lidar uav
    uint8_t uavId = p->data[0];
    uint8_t reqType = p->data[1];
    uint16_t seq = p->data[2];
    uint8_t rssi = p->rssi;

    if (reqType == MAPPING_REQ) {
        uint8_t mappingRequestPayloadLength = p->data[3];
        coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
        memcpy(mappingRequestPayload, &p->data[4], sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]Receive P2P mapping request from: %d, RSSI: -%d dBm, seq: %d, payloadLength: %d\n", uavId, rssi, seq, mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]First coordinate pair: (%d, %d, %d), (%d, %d, %d)\n",
            mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
            mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);
    } else {
        DEBUG_PRINT("[STM32-Edge]Receive P2P other request from: %d, RSSI: -%d dBm, seq: %d, reqType: %d\n", uavId, rssi, seq, reqType);
    }


    // static coordinate_t msg[5];
    // memcpy(msg, &p->data[3], sizeof(coordinate_t)*reqType);
    // uint8_t rssi = p->rssi;
    // DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d, reqType:%d, Seq:%d,Point1: (%d,%d,%d) Sending to AD...\n", rssi,other_id, reqType, seq,msg[0].x,msg[0].y,msg[0].z);
    // //Send msg to GAP8
    // CPXPacket_t cpxPacket;
    // cpxInitRoute(CPX_T_STM32,CPX_T_GAP8,CPX_F_APP,&cpxPacket.route);
    // cpxPacket.dataLength=sizeof(coordinate_t)*reqType+2*sizeof(uint8_t);
    // cpxPacket.data[0]=other_id;
    // cpxPacket.data[1]=reqType;
    // memcpy(&cpxPacket.data[2], msg, cpxPacket.dataLength);
    // bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
    // DEBUG_PRINT("Send %s\n",flag==false?"timeout":"success");
}

void P2PListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[STM32-Edge]P2P Listening Init...\n");
    // cpxInternalRouterInit();
    // cpxExternalRouterInit();
}
