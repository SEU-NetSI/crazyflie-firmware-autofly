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

coordinate_pair_t requestPayload[CRTP_RECEIVE_PAYLOAD_LENGTH];

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse CRTP packet data
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t reqType = p->data[1];
    // Calculate the sequence number
    uint8_t a = p[2];
    uint8_t b = p[3];
    uint8_t c = a << 8;
    uint16_t seq = c | b;

    if (reqType == MAPPING_REQ) {
        uint8_t mappingRequestPayloadLength = p->data[4];
        coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
        memcpy(mappingRequestPayload, &p->data[5], sizeof(coordinate_pair_t) * mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]Receive P2P mapping request from: %d, RSSI: -%d dBm, seq: %d, payloadLength: %d\n", sourceId, rssi, seq, mappingRequestPayloadLength);
        DEBUG_PRINT("[STM32-Edge]First coordinate pair: (%d, %d, %d), (%d, %d, %d)\n",
            mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
            mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);
        
        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        cpxPacket.dataLength=sizeof(sourceId) + sizeof(reqType) + sizeof(seq) + sizeof(mappingRequestPayloadLength) + sizeof(coordinate_pair_t) * mappingRequestPayloadLength;
        cpxPacket.data[0] = sourceId;
        cpxPacket.data[1] = reqType;
        packet.data[2] = seq >> 8;
        packet.data[3] = seq & 0xff;
        cpxPacket.data[4] = mappingRequestPayloadLength;
        memcpy(&cpxPacket.data[5], mappingRequestPayload, cpxPacket.dataLength);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[STM32-Edge]CPX Forward mapping request %s, from: %d, seq: %d\n", flag == false ? "timeout" : "success", sourceId, seq);
    } else {
        DEBUG_PRINT("[STM32-Edge]Receive P2P other request from: %d, RSSI: -%d dBm, seq: %d, reqType: %d\n", sourceId, rssi, seq, reqType);
    }
}

void CPXForwardInit() {
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[STM32-Edge]CPX Forward Init...\n");
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}
