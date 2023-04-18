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

#define DEBUG_PRINT_ENABLED 1

coordinate_pair_t requestPayload[CRTP_RECEIVE_PAYLOAD_LENGTH];

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse CRTP packet data
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t reqType = p->data[1];
    // Calculate the sequence number
    uint8_t a = p->data[2];
    uint8_t b = p->data[3];
    uint16_t c = a << 8;
    uint16_t seq = c | b;

    if (reqType == MAPPING_REQ) {
        uint8_t mappingRequestPayloadLength = p->data[4];
        mapping_req_payload_t mappingRequestPayload[mappingRequestPayloadLength];
        memcpy(mappingRequestPayload, &p->data[5], sizeof(mapping_req_payload_t) * mappingRequestPayloadLength);
        DEBUG_PRINT("[Edge-STM32]P2P: Receive mapping request from: %d, RSSI: -%d dBm, seq: %d, payloadLength: %d\n", sourceId, rssi, seq, mappingRequestPayloadLength);
        // print debug info
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[Edge-STM32]Mapping request payload: \n");
            for (int i = 0; i < mappingRequestPayloadLength; i++)
            {
                DEBUG_PRINT("[Edge-STM32]Coordinate pair %d: (%d, %d, %d), (%d, %d, %d), mergedNums: %d\n", 
                    i, 
                    mappingRequestPayload[i].coordinatePair.startPoint.x, 
                    mappingRequestPayload[i].coordinatePair.startPoint.y, 
                    mappingRequestPayload[i].coordinatePair.startPoint.z,
                    mappingRequestPayload[i].coordinatePair.endPoint.x, 
                    mappingRequestPayload[i].coordinatePair.endPoint.y, 
                    mappingRequestPayload[i].coordinatePair.endPoint.z,
                    mappingRequestPayload[i].mergedNums);
                vTaskDelay(50);
            }
        }
        
        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        cpxPacket.dataLength=sizeof(sourceId) + sizeof(reqType) + sizeof(seq) + sizeof(mappingRequestPayloadLength) + sizeof(coordinate_pair_t) * mappingRequestPayloadLength;
        cpxPacket.data[0] = sourceId;
        cpxPacket.data[1] = reqType;
        cpxPacket.data[2] = seq >> 8;
        cpxPacket.data[3] = seq & 0xff;
        cpxPacket.data[4] = mappingRequestPayloadLength;
        memcpy(&cpxPacket.data[5], mappingRequestPayload, cpxPacket.dataLength);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[Edge-STM32]CPX: Forward mapping request %s, from: %d, seq: %d\n", flag == false ? "timeout" : "success", sourceId, seq);
    } else if (reqType == EXPLORE_REQ) {
        explore_req_payload_t exploreRequestPayload;
        memcpy(&exploreRequestPayload, &p->data[4], sizeof(explore_req_payload_t));
        DEBUG_PRINT("[Edge-STM32]P2P: Receive explore request from: %d, RSSI: -%d dBm, seq: %d\n", sourceId, rssi, seq);
        // print debug info
        if (DEBUG_PRINT_ENABLED)
        {
            DEBUG_PRINT("[Edge-STM32]P2P: Explore request payload: \n");
            DEBUG_PRINT("[Edge-STM32]P2P: startPoint: (%d, %d, %d)\n", 
                exploreRequestPayload.startPoint.x, 
                exploreRequestPayload.startPoint.y, 
                exploreRequestPayload.startPoint.z);
            DEBUG_PRINT("[Edge-STM32]P2P: data: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
                (double)exploreRequestPayload.measurement.data[0], 
                (double)exploreRequestPayload.measurement.data[1], 
                (double)exploreRequestPayload.measurement.data[2], 
                (double)exploreRequestPayload.measurement.data[3], 
                (double)exploreRequestPayload.measurement.data[4], 
                (double)exploreRequestPayload.measurement.data[5]);
            DEBUG_PRINT("[Edge-STM32]P2P: roll: %.2f, pitch: %.2f, yaw: %.2f\n", 
                (double)exploreRequestPayload.measurement.roll, 
                (double)exploreRequestPayload.measurement.pitch, 
                (double)exploreRequestPayload.measurement.yaw);
            vTaskDelay(50);
        }

        // Send msg to GAP8
        CPXPacket_t cpxPacket;
        cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpxPacket.route);
        cpxPacket.dataLength=sizeof(sourceId) + sizeof(reqType) + sizeof(seq) + sizeof(explore_req_payload_t);
        cpxPacket.data[0] = sourceId;
        cpxPacket.data[1] = reqType;
        cpxPacket.data[2] = seq >> 8;
        cpxPacket.data[3] = seq & 0xff;
        memcpy(&cpxPacket.data[4], &exploreRequestPayload, cpxPacket.dataLength);
        bool flag = cpxSendPacketBlockingTimeout(&cpxPacket, 1000);
        DEBUG_PRINT("[Edge-STM32]CPX: Forward explore request %s, from: %d, seq: %d\n", flag == false ? "timeout" : "success", sourceId, seq);
    } else {
        DEBUG_PRINT("[Edge-STM32]P2P: Receive unknown packet from: %d, RSSI: -%d dBm, seq: %d, reqType: %d\n", sourceId, rssi, seq, reqType);
    }
}

void CPXForwardInit() {
    p2pRegisterCB(P2PCallbackHandler);
    DEBUG_PRINT("[Edge-STM32]CPX Forward Init...\n");
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}
