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

uint8_t getSourceId()
{
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    return sourceId;
}

void P2PCallbackHandler(P2PPacket *p)
{
    // Parse the P2P packet
    uint8_t rssi = p->rssi;
    uint8_t sourceId = p->data[0];
    uint8_t destinationId = p->data[1];
    uint8_t respType = p->data[2];
    // Calculate the sequence number
    uint8_t a = p->data[3];
    uint8_t b = p->data[4];
    uint16_t c = a << 8;
    uint16_t seq = c | b;

    if (destinationId != getSourceId() || respType != EXPLORE_RESP)
    {
        return;
    }

    DEBUG_PRINT("[LiDAR-STM32]P2P: Receive explore response from: %d, RSSI: -%d dBm, respType: %d, seq: %d\n", sourceId, rssi, respType, seq);
    static explore_resp_payload_t responsePayload;
    memcpy(&responsePayload, &p->data[5], sizeof(explore_resp_payload_t));
    
    // TODO
    // use crtpCommanderHighLevelGoTo to move
    // use xQueueSend to notify the main task to send the next explore request
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool sendMappingRequest(mapping_req_payload_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = DESTINATION_ID;
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = destinationId;
    packet.data[2] = (uint8_t)MAPPING_REQ;
    packet.data[3] = mappingRequestSeq >> 8;
    packet.data[4] = mappingRequestSeq & 0xff;
    packet.data[5] = mappingRequestPayloadLength;
    memcpy(&packet.data[6], mappingRequestPayloadPtr, sizeof(mapping_req_payload_t)*mappingRequestPayloadLength);
    // 1b for sourceId, 2b for mappingRequestSeq, 1b for mappingRequestPayloadLength, 12b for each coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof((uint8_t)MAPPING_REQ) + sizeof(mappingRequestSeq) 
        + sizeof(mappingRequestPayloadLength) + sizeof(mapping_req_payload_t)*mappingRequestPayloadLength;
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendExploreRequest(explore_req_payload_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    uint8_t sourceId = getSourceId();
    uint8_t destinationId = DESTINATION_ID;
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = destinationId;
    packet.data[2] = (uint8_t)EXPLORE_REQ;
    packet.data[3] = exploreRequestSeq >> 8;
    packet.data[4] = exploreRequestSeq & 0xff;
    memcpy(&packet.data[4], exploreRequestPayloadPtr, sizeof(explore_req_payload_t));
    // 1b for sourceId, 2b for exploreRequestSeq, 6b for coordinate_t
    packet.size = sizeof(sourceId) + sizeof(exploreRequestSeq) + sizeof(explore_req_payload_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
