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
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("Callback called!");
    uint8_t other_id = p->data[0];
    uint8_t reqType = p->data[1];
    static coordinate_t msg[5];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*reqType);
    uint8_t rssi = p->rssi;
    //TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d), Sent to Ad\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
}

void ListeningInit()
{
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(P2PCallbackHandler);
}

bool sendMappingRequest(coordinate_pair_t* mappingRequestPayloadPtr, uint8_t mappingRequestPayloadLength, uint16_t mappingRequestSeq) 
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)MAPPING_REQ;
    packet.data[2] = mappingRequestSeq;
    packet.data[3] = mappingRequestPayloadLength;
    memcpy(&packet.data[4], mappingRequestPayloadPtr, sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
    // 1b for sourceId, 2b for mappingRequestSeq, 1b for mappingRequestPayloadLength, 12b for each coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof((uint8_t)MAPPING_REQ) + sizeof(mappingRequestSeq) + sizeof(mappingRequestPayloadLength) + sizeof(coordinate_pair_t)*mappingRequestPayloadLength;
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendExploreRequest(coordinate_t* exploreRequestPayloadPtr, uint16_t exploreRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)EXPLORE_REQ;
    packet.data[2] = exploreRequestSeq;
    memcpy(&packet.data[3], exploreRequestPayloadPtr, sizeof(coordinate_t));
    // 1b for sourceId, 2b for exploreRequestSeq, 6b for coordinate_t
    packet.size = sizeof(sourceId) + sizeof(exploreRequestSeq) + sizeof(coordinate_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool sendPathRequest(coordinate_pair_t* pathRequestPayloadPtr, uint16_t pathRequestSeq)
{
    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port = 0x00;
    // Get the current crazyflie id
    uint64_t address = configblockGetRadioAddress();
    uint8_t sourceId = (uint8_t)((address) & 0x00000000ff);
    // Assemble the packet
    packet.data[0] = sourceId;
    packet.data[1] = (uint8_t)PATH_REQ;
    packet.data[2] = pathRequestSeq;
    memcpy(&packet.data[3], pathRequestPayloadPtr, sizeof(coordinate_pair_t));
    // 1b for sourceId, 2b for pathRequestSeq, 12b for coordinate_pair_t
    packet.size = sizeof(sourceId) + sizeof(pathRequestSeq) + sizeof(coordinate_pair_t);
    // Send the P2P packet
    return radiolinkSendP2PPacketBroadcast(&packet);
}
