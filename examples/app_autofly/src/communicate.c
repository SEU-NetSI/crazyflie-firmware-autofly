#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#include"radiolink.h"
#include"octoMap.h"
#include "cpx_internal_router.h"
#include "cpx_external_router.h"
#include "communicate.h"
#define COORDS_LENGTH 5

void MtrP2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    DEBUG_PRINT("Callback called!");
    uint8_t other_id = p->data[0];
    static coordinate_t msg[COORDS_LENGTH];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*COORDS_LENGTH);
    uint8_t rssi = p->rssi;
    //TODO Listened Msg Process for Mtr
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d), Sent to Ad\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
}

void AdP2PCallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    uint8_t other_id = p->data[0];
    static coordinate_t msg[COORDS_LENGTH];
    memcpy(msg, &p->data[1], sizeof(coordinate_t)*COORDS_LENGTH);
    uint8_t rssi = p->rssi;
    DEBUG_PRINT("[RSSI: -%d dBm] P2PMsg from:%d,Point1: (%d,%d,%d) Sending to AD...\n", rssi, other_id, msg[0].x,msg[0].y,msg[0].z);
    //Send msg to GAP8
    CPXPacket_t cpxPacket;
    cpxInitRoute(CPX_T_STM32,CPX_T_GAP8,CPX_F_APP,&cpxPacket.route);
    cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
    memcpy(cpxPacket.data, msg, cpxPacket.dataLength);
    bool flag= cpxSendPacketBlockingTimeout(&cpxPacket,1000);
    DEBUG_PRINT("Send %s\n",flag==false?"timeout":"success");
}

void MtrP2PListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(MtrP2PCallbackHandler);
}

void AdP2PListeningInit(){
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(AdP2PCallbackHandler);
    cpxInternalRouterInit();
    cpxExternalRouterInit();
}

bool SendCoords(coordinate_t* coords){

    // Initialize the p2p packet
    static P2PPacket packet;
    packet.port=0x00;

    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    packet.data[0]=my_id;

    memcpy(&packet.data[1], coords, sizeof(coordinate_t)*COORDS_LENGTH);

    // Set the size, which is the amount of bytes the payload with ID and the string
    packet.size=sizeof(coordinate_t)*COORDS_LENGTH+1;
    // Send the P2P packet
    DEBUG_PRINT("P2P Msg Sent by:%d, First Coord is: (%d,%d,%d)\n",my_id,coords[0].x,coords[0].y,coords[0].z);
    return radiolinkSendP2PPacketBroadcast(&packet);
}

bool SendReq(coordinate_t* coords) {
    //TODO Req send and differ packets
    return false;
}