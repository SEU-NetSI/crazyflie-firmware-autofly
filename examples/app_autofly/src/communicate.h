#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
void MtrP2PCallbackHandler(P2PPacket *p);
void AdP2PCallbackHandler(P2PPacket *p);
void MtrP2PListeningInit();
void AdP2PListeningInit();
bool SendCoords(coordinate_t* coords);
bool SendReq(coordinate_t* coords);
#endif