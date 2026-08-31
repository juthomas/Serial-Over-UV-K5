/* Serial Bridge — UART <-> FSK / DTMF RF tunnel for UV-K5 (egzumer-based) */

#ifndef APP_SERIAL_BRIDGE_H
#define APP_SERIAL_BRIDGE_H

#ifdef ENABLE_SERIAL_BRIDGE

#include <stdbool.h>
#include <stdint.h>

#include "driver/keyboard.h"

#define SERIAL_BRIDGE_PAYLOAD_MAX 56u
#define SERIAL_BRIDGE_DTMF_PAYLOAD_MAX 8u
#define SERIAL_BRIDGE_MORSE_PAYLOAD_MAX 16u
#define SERIAL_BRIDGE_DEFAULT_FREQ 43300000u  /* 433.000 MHz (10 Hz units) */

#ifndef SERIAL_BRIDGE_DEFAULT_MODE
#define SERIAL_BRIDGE_DEFAULT_MODE 0  /* 0 = FSK, 1 = data, 2 = Morse */
#endif

extern uint16_t gSerialBridgeFSKBuffer[36];
extern uint16_t gSerialBridgeTxPackets;
extern uint16_t gSerialBridgeRxPackets;
extern uint16_t gSerialBridgeRxErrors;
extern uint16_t gSerialBridgeTxBytes;
extern uint16_t gSerialBridgeRxBytes;
extern bool     gSerialBridgeBusyTx;

void SERIAL_BRIDGE_Start(void);
void SERIAL_BRIDGE_Stop(void);
void SERIAL_BRIDGE_ReArm(void);
void SERIAL_BRIDGE_ApplyDecoder(void);
void SERIAL_BRIDGE_LeaveFsk(void);
void SERIAL_BRIDGE_EnterAnalogRx(void);
void SERIAL_BRIDGE_HoldAfterTx(void);
void SERIAL_BRIDGE_TimeSlice10ms(void);
void SERIAL_BRIDGE_StorePacket(void);
void SERIAL_BRIDGE_OnDtmf(char c);
void SERIAL_BRIDGE_OnSymbol(uint8_t code);
void SERIAL_BRIDGE_OnCwHit(uint8_t code);
void SERIAL_BRIDGE_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld);
bool SERIAL_BRIDGE_IsActive(void);
bool SERIAL_BRIDGE_IsAnalogRx(void);
bool SERIAL_BRIDGE_IsFsk(void);
bool SERIAL_BRIDGE_IsDtmf(void);
bool SERIAL_BRIDGE_IsMorse(void);
bool SERIAL_BRIDGE_UsesSelCall(void);
const char *SERIAL_BRIDGE_ModeName(void);
const char *SERIAL_BRIDGE_NextModeName(void);
const char *SERIAL_BRIDGE_ScaleName(void);
const char *SERIAL_BRIDGE_NextScaleName(void);
const char *SERIAL_BRIDGE_NextCwToneName(void);
const char *SERIAL_BRIDGE_TempoLabel(void);
bool SERIAL_BRIDGE_HoldDataRx(void);
bool SERIAL_BRIDGE_BlockAnalogPtt(void);

#endif /* ENABLE_SERIAL_BRIDGE */

#endif /* APP_SERIAL_BRIDGE_H */
