#include "ws_server.h"
#include "config.h"
#include "../data/json_builder.h"

#include <Arduino.h>
#include <WebSocketsServer.h>

static WebSocketsServer sWs(IMU_WS_PORT);
static uint16_t sClientCount = 0;

static void onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    (void)payload; (void)length;
    switch (type) {
        case WStype_CONNECTED:
            sClientCount = sWs.connectedClients();
            Serial.printf("[IMU-WS] Client %u connected (%u total)\n", num, sClientCount);
            break;
        case WStype_DISCONNECTED:
            sClientCount = sWs.connectedClients();
            Serial.printf("[IMU-WS] Client %u disconnected (%u total)\n", num, sClientCount);
            break;
        default: break;
    }
}

void ws_init() {
    sWs.begin();
    sWs.onEvent(onEvent);
}

void ws_loop() { sWs.loop(); }
uint16_t ws_client_count() { return sClientCount; }

void ws_broadcast_imu() {
    if (sClientCount == 0) return;
    char buf[768];
    size_t len = json_build_imu(buf, sizeof(buf));
    if (len > 0 && len < sizeof(buf)) {
        sWs.broadcastTXT(buf);
    }
}
