/*
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: CC0-1.0
 *
 * Daly BMS BLE Client + WiFi STA — ESP-IDF v5.x
 * Bluetooth y WiFi en coexistencia simultánea
 */

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cctype>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "NimBLEDevice.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "mdns.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "nvs.h"
#include "esp_system.h"
#include "lwip/sockets.h"

static const char *TAG     = "DALY_BLE";
static const char *TAG_WIFI = "WIFI";

/* ---- WiFi STA ---- */
#define WIFI_RETRY_MAX   10

/* ---- AP provisioning ---- */
#define AP_SSID          "DALY-BMS-Setup"
#define AP_CHANNEL       1
#define AP_MAX_CONN      4

/* ---- NVS keys para credenciales WiFi ---- */
#define NVS_WIFI_NS      "wifi_cfg"
#define NVS_WIFI_SSID    "ssid"
#define NVS_WIFI_PASS    "pass"

/* bits del EventGroup WiFi */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = nullptr;
static int                s_wifi_retry_count  = 0;

#define BMS_MAC_ADDRESS   "50:18:12:01:1b:68"  /* dirección MAC del BMS (en minúsculas) */
#define SERVICE_UUID_MAIN "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHAR_UUID_TX      "0000fff1-0000-1000-8000-00805f9b34fb"  /* NOTIFY BMS→Host */
#define CHAR_UUID_RX      "0000fff2-0000-1000-8000-00805f9b34fb"  /* WRITE  Host→BMS */

/* ---- Push remoto a Hostalia ---- */
#define REMOTE_PUSH_URL   "https://luaresearch.es/bms/update.php"  /* cambiar por tu dominio */
#define REMOTE_API_KEY    "c864645245ca1dcd5d6a0d6ca3754512c26a7a07ff722c853d1587b18bf1916f"              /* igual que en update.php */
#define REMOTE_PUSH_MS    5000                                     /* intervalo push en ms */
//-------------------------------------------------------------------
/* ---- sustitutos de millis() / delay() de Arduino ---- */
static inline uint64_t millis(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}
//-------------------------------------------------------------------
static inline void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
//-------------------------------------------------------------------
/* ---- estado global (idéntico al original) ---- */
static int      expectedCellCount    = 12;
static int      expectedFramesNeeded = (12 + 2) / 3;

static float    cellVoltages[48];
static uint16_t cellRawMv[48];
static bool     frameReceived[16];
static int      framesReceivedCount  = 0;
static bool     assembled            = false;

static bool frameBaseDetected = false;
static int  frameBase         = 0;

static uint8_t rxStreamBuf[1024];
static size_t  rxStreamLen = 0;

static NimBLEClient               *pClient           = nullptr;
static NimBLERemoteCharacteristic *pTxCharacteristic = nullptr;
static NimBLERemoteCharacteristic *pRxCharacteristic = nullptr;
static bool connected = false;
static bool scanning  = false;

struct BMSDeviceInfo {
    NimBLEAddress address;
    std::string   name;
    int           rssi;
    bool          found;
} bmsInfo;

/* ---- Estado BMS compartido entre tareas (mutex protegido) ---- */
struct BmsState {
    float    voltage;        /* V    0x90 */
    float    current;        /* A         */
    float    soc;            /* %         */
    float    max_cell_v;     /* V    0x91 */
    uint8_t  max_cell_n;
    float    min_cell_v;     /* V         */
    uint8_t  min_cell_n;
    int      diff_mv;        /* mV        */
    float    max_temp;       /* °C   0x92 */
    uint8_t  max_temp_s;
    float    min_temp;       /* °C        */
    uint8_t  min_temp_s;
    uint8_t  charge_mode;    /* 0/1/2 0x93 */
    bool     charge_mos;
    bool     discharge_mos;
    uint32_t remaining_mah;
    uint8_t  cell_count;     /* 0x94 */
    uint8_t  temp_count;
    bool     charger;
    bool     load;
    uint16_t cycles;
    float    cell_v[48];     /* V    0x95 */
    uint16_t cell_mv[48];
    bool     balance[48];    /* 0x97 */
    float    cell_temps[16]; /* °C   0x96 — temperaturas individuales por sensor */
    uint32_t alarm_flags;    /* bitmask 0x98 — flags de alarma activos */
    bool     ble_connected;
    char     wifi_ip[16];
    uint16_t valid;          /* bitmask: bit0=0x90 … bit7=0x96, bit8=0x98 */
};
static BmsState          s_bms       = {};
static SemaphoreHandle_t s_bms_mutex = nullptr;

/* ---- HTTP / WebSocket globals ---- */
#define WS_MAX_CLIENTS 4
static httpd_handle_t    s_httpd       = nullptr;
static int               s_ws_fds[WS_MAX_CLIENTS];
static volatile bool     s_trigger_scan = false;
//-------------------------------------------------------------------
/* ------------------------------------------------------------------ */
static bool verifyChecksum(const uint8_t *pkt, size_t len)
{
    if (len < 2) return false;
    uint8_t sum = 0;
    for (size_t i = 0; i < len - 1; ++i) sum += pkt[i];
    return (sum == pkt[len - 1]);
}
//-------------------------------------------------------------------
static void recomputeExpectedFrames(void)
{
    if (expectedCellCount < 1)  expectedCellCount = 12;
    if (expectedCellCount > 48) expectedCellCount = 48;
    expectedFramesNeeded = (expectedCellCount + 2) / 3;
}
//-------------------------------------------------------------------
static void resetCellAssembly(void)
{
    memset(cellVoltages,  0, sizeof(cellVoltages));
    memset(cellRawMv,     0, sizeof(cellRawMv));
    memset(frameReceived, 0, sizeof(frameReceived));
    framesReceivedCount = 0;
    assembled = false;
}
//-------------------------------------------------------------------
/* ---- parser de trama DALY de 13 bytes ---- */
static void parseFrame(const uint8_t *frame)
{
    if (frame[0] != 0xA5) return;
    if (!verifyChecksum(frame, 13)) return;

    uint8_t        cmd     = frame[2];
    const uint8_t *payload = frame + 4;

    switch (cmd) {
        case 0x90: {
            uint16_t total_raw   = (payload[0] << 8) | payload[1];
            uint16_t current_raw = (payload[4] << 8) | payload[5];
            uint16_t soc_raw     = (payload[6] << 8) | payload[7];
            float v   = total_raw / 10.0f;
            float i   = ((int)current_raw - 30000) / 10.0f;
            float soc = soc_raw / 10.0f;
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.voltage = v;  s_bms.current = i;  s_bms.soc = soc;
            s_bms.valid  |= (1u << 0);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x90 V=%.1f I=%.1f SOC=%.1f", v, i, soc);
            break;
        }
        case 0x91: {
            uint16_t max_mv = (payload[0] << 8) | payload[1];
            uint8_t  max_no = payload[2];
            uint16_t min_mv = (payload[3] << 8) | payload[4];
            uint8_t  min_no = payload[5];
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.max_cell_v = max_mv / 1000.0f;  s_bms.max_cell_n = max_no;
            s_bms.min_cell_v = min_mv / 1000.0f;  s_bms.min_cell_n = min_no;
            s_bms.diff_mv    = (int)max_mv - (int)min_mv;
            s_bms.valid     |= (1u << 1);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x91 max=%.3fV@%d min=%.3fV@%d diff=%dmV",
                     max_mv/1000.0f, max_no, min_mv/1000.0f, min_no, (int)max_mv-(int)min_mv);
            break;
        }
        case 0x92: {
            float   max_t  = payload[0] - 40.0f;
            uint8_t max_no = payload[1];
            float   min_t  = payload[2] - 40.0f;
            uint8_t min_no = payload[3];
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.max_temp = max_t;  s_bms.max_temp_s = max_no;
            s_bms.min_temp = min_t;  s_bms.min_temp_s = min_no;
            s_bms.valid   |= (1u << 2);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x92 max=%.1fC@%d min=%.1fC@%d", max_t, max_no, min_t, min_no);
            break;
        }
        case 0x93: {
            uint8_t  state        = payload[0];
            uint8_t  chargeMOS    = payload[1];
            uint8_t  dischargeMOS = payload[2];
            uint32_t remCap =
                ((uint32_t)payload[4] << 24) | ((uint32_t)payload[5] << 16) |
                ((uint32_t)payload[6] <<  8) |  payload[7];
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.charge_mode   = state;
            s_bms.charge_mos    = (chargeMOS    == 1);
            s_bms.discharge_mos = (dischargeMOS == 1);
            s_bms.remaining_mah = remCap;
            s_bms.valid        |= (1u << 3);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x93 mode=%d cMOS=%d dMOS=%d rem=%lu",
                     state, chargeMOS, dischargeMOS, (unsigned long)remCap);
            break;
        }
        case 0x94: {
            uint8_t  batteryStrings = payload[0];
            uint8_t  tempCount      = payload[1];
            uint8_t  chargerStatus  = payload[2];
            uint8_t  loadStatus     = payload[3];
            uint16_t cycles         = (payload[5] << 8) | payload[6];
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.cell_count = batteryStrings;  s_bms.temp_count = tempCount;
            s_bms.charger    = (chargerStatus == 1);
            s_bms.load       = (loadStatus    == 1);
            s_bms.cycles     = cycles;
            s_bms.valid     |= (1u << 4);
            xSemaphoreGive(s_bms_mutex);
            if (batteryStrings >= 1 && batteryStrings <= 48) {
                expectedCellCount = batteryStrings;
                recomputeExpectedFrames();
                resetCellAssembly();
            }
            ESP_LOGD(TAG, "0x94 cells=%d temps=%d cycles=%d",
                     batteryStrings, tempCount, cycles);
            break;
        }
        case 0x95: {
            uint8_t rawFrameNo = payload[0];
            if (!frameBaseDetected) {
                frameBase         = (rawFrameNo == 0) ? 0 : 1;
                frameBaseDetected = true;
            }
            int idx = (int)rawFrameNo - frameBase;
            if (idx < 0 || idx >= 16) break;
            if (assembled)          break;
            if (frameReceived[idx]) break;
            for (int i = 0; i < 3; ++i) {
                int      p         = 1 + i * 2;
                uint16_t mv        = (payload[p] << 8) | payload[p + 1];
                int      cellIndex = idx * 3 + i;
                if (cellIndex < expectedCellCount) {
                    if (mv == 0xFFFF || mv == 0x0000) {
                        cellRawMv[cellIndex] = 0;
                    } else {
                        cellRawMv[cellIndex]    = mv;
                        cellVoltages[cellIndex] = mv / 1000.0f;
                    }
                }
            }
            frameReceived[idx] = true;
            framesReceivedCount++;
            bool allGot = true;
            for (int f = 0; f < expectedFramesNeeded; ++f)
                if (!frameReceived[f]) { allGot = false; break; }
            if (allGot) {
                xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
                memcpy(s_bms.cell_mv, cellRawMv,    sizeof(uint16_t) * (size_t)expectedCellCount);
                memcpy(s_bms.cell_v,  cellVoltages, sizeof(float)    * (size_t)expectedCellCount);
                s_bms.valid |= (1u << 5);
                xSemaphoreGive(s_bms_mutex);
                assembled = true;
                ESP_LOGD(TAG, "0x95 %d celdas completas", expectedCellCount);
            }
            break;
        }
        case 0x97: {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            for (int i = 0; i < expectedCellCount; ++i)
                s_bms.balance[i] = (payload[i/8] >> (i%8)) & 0x01;
            s_bms.valid |= (1u << 6);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x97 balance actualizado");
            break;
        }
        case 0x96: {
            /* Temperaturas individuales: 7 sensores por trama, frame_number 1-based */
            uint8_t frameNo = payload[0];
            if (frameNo == 0) break;
            int base = ((int)frameNo - 1) * 7;
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            int tCount = (s_bms.temp_count > 0) ? (int)s_bms.temp_count : 8;
            if (tCount > 16) tCount = 16;
            for (int i = 0; i < 7; ++i) {
                int idx = base + i;
                if (idx >= tCount || idx >= 16) break;
                uint8_t raw = payload[1 + i];
                s_bms.cell_temps[idx] = (raw >= 1) ? (raw - 40.0f) : 0.0f;
            }
            s_bms.valid |= (1u << 7);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x96 frame=%d temps individuales", frameNo);
            break;
        }
        case 0x98: {
            /* Flags de alarma: payload[0]=byte alto, payload[1]=byte bajo alarmas */
            /* byte0 bit7=cell_overvolt, 6=cell_undervolt, 5=pack_overvolt,        */
            /* 4=pack_undervolt, 3=chg_overtemp, 2=chg_undertemp,                  */
            /* 1=dis_overtemp, 0=dis_undertemp                                     */
            /* byte1 bit7=chg_overcurrent, 6=dis_overcurrent, 5=short_circuit,     */
            /* 4=ic_error, 3=mos_lock                                              */
            uint32_t flags = (uint32_t)payload[0]
                           | ((uint32_t)payload[1] << 8)
                           | ((uint32_t)payload[2] << 16)
                           | ((uint32_t)payload[3] << 24);
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.alarm_flags = flags;
            s_bms.valid |= (1u << 8);
            xSemaphoreGive(s_bms_mutex);
            ESP_LOGD(TAG, "0x98 alarm_flags=0x%08lX", (unsigned long)flags);
            break;
        }
        default:
            break;
    }
}
//-------------------------------------------------------------------
/* ---- procesador del stream RX ---- */
static void processRxStream(void)
{
    while (rxStreamLen >= 13) {
        if (rxStreamBuf[0] != 0xA5) {
            size_t pos = 0;
            while (pos < rxStreamLen && rxStreamBuf[pos] != 0xA5) ++pos;
            if (pos == rxStreamLen) { rxStreamLen = 0; return; }
            if (pos > 0) {
                memmove(rxStreamBuf, rxStreamBuf + pos, rxStreamLen - pos);
                rxStreamLen -= pos;
            }
            if (rxStreamLen < 13) return;
        }
        uint8_t frame[13];
        memcpy(frame, rxStreamBuf, 13);
        memmove(rxStreamBuf, rxStreamBuf + 13, rxStreamLen - 13);
        rxStreamLen -= 13;
        parseFrame(frame);
    }
}
//-------------------------------------------------------------------
/* ---- callback de notificaciones BLE ---- */
static void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic,
                     uint8_t *pData, size_t length, bool isNotify)
{
    if (rxStreamLen + length > sizeof(rxStreamBuf)) {
        rxStreamLen = 0;
    }
    memcpy(rxStreamBuf + rxStreamLen, pData, length);
    rxStreamLen += length;
    processRxStream();
}
//-------------------------------------------------------------------
/* ---- callbacks de escaneo BLE ---- */
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override
    {
        std::string addr = dev->getAddress().toString();
        std::transform(addr.begin(), addr.end(), addr.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        if (addr == BMS_MAC_ADDRESS) {
            bmsInfo.address = dev->getAddress();
            bmsInfo.name    = dev->haveName() ? dev->getName() : "";
            bmsInfo.rssi    = dev->getRSSI();
            bmsInfo.found   = true;
            if (scanning) {
                NimBLEDevice::getScan()->stop();
                scanning = false;
            }
        }
    }
};
//-------------------------------------------------------------------
/* ---- callbacks del cliente BLE ---- */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pClient) override
    {
        connected = true;
        resetCellAssembly();
        rxStreamLen       = 0;
        frameBaseDetected = false;
        if (s_bms_mutex) {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.ble_connected = true;
            xSemaphoreGive(s_bms_mutex);
        }
    }
    void onDisconnect(NimBLEClient *pClient, int reason) override
    {
        connected         = false;
        pRxCharacteristic = nullptr;
        pTxCharacteristic = nullptr;
        resetCellAssembly();
        rxStreamLen = 0;
        if (s_bms_mutex) {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            s_bms.ble_connected = false;
            xSemaphoreGive(s_bms_mutex);
        }
        if (::pClient) {
            NimBLEDevice::deleteClient(::pClient);
            ::pClient = nullptr;
        }
    }
};
//-------------------------------------------------------------------
/* ---- funciones de comunicación con el BMS ---- */
static void scanForBMS(int durationSeconds)
{
    bmsInfo.found = false;
    scanning      = true;

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new MyAdvertisedDeviceCallbacks());
    scan->setActiveScan(true);
    scan->setInterval(80);
    scan->setWindow(60);
    scan->setMaxResults(0);
    scan->start((uint32_t)durationSeconds * 1000, false);

    uint64_t start = millis();
    while (millis() - start < (uint64_t)durationSeconds * 1000 + 1000) {
        if (!scanning) break;
        delay(100);
    }
    if (scanning) {
        scan->stop();
        scanning = false;
    }
}
//-------------------------------------------------------------------
static void sendDalyRequest(uint8_t cmd, const uint8_t payload8[8] = nullptr)
{
    if (!pRxCharacteristic || !connected) return;
    if (cmd == 0x95) {
        resetCellAssembly();
        frameBaseDetected = false;
    }

    uint8_t pkt[13];
    pkt[0] = 0xA5;
    pkt[1] = 0x40;
    pkt[2] = cmd;
    pkt[3] = 0x08;
    for (int i = 0; i < 8; ++i) pkt[4 + i] = payload8 ? payload8[i] : 0x00;
    uint8_t sum = 0;
    for (int i = 0; i < 12; ++i) sum += pkt[i];
    pkt[12] = sum;

    if (pRxCharacteristic->canWrite()) {
        pRxCharacteristic->writeValue(pkt, 13, true);
    }
}
//-------------------------------------------------------------------
static bool discoverAndSubscribe(void)
{
    if (!pClient || !pClient->isConnected()) return false;
    NimBLERemoteService *svc = pClient->getService(SERVICE_UUID_MAIN);
    if (!svc) return false;
    pTxCharacteristic = svc->getCharacteristic(CHAR_UUID_TX);
    pRxCharacteristic = svc->getCharacteristic(CHAR_UUID_RX);
    if (!pTxCharacteristic || !pRxCharacteristic) return false;
    if (pTxCharacteristic->canNotify()) {
        pTxCharacteristic->subscribe(true, notifyCB);
    }
    return true;
}
//-------------------------------------------------------------------
static bool connectToBMS(void)
{
    if (!bmsInfo.found) return false;
    if (pClient) { NimBLEDevice::deleteClient(pClient); pClient = nullptr; }
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());
    pClient->setConnectTimeout(5000);
    if (!pClient->connect(bmsInfo.address)) {
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }
    if (!discoverAndSubscribe()) {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }
    return true;
}
//-------------------------------------------------------------------
static void cleanupClient(void)
{
    if (pClient) { NimBLEDevice::deleteClient(pClient); pClient = nullptr; }
    pRxCharacteristic = nullptr;
    pTxCharacteristic = nullptr;
}
//-------------------------------------------------------------------
/* ------------------------------------------------------------------ */
/* ---- HTTP / WebSocket server                                   ---- */
/* ------------------------------------------------------------------ */

extern const uint8_t bms_ui_html_start[] asm("_binary_bms_ui_html_start");
extern const uint8_t bms_ui_html_end[]   asm("_binary_bms_ui_html_end");

/* ---- construir JSON de estado (el caller debe free()) ---- */
static char *build_status_json(void)
{
    if (!s_bms_mutex) return nullptr;
    cJSON *root = cJSON_CreateObject();
    if (!root) return nullptr;
    cJSON_AddStringToObject(root, "t", "status");

    xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
    BmsState snap = s_bms;   /* copia local bajo mutex */
    xSemaphoreGive(s_bms_mutex);

    cJSON *ble = cJSON_AddObjectToObject(root, "ble");
    cJSON_AddBoolToObject(ble, "connected", snap.ble_connected);

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddStringToObject(wifi, "ip", snap.wifi_ip[0] ? snap.wifi_ip : "--");

    if (snap.valid & (1u << 0)) {
        cJSON *b = cJSON_AddObjectToObject(root, "basic");
        cJSON_AddNumberToObject(b, "v",   (double)snap.voltage);
        cJSON_AddNumberToObject(b, "i",   (double)snap.current);
        cJSON_AddNumberToObject(b, "soc", (double)snap.soc);
    }
    if (snap.valid & (1u << 1)) {
        cJSON *mm = cJSON_AddObjectToObject(root, "minmax");
        cJSON_AddNumberToObject(mm, "max_v",   (double)snap.max_cell_v);
        cJSON_AddNumberToObject(mm, "max_n",   snap.max_cell_n);
        cJSON_AddNumberToObject(mm, "min_v",   (double)snap.min_cell_v);
        cJSON_AddNumberToObject(mm, "min_n",   snap.min_cell_n);
        cJSON_AddNumberToObject(mm, "diff_mv", snap.diff_mv);
    }
    if (snap.valid & (1u << 2)) {
        cJSON *t = cJSON_AddObjectToObject(root, "temp");
        cJSON_AddNumberToObject(t, "max",   (double)snap.max_temp);
        cJSON_AddNumberToObject(t, "max_s", snap.max_temp_s);
        cJSON_AddNumberToObject(t, "min",   (double)snap.min_temp);
        cJSON_AddNumberToObject(t, "min_s", snap.min_temp_s);
    }
    if (snap.valid & (1u << 3)) {
        static const char *modes[] = {"Stationary", "Charging", "Discharging"};
        cJSON *s = cJSON_AddObjectToObject(root, "state");
        cJSON_AddNumberToObject(s, "mode",     snap.charge_mode);
        cJSON_AddStringToObject(s, "mode_str", snap.charge_mode < 3
                                               ? modes[snap.charge_mode] : "Unknown");
        cJSON_AddBoolToObject(s,   "c_mos",    snap.charge_mos);
        cJSON_AddBoolToObject(s,   "d_mos",    snap.discharge_mos);
        cJSON_AddNumberToObject(s, "rem_mah",  (double)snap.remaining_mah);
    }
    if (snap.valid & (1u << 4)) {
        cJSON *c = cJSON_AddObjectToObject(root, "cfg");
        cJSON_AddNumberToObject(c, "cells",   snap.cell_count);
        cJSON_AddNumberToObject(c, "temps",   snap.temp_count);
        cJSON_AddBoolToObject(c,   "charger", snap.charger);
        cJSON_AddBoolToObject(c,   "load",    snap.load);
        cJSON_AddNumberToObject(c, "cycles",  snap.cycles);
    }
    if ((snap.valid & (1u << 5)) && snap.cell_count > 0) {
        cJSON *vs = cJSON_AddArrayToObject(root, "voltages");
        for (int i = 0; i < snap.cell_count; i++)
            cJSON_AddItemToArray(vs, cJSON_CreateNumber((double)snap.cell_v[i]));
    }
    if ((snap.valid & (1u << 6)) && snap.cell_count > 0) {
        cJSON *bal = cJSON_AddArrayToObject(root, "balance");
        for (int i = 0; i < snap.cell_count; i++)
            cJSON_AddItemToArray(bal, cJSON_CreateBool(snap.balance[i]));
    }
    if ((snap.valid & (1u << 7)) && snap.temp_count > 0) {
        cJSON *ct = cJSON_AddArrayToObject(root, "cell_temps");
        int tCount = snap.temp_count < 16 ? snap.temp_count : 16;
        for (int i = 0; i < tCount; i++)
            cJSON_AddItemToArray(ct, cJSON_CreateNumber((double)snap.cell_temps[i]));
    }
    if (snap.valid & (1u << 8)) {
        cJSON *al = cJSON_AddObjectToObject(root, "alarms");
        cJSON_AddNumberToObject(al, "flags",           (double)snap.alarm_flags);
        cJSON_AddBoolToObject(al, "cell_overvolt",   (snap.alarm_flags >> 7) & 1);
        cJSON_AddBoolToObject(al, "cell_undervolt",  (snap.alarm_flags >> 6) & 1);
        cJSON_AddBoolToObject(al, "pack_overvolt",   (snap.alarm_flags >> 5) & 1);
        cJSON_AddBoolToObject(al, "pack_undervolt",  (snap.alarm_flags >> 4) & 1);
        cJSON_AddBoolToObject(al, "chg_overtemp",    (snap.alarm_flags >> 3) & 1);
        cJSON_AddBoolToObject(al, "chg_undertemp",   (snap.alarm_flags >> 2) & 1);
        cJSON_AddBoolToObject(al, "dis_overtemp",    (snap.alarm_flags >> 1) & 1);
        cJSON_AddBoolToObject(al, "dis_undertemp",   (snap.alarm_flags >> 0) & 1);
        cJSON_AddBoolToObject(al, "chg_overcurrent", (snap.alarm_flags >> 15) & 1);
        cJSON_AddBoolToObject(al, "dis_overcurrent", (snap.alarm_flags >> 14) & 1);
        cJSON_AddBoolToObject(al, "short_circuit",   (snap.alarm_flags >> 13) & 1);
        cJSON_AddBoolToObject(al, "ic_error",        (snap.alarm_flags >> 12) & 1);
        cJSON_AddBoolToObject(al, "mos_lock",        (snap.alarm_flags >> 11) & 1);
    }
    cJSON_AddNumberToObject(root, "uptime",
                            (double)(esp_timer_get_time() / 1000000ULL));
    cJSON_AddNumberToObject(root, "heap",
                            (double)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}
//-------------------------------------------------------------------
/* ---- envío async desde tarea al WebSocket ---- */
struct ws_send_arg_t {
    httpd_handle_t hd;
    int            fd;
    char          *json;
    size_t         len;
};
//------------------------------------------------------
static void ws_async_send_fn(void *arg)
{
    ws_send_arg_t *a = (ws_send_arg_t *)arg;
    httpd_ws_frame_t pkt = {};
    pkt.type    = HTTPD_WS_TYPE_TEXT;
    pkt.payload = (uint8_t *)a->json;
    pkt.len     = a->len;
    pkt.final   = true;
    esp_err_t r = httpd_ws_send_frame_async(a->hd, a->fd, &pkt);
    if (r != ESP_OK) {
        for (int i = 0; i < WS_MAX_CLIENTS; i++)
            if (s_ws_fds[i] == a->fd) { s_ws_fds[i] = -1; break; }
    }
    free(a->json);
    free(a);
}
//
/* ---- tarea de push periódico (1 s) ---- */
static void ws_push_task(void *pvParameters)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_httpd) continue;
        char  *json = build_status_json();
        if (!json) continue;
        size_t len  = strlen(json);
        for (int i = 0; i < WS_MAX_CLIENTS; i++) {
            if (s_ws_fds[i] == -1) continue;
            ws_send_arg_t *a = (ws_send_arg_t *)malloc(sizeof(ws_send_arg_t));
            if (!a) continue;
            a->hd   = s_httpd;
            a->fd   = s_ws_fds[i];
            a->json = strdup(json);
            a->len  = len;
            if (!a->json ||
                httpd_queue_work(s_httpd, ws_async_send_fn, a) != ESP_OK) {
                if (a->json) free(a->json);
                free(a);
            }
        }
        free(json);
    }
}
//-------------------------------------------------------------------
/* ---- GET / → HTML ---- */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req,
                    (const char *)bms_ui_html_start,
                    bms_ui_html_end - bms_ui_html_start);
    return ESP_OK;
}
//-------------------------------------------------------------------
/* ---- GET /favicon.ico → 204 No Content ---- */
static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, nullptr, 0);
    return ESP_OK;
}
//-------------------------------------------------------------------
/* ---- WebSocket /ws ---- */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        for (int i = 0; i < WS_MAX_CLIENTS; i++)
            if (s_ws_fds[i] == -1) { s_ws_fds[i] = fd; break; }
        ESP_LOGI(TAG, "WS cliente conectado fd=%d", fd);
        return ESP_OK;
    }
    /* Mensaje entrante: comando del frontend */
    httpd_ws_frame_t pkt = {};
    uint8_t buf[256]     = {};
    pkt.payload = buf;
    if (httpd_ws_recv_frame(req, &pkt, sizeof(buf) - 1) != ESP_OK) return ESP_FAIL;
    if (pkt.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;
    buf[pkt.len] = 0;
    cJSON *json = cJSON_Parse((char *)buf);
    if (!json) return ESP_OK;
    cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
    if (cmd && cJSON_IsString(cmd)) {
        const char *cs = cmd->valuestring;
        if (strcmp(cs, "connect") == 0) {
            if (!connected && bmsInfo.found) connectToBMS();
        } else if (strcmp(cs, "disconnect") == 0) {
            if (pClient && pClient->isConnected()) pClient->disconnect();
            cleanupClient();  connected = false;
        } else if (strcmp(cs, "scan") == 0) {
            s_trigger_scan = true;
        } else if (strcmp(cs, "request") == 0) {
            cJSON *code = cJSON_GetObjectItem(json, "code");
            if (code && cJSON_IsNumber(code))
                sendDalyRequest((uint8_t)code->valueint);
        } else if (strcmp(cs, "set_charge_mos") == 0) {
            /* { "cmd": "set_charge_mos", "enable": true|false } */
            cJSON *en = cJSON_GetObjectItem(json, "enable");
            if (en && cJSON_IsBool(en)) {
                uint8_t payload[8] = {};
                payload[0] = cJSON_IsTrue(en) ? 0x01 : 0x00;
                sendDalyRequest(0x44, payload);
                ESP_LOGI(TAG, "WS → set_charge_mos %s", cJSON_IsTrue(en) ? "ON" : "OFF");
            }
        } else if (strcmp(cs, "set_discharge_mos") == 0) {
            /* { "cmd": "set_discharge_mos", "enable": true|false } */
            cJSON *en = cJSON_GetObjectItem(json, "enable");
            if (en && cJSON_IsBool(en)) {
                uint8_t payload[8] = {};
                payload[0] = cJSON_IsTrue(en) ? 0x01 : 0x00;
                sendDalyRequest(0x45, payload);
                ESP_LOGI(TAG, "WS → set_discharge_mos %s", cJSON_IsTrue(en) ? "ON" : "OFF");
            }
        } else if (strcmp(cs, "reset_soc") == 0) {
            /* { "cmd": "reset_soc" }  — resetea el SOC al 100% */
            sendDalyRequest(0x00);
            ESP_LOGI(TAG, "WS → reset_soc enviado");
        }
    }
    cJSON_Delete(json);
    return ESP_OK;
}
//-------------------------------------------------------------------
/* ---- iniciar HTTP server ---- */
static void http_server_init(void)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) s_ws_fds[i] = -1;

    httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = WS_MAX_CLIENTS + 2;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando HTTP server");
        return;
    }
    static const httpd_uri_t root_uri = {
        .uri                    = "/",
        .method                 = HTTP_GET,
        .handler                = root_handler,
        .user_ctx               = nullptr,
        .is_websocket           = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol  = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &root_uri);

    static const httpd_uri_t ws_uri = {
        .uri                    = "/ws",
        .method                 = HTTP_GET,
        .handler                = ws_handler,
        .user_ctx               = nullptr,
        .is_websocket           = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol  = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &ws_uri);

    static const httpd_uri_t favicon_uri = {
        .uri                    = "/favicon.ico",
        .method                 = HTTP_GET,
        .handler                = favicon_handler,
        .user_ctx               = nullptr,
        .is_websocket           = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol  = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &favicon_uri);

    ESP_LOGI(TAG, "HTTP server en http://%s/",
             s_bms.wifi_ip[0] ? s_bms.wifi_ip : "<ip pendiente>");

    xTaskCreate(ws_push_task, "ws_push", 4096, nullptr, 4, nullptr);
}
//-------------------------------------------------------------------
/* ================================================================== */
/* ---- WiFi: NVS credentials + captive portal provisioning      ---- */
/* ================================================================== */

/* ---- HTML del captive portal (embebido inline) ---- */
static const char CP_HTML[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>DALY BMS - WiFi Setup</title>"
"<style>"
"*{box-sizing:border-box}"
"body{font-family:sans-serif;max-width:420px;margin:30px auto;padding:16px;background:#0d1b2a;color:#e0e0e0}"
"h1{color:#00d4ff;text-align:center;margin-bottom:20px}"
"label{display:block;margin:10px 0 3px;color:#aaa;font-size:.9em}"
"input,select{width:100%;padding:9px;background:#1a2a3a;color:#eee;border:1px solid #2a4a6a;border-radius:5px;font-size:1em}"
".btn{display:block;width:100%;padding:11px;margin-top:8px;border:none;border-radius:5px;cursor:pointer;font-size:1em;font-weight:bold}"
".sec{background:#1a3a5a;color:#00d4ff}.pri{background:#00d4ff;color:#000;margin-top:14px}"
"#st{text-align:center;margin-top:14px;min-height:18px;color:#aaa}"
"</style></head><body>"
"<h1>DALY BMS<br><small style=\"font-size:.5em;color:#888\">WiFi Setup</small></h1>"
"<label>Red WiFi (SSID)</label>"
"<input id=\"sid\" type=\"text\" placeholder=\"Nombre de la red\" required>"
"<label>Contrase&#xF1;a</label>"
"<input id=\"pwd\" type=\"password\" placeholder=\"Vac&#xED;o si es red abierta\">"
"<button class=\"btn sec\" onclick=\"scan()\">&#128246; Escanear redes</button>"
"<select id=\"nets\" style=\"display:none;margin-top:5px\""
" onchange=\"if(this.value)document.getElementById('sid').value=this.value\">"
"<option value=\"\">-- Seleccionar --</option></select>"
"<button class=\"btn pri\" onclick=\"save()\">&#10003; Guardar y reiniciar</button>"
"<div id=\"st\"></div>"
"<script>"
"function st(m,c){var e=document.getElementById('st');e.textContent=m;e.style.color=c||'#aaa';}"
"function scan(){"
"  st('Escaneando...');"
"  fetch('/wifi/scan').then(function(r){return r.json();}).then(function(d){"
"    var s=document.getElementById('nets');"
"    s.innerHTML='<option value=\"\">-- Seleccionar --</option>';"
"    d.forEach(function(n){var o=document.createElement('option');o.textContent=n;s.appendChild(o);});"
"    s.style.display='block';st(d.length+' redes encontradas');"
"  }).catch(function(){st('Error al escanear','#f66');});"
"}"
"function save(){"
"  var s=document.getElementById('sid').value.trim();"
"  if(!s){st('Introduce un SSID','#f66');return;}"
"  st('Guardando...');"
"  fetch('/wifi/save',{method:'POST',headers:{'Content-Type':'application/json'},"
"    body:JSON.stringify({ssid:s,pass:document.getElementById('pwd').value})})"
"  .then(function(r){return r.text();}).then(function(t){st(t,'#0d0');})"
"  .catch(function(){st('Error guardando','#f66');});"
"}"
"</script></body></html>";

/* ---- NVS: cargar credenciales WiFi ---- */
static bool wifi_credentials_load(char *ssid, size_t ssid_len,
                                   char *pass, size_t pass_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = (nvs_get_str(h, NVS_WIFI_SSID, ssid, &ssid_len) == ESP_OK &&
               nvs_get_str(h, NVS_WIFI_PASS, pass, &pass_len) == ESP_OK &&
               ssid[0] != '\0');
    nvs_close(h);
    return ok;
}

/* ---- NVS: guardar credenciales WiFi ---- */
static void wifi_credentials_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_WIFI_SSID, ssid);
    nvs_set_str(h, NVS_WIFI_PASS, pass ? pass : "");
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG_WIFI, "Credenciales WiFi guardadas para SSID: '%s'", ssid);
}

/* ---- DNS server: responde todas las queries A con 192.168.4.1 ---- */
static void dns_server_task(void *pvParameters)
{
    const uint8_t ap_ip[4] = {192, 168, 4, 1};

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG_WIFI, "DNS captive portal escuchando en :53");

    uint8_t buf[512];
    uint8_t resp[512 + 20];

    for (;;) {
        struct sockaddr_in client = {};
        socklen_t clen = sizeof(client);
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr *)&client, &clen);
        if (len < 12 || len > (int)(sizeof(resp) - 20)) continue;

        memcpy(resp, buf, (size_t)len);
        resp[2] = 0x81; /* QR=1, AA=1 */
        resp[3] = 0x80; /* RA=1, RCODE=0 */
        resp[6] = 0x00; resp[7] = 0x01;  /* answer count = 1 */
        resp[8] = resp[9]   = 0x00;      /* authority = 0    */
        resp[10] = resp[11] = 0x00;      /* additional = 0   */

        int rlen = len;
        /* Answer: ptr→pregunta, tipo A, clase IN, TTL 10s, 4 bytes IP */
        resp[rlen++] = 0xC0; resp[rlen++] = 0x0C;
        resp[rlen++] = 0x00; resp[rlen++] = 0x01; /* type A  */
        resp[rlen++] = 0x00; resp[rlen++] = 0x01; /* class IN */
        resp[rlen++] = 0x00; resp[rlen++] = 0x00;
        resp[rlen++] = 0x00; resp[rlen++] = 0x0A; /* TTL = 10s */
        resp[rlen++] = 0x00; resp[rlen++] = 0x04; /* rdlength = 4 */
        resp[rlen++] = ap_ip[0]; resp[rlen++] = ap_ip[1];
        resp[rlen++] = ap_ip[2]; resp[rlen++] = ap_ip[3];

        sendto(sock, resp, (size_t)rlen, 0, (struct sockaddr *)&client, clen);
    }
}

/* ---- HTTP: página principal del captive portal ---- */
static esp_err_t wifi_setup_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CP_HTML, (ssize_t)strlen(CP_HTML));
    return ESP_OK;
}

/* ---- HTTP: escaneo WiFi → JSON array de SSIDs ---- */
static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    wifi_scan_config_t scfg = {};
    scfg.show_hidden = false;
    esp_wifi_scan_start(&scfg, true);   /* bloqueante */

    uint16_t count = 20;
    wifi_ap_record_t records[20];
    memset(records, 0, sizeof(records));
    esp_wifi_scan_get_ap_records(&count, records);

    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < (int)count; i++) {
        if (records[i].ssid[0] != '\0')
            cJSON_AddItemToArray(arr, cJSON_CreateString((char *)records[i].ssid));
    }
    char *json = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, (ssize_t)strlen(json));
    free(json);
    return ESP_OK;
}

/* ---- HTTP: guardar credenciales WiFi y reiniciar ---- */
static esp_err_t wifi_save_handler(httpd_req_t *req)
{
    char buf[384] = {};
    int received  = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    cJSON *json   = cJSON_Parse(buf);
    cJSON *ssid_j = json ? cJSON_GetObjectItem(json, "ssid") : nullptr;
    cJSON *pass_j = json ? cJSON_GetObjectItem(json, "pass") : nullptr;

    if (!ssid_j || !cJSON_IsString(ssid_j) || ssid_j->valuestring[0] == '\0') {
        if (json) cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID requerido");
        return ESP_FAIL;
    }

    const char *ssid = ssid_j->valuestring;
    const char *pass = (pass_j && cJSON_IsString(pass_j)) ? pass_j->valuestring : "";
    wifi_credentials_save(ssid, pass);
    cJSON_Delete(json);

    static const char ok_msg[] =
        "Credenciales guardadas. Reiniciando en 2 segundos...";
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, ok_msg, (ssize_t)sizeof(ok_msg) - 1);

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK; /* no alcanzado */
}

/* ---- HTTP 404: redirige todo al portal (captive portal detection) ---- */
static esp_err_t captive_404_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ---- iniciar HTTP server del captive portal ---- */
static void http_captive_server_init(void)
{
    httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = AP_MAX_CONN + 2;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando HTTP captive portal");
        return;
    }

    static const httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET,
        .handler = wifi_setup_handler, .user_ctx = nullptr,
        .is_websocket = false, .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &uri_root);

    static const httpd_uri_t uri_scan = {
        .uri = "/wifi/scan", .method = HTTP_GET,
        .handler = wifi_scan_handler, .user_ctx = nullptr,
        .is_websocket = false, .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &uri_scan);

    static const httpd_uri_t uri_save = {
        .uri = "/wifi/save", .method = HTTP_POST,
        .handler = wifi_save_handler, .user_ctx = nullptr,
        .is_websocket = false, .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &uri_save);

    /* captive portal detection: Android /generate_204 → todo via 404 handler */
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, captive_404_handler);

    ESP_LOGI(TAG, "Captive portal activo en http://192.168.4.1/");
}

/* ---- arrancar modo provisioning: SoftAP + DNS + captive portal ---- */
static void wifi_start_provisioning(void)
{
    wifi_config_t ap_cfg = {};
    memcpy(ap_cfg.ap.ssid, AP_SSID, strlen(AP_SSID));
    ap_cfg.ap.ssid_len       = (uint8_t)strlen(AP_SSID);
    ap_cfg.ap.channel        = AP_CHANNEL;
    ap_cfg.ap.max_connection = AP_MAX_CONN;
    ap_cfg.ap.authmode       = WIFI_AUTH_OPEN;

    /* APSTA: AP visible + STA disponible para escaneo de redes */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(dns_server_task, "dns_srv", 4096, nullptr, 5, nullptr);
    http_captive_server_init();

    ESP_LOGI(TAG_WIFI, "Provisioning: conecta a WiFi '%s' → http://192.168.4.1/",
             AP_SSID);
}

/* ================================================================== */
/* ---- WiFi STA                                                  ---- */
/* ================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_count < WIFI_RETRY_MAX) {
            esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGW(TAG_WIFI, "Reintentando conexión (%d/%d)...",
                     s_wifi_retry_count, WIFI_RETRY_MAX);
        } else {
            if (s_wifi_event_group)
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG_WIFI, "No se pudo conectar a la red WiFi guardada");
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        if (s_bms_mutex) {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            snprintf(s_bms.wifi_ip, sizeof(s_bms.wifi_ip),
                     IPSTR, IP2STR(&event->ip_info.ip));
            xSemaphoreGive(s_bms_mutex);
        }
        s_wifi_retry_count = 0;
        if (s_wifi_event_group)
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
//-------------------------------------------------------------------
/* ---- intento de conexión WiFi STA con credenciales dadas ---- */
/* Retorna true si se obtiene IP, false si falla o timeout.      */
static bool wifi_try_sta(const char *ssid, const char *pass)
{
    s_wifi_event_group = xEventGroupCreate();
    s_wifi_retry_count = 0;

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode =
        (pass[0] != '\0') ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable  = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Conectando a SSID: '%s'...", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "Conectado a '%s'", ssid);
        /* dejar handlers registrados para gestionar reconexiones */
        return true;
    }

    ESP_LOGW(TAG_WIFI, "No se pudo conectar a '%s' — iniciando provisioning", ssid);

    /* desregistrar handlers antes de parar WiFi para evitar race conditions */
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,   instance_any_id);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_wifi_stop();
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = nullptr;
    return false;
}
//-------------------------------------------------------------------
/* ---- tarea principal BLE (equivale a setup() + loop()) ---- */
static void daly_ble_task(void *pvParameters)
{
    /* setup() */
    NimBLEDevice::init("ESP32-Daly-BLE");
    recomputeExpectedFrames();
    ESP_LOGI(TAG, "Escaneando BMS en %s ...", BMS_MAC_ADDRESS);
    scanForBMS(8);

    if (bmsInfo.found && connectToBMS()) {
        connected = true;
        delay(300);
        sendDalyRequest(0x94);
        delay(200);
        sendDalyRequest(0x90); delay(120);
        sendDalyRequest(0x91); delay(120);
        sendDalyRequest(0x92); delay(120);
        sendDalyRequest(0x93); delay(120);
        sendDalyRequest(0x95);
        sendDalyRequest(0x96); delay(120);
        sendDalyRequest(0x97);
        sendDalyRequest(0x98);
    } else {
        ESP_LOGW(TAG, "BMS no encontrado en el escaneo inicial");
    }

    /* loop() */
    static uint64_t lastPoll      = 0;
    static uint64_t lastPoll98    = 0;
    static uint64_t lastReconnect = 0;

    for (;;) {
        if (connected && pClient && pClient->isConnected()) {
            if (millis() - lastPoll > 1000) {
                sendDalyRequest(0x90); delay(80);
                sendDalyRequest(0x91); delay(80);
                sendDalyRequest(0x92); delay(80);
                sendDalyRequest(0x93); delay(80);
                sendDalyRequest(0x94); delay(150);
                sendDalyRequest(0x95);
                sendDalyRequest(0x96); delay(80);
                sendDalyRequest(0x97);
                lastPoll = millis();
            }
            if (millis() - lastPoll98 > 2000) {
                sendDalyRequest(0x98);
                lastPoll98 = millis();
            }
        } else {
            if (s_trigger_scan || (millis() - lastReconnect > 3000)) {
                s_trigger_scan = false;
                cleanupClient();
                ESP_LOGI(TAG, "Intentando reconexión...");
                scanForBMS(6);
                if (bmsInfo.found) connectToBMS();
                lastReconnect = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
//-------------------------------------------------------------------
/* ------------------------------------------------------------------ */
/* ---- Tarea push remoto a Hostalia                              ---- */
/* ------------------------------------------------------------------ */
static void remote_push_task(void *pvParameters)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(REMOTE_PUSH_MS));

        /* Esperar a tener WiFi (ip conocida) */
        if (s_bms_mutex) {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            bool has_ip = (s_bms.wifi_ip[0] != '\0');
            xSemaphoreGive(s_bms_mutex);
            if (!has_ip) continue;
        }

        char *json = build_status_json();
        if (!json) continue;

        esp_http_client_config_t cfg = {};
        cfg.url               = REMOTE_PUSH_URL;
        cfg.method            = HTTP_METHOD_POST;
        cfg.crt_bundle_attach = esp_crt_bundle_attach;  /* verifica certificado SSL */
        cfg.timeout_ms        = 8000;

        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_header(client, "X-API-Key",    REMOTE_API_KEY);
        esp_http_client_set_post_field(client, json, (int)strlen(json));

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int status = esp_http_client_get_status_code(client);
            if (status == 200) {
                ESP_LOGI(TAG, "Push remoto OK (HTTP 200)");
            } else {
                ESP_LOGW(TAG, "Push remoto: HTTP %d", status);
            }
        } else {
            ESP_LOGW(TAG, "Push remoto falló: %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
        free(json);
    }
}
//-------------------------------------------------------------------
/* ---- punto de entrada ESP-IDF ---- */
extern "C" void app_main(void)
{
    /* NVS requerido por WiFi y BLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Mutex para BmsState — crear antes de arrancar cualquier tarea */
    s_bms_mutex = xSemaphoreCreateMutex();
    configASSERT(s_bms_mutex);

    /* Reducir verbosidad de NimBLE (oculta cada write/procedure GATT) */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    /* Netif + event loop compartido con NimBLE — solo una vez */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Crear ambas interfaces (STA y AP) antes de inicializar el driver */
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    /* Inicializar driver WiFi (solo una vez) */
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    /* Intentar conectar con credenciales guardadas en NVS */
    char saved_ssid[64] = {};
    char saved_pass[64] = {};
    bool has_creds = wifi_credentials_load(saved_ssid, sizeof(saved_ssid),
                                           saved_pass, sizeof(saved_pass));
    bool wifi_ok = false;

    if (has_creds) {
        ESP_LOGI(TAG_WIFI, "Credenciales guardadas: SSID='%s'", saved_ssid);
        wifi_ok = wifi_try_sta(saved_ssid, saved_pass);
    } else {
        ESP_LOGW(TAG_WIFI, "No hay credenciales WiFi guardadas");
    }

    if (!wifi_ok) {
        /* Sin WiFi: modo provisioning (SoftAP + DNS + captive portal) */
        wifi_start_provisioning();
        /* Loop indefinido — el restart se hace desde wifi_save_handler tras guardar */
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Modo normal: HTTP server BMS + mDNS + BLE + remote push */
    http_server_init();

    /* mDNS: accesible como http://bms.local en la red local */
    mdns_init();
    mdns_hostname_set("bms");
    mdns_instance_name_set("Daly BMS");
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    ESP_LOGI(TAG, "mDNS iniciado: http://bms.local/");

    xTaskCreate(daly_ble_task,    "daly_ble",    12288, nullptr, 5, nullptr);
    xTaskCreate(remote_push_task, "remote_push",  8192, nullptr, 3, nullptr);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//-------------------------------------------------------------------