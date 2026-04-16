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
#include "esp_heap_caps.h"
#include "cJSON.h"

static const char *TAG     = "DALY_BLE";
static const char *TAG_WIFI = "WIFI";

/* ---- credenciales WiFi ---- */
#define WIFI_SSID        "LuaNet"
#define WIFI_PASS        "Escondido2015"
#define WIFI_RETRY_MAX   10

/* bits del EventGroup WiFi */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = nullptr;
static int                s_wifi_retry_count  = 0;

#define BMS_MAC_ADDRESS   "50:18:06:01:15:2a"
#define SERVICE_UUID_MAIN "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHAR_UUID_TX      "0000fff1-0000-1000-8000-00805f9b34fb"  /* NOTIFY BMS→Host */
#define CHAR_UUID_RX      "0000fff2-0000-1000-8000-00805f9b34fb"  /* WRITE  Host→BMS */
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
    bool     ble_connected;
    char     wifi_ip[16];
    uint8_t  valid;          /* bitmask: bit0=0x90 … bit6=0x97 */
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
    scan->start(durationSeconds, false);

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
    pClient->setConnectTimeout(5);
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

    ESP_LOGI(TAG, "HTTP server en http://%s/",
             s_bms.wifi_ip[0] ? s_bms.wifi_ip : "<ip pendiente>");

    xTaskCreate(ws_push_task, "ws_push", 4096, nullptr, 4, nullptr);
}
//-------------------------------------------------------------------
/* ------------------------------------------------------------------ */
/* ---- WiFi STA                                                  ---- */
/* ------------------------------------------------------------------ */

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
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG_WIFI, "No se pudo conectar a '" WIFI_SSID "'");
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
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
//-------------------------------------------------------------------
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* registrar handlers de eventos */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable    = true;
    wifi_config.sta.pmf_cfg.required   = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Conectando a SSID: '" WIFI_SSID "'...");

    /* esperar resultado de la conexión inicial */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "Conectado a '" WIFI_SSID "'");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG_WIFI, "Falló la conexión WiFi — continuando sin red");
    } else {
        ESP_LOGW(TAG_WIFI, "Timeout esperando WiFi — continuando sin red");
    }
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
        sendDalyRequest(0x97);
    } else {
        ESP_LOGW(TAG, "BMS no encontrado en el escaneo inicial");
    }

    /* loop() */
    static uint64_t lastPoll      = 0;
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
                sendDalyRequest(0x97);
                lastPoll = millis();
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

    /* WiFi primero: inicializa netif y el event loop compartido con NimBLE */
    wifi_init_sta();

    /* HTTP server + WebSocket (requiere IP activa) */
    http_server_init();

    xTaskCreate(daly_ble_task, "daly_ble", 12288, nullptr, 5, nullptr);
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//-------------------------------------------------------------------