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
#include "esp_spiffs.h"
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>

#ifdef CONFIG_BMS_TYPE_DALY
static const char *TAG     = "DALY_BLE";
#else
static const char *TAG     = "JK_BLE";
#endif
static const char *TAG_WIFI = "WIFI";

/* ---- WiFi STA ---- */
#define WIFI_RETRY_MAX   10

/* ---- AP provisioning ---- */
#ifdef CONFIG_BMS_TYPE_DALY
#  define AP_SSID        "DALY-BMS-Setup"
#else
#  define AP_SSID        "JK-BMS-Setup"
#endif
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

/* ---- Dirección MAC del BMS (configurable vía menuconfig) ---- */
#ifdef CONFIG_BMS_TYPE_DALY
#  define BMS_MAC_ADDRESS   CONFIG_BMS_DALY_MAC_ADDRESS
#else
#  define BMS_MAC_ADDRESS   CONFIG_BMS_JK_MAC_ADDRESS
#endif

/* ---- UUIDs BLE según tipo de BMS ---- */
#ifdef CONFIG_BMS_TYPE_DALY
#  define SERVICE_UUID_MAIN "0000fff0-0000-1000-8000-00805f9b34fb"
#  define CHAR_UUID_TX      "0000fff1-0000-1000-8000-00805f9b34fb"  /* NOTIFY BMS→Host */
#  define CHAR_UUID_RX      "0000fff2-0000-1000-8000-00805f9b34fb"  /* WRITE  Host→BMS */
#endif
#ifdef CONFIG_BMS_TYPE_JK
#  define SERVICE_UUID_MAIN "0000ffe0-0000-1000-8000-00805f9b34fb"
#  define CHAR_UUID_TX      "0000ffe1-0000-1000-8000-00805f9b34fb"  /* NOTIFY BMS→Host */
#  define CHAR_UUID_RX      "0000ffe1-0000-1000-8000-00805f9b34fb"  /* WRITE  Host→BMS */
/* JK BMS: trama fija de 300 bytes, protocolo JK02_24S */
#  define JK_FRAME_SIZE     300
#  define JK_CMD_CELL_INFO  0x96
#  define JK_CMD_DEV_INFO   0x97
#endif

/* ---- Push remoto a Hostalia ---- */
#define REMOTE_PUSH_URL   "https://luaresearch.es/bms/update.php"  /* cambiar por tu dominio */
#define REMOTE_API_KEY    "c864645245ca1dcd5d6a0d6ca3754512c26a7a07ff722c853d1587b18bf1916f"              /* igual que en update.php */
#define REMOTE_PUSH_MS    5000                                     /* intervalo push en ms */

/* ---- Push histórico remoto ---- */
#define REMOTE_HIST_URL   "https://luaresearch.es/bms/update_history.php"
#define REMOTE_HIST_USER  "lua"   /* usuario HTTP Basic del servidor */
#define REMOTE_HIST_PASS  "0304"        /* contraseña HTTP Basic (dejar vacío si no hay) */
/* ---- Intervalo de agregación: 5 minutos ---- */
#define HIST_INTERVAL_S   60
/* ---- Epoch mínima para considerar SNTP sincronizado (2020-01-01) ---- */
#define SNTP_EPOCH_MIN    1577836800UL
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
/* ================================================================== */
/* ---- Variables de ensamblado de trama DALY                    ---- */
/* ================================================================== */
#ifdef CONFIG_BMS_TYPE_DALY
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
#endif /* CONFIG_BMS_TYPE_DALY */

#ifdef CONFIG_BMS_TYPE_JK
/* ---- Buffer de trama JK (300 bytes fijos) ---- */
static uint8_t  jkStreamBuf[JK_FRAME_SIZE + 32];
static size_t   jkStreamLen = 0;
#endif /* CONFIG_BMS_TYPE_JK */

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

/* ---- Registro histórico binario (12 bytes por entrada) ---- */
typedef struct {
    uint32_t ts;       /* Unix timestamp inicio del intervalo */
    float    voltage;  /* promedio voltaje (V) */
    float    current;  /* promedio corriente (A, negativo = descarga) */
} HistRecord;

/* ---- Acumuladores 1 Hz → promedio 5 min ---- */
static float             s_volt_sum   = 0.0f;
static float             s_curr_sum   = 0.0f;
static int               s_sample_cnt = 0;
static SemaphoreHandle_t s_hist_mutex = nullptr;

/* ---- HTTP / WebSocket globals ---- */
#define WS_MAX_CLIENTS 4
static httpd_handle_t    s_httpd       = nullptr;
static int               s_ws_fds[WS_MAX_CLIENTS];
static volatile bool     s_trigger_scan = false;
//-------------------------------------------------------------------
/* ================================================================== */
/* ---- Funciones de protocolo DALY                               ---- */
/* ================================================================== */
#ifdef CONFIG_BMS_TYPE_DALY
//-------------------------------------------------------------------
/* ---- verificación de checksum DALY ---- */
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
#endif /* CONFIG_BMS_TYPE_DALY */
/* ================================================================== */
/* ---- Funciones de protocolo JK BMS (JK02_24S)                 ---- */
/* ================================================================== */
#ifdef CONFIG_BMS_TYPE_JK
//-------------------------------------------------------------------
static inline uint16_t jk_get_16bit(const uint8_t *d, size_t i)
{
    return (uint16_t(d[i + 1]) << 8) | uint16_t(d[i]);
}
static inline uint32_t jk_get_32bit(const uint8_t *d, size_t i)
{
    return (uint32_t(d[i + 3]) << 24) | (uint32_t(d[i + 2]) << 16) |
           (uint32_t(d[i + 1]) <<  8) |  uint32_t(d[i]);
}
//-------------------------------------------------------------------
/* ---- parser de la trama JK cell info (300 bytes, type 0x02) ---- */
static void parseJkCellInfo(const uint8_t *d)
{
    /* Protocolo JK02_32S (offset_final = 32 respecto a JK02_24S)
     * d[0..3]  = 55 AA EB 90 (cabecera)
     * d[4]    = tipo (0x02)
     * d[5]    = contador de trama
     * d[6-69] = 32 × uint16 LE tensiones de celda (mV)  — las no usadas valen 0
     * d[70-73]= bitmask de celdas habilitadas (uint32 LE)
     * d[80-143]= 32 × uint16 LE resistencias de celda (mΩ)
     * d[144-145] = temp. MOS     (int16 LE, ×0.1 °C)   [112+32]
     * d[146-149] = bitmask resist. alambre
     * d[150-153] = tensión pack (uint32 LE, ×0.001 V)  [118+32]
     * d[154-157] = potencia pack (uint32 LE, ×0.001 W) [122+32]
     * d[158-161] = corriente (int32 LE, ×0.001 A)      [126+32]
     * d[162-163] = temp. sensor 1 (int16 LE, ×0.1 °C) [130+32]
     * d[164-165] = temp. sensor 2 (int16 LE, ×0.1 °C) [132+32]
     * d[166-167] = alarmas (big-endian uint16)          [134+32]
     * d[170-171] = corriente balanceo (int16 LE, ×0.001 A) [138+32]
     * d[172]    = balanceo (0=off, 1=carga, 2=descarga) [140+32]
     * d[173]    = SOC (uint8, %)                        [141+32]
     * d[174-177]= capacidad restante (uint32 LE, mAh)  [142+32]
     * d[178-181]= capacidad nominal  (uint32 LE, mAh)  [146+32]
     * d[182-185]= ciclos (uint32 LE)                   [150+32]
     * d[198]    = MOS carga habilitado                  [166+32]
     * d[199]    = MOS descarga habilitado               [167+32]
     * d[245]    = cargador conectado                    [213+32]
     * d[299]    = CRC / campo de estado
     */

    /* Determinar número de celdas a partir del bitmask (bitmask en d[70] para JK02_32S) */
    uint32_t cellMask = jk_get_32bit(d, 70);
    int cellCount = __builtin_popcount(cellMask);
    if (cellCount < 1 || cellCount > 32) cellCount = 16;

    xSemaphoreTake(s_bms_mutex, portMAX_DELAY);

    /* Tensiones de celda */
    for (int i = 0; i < cellCount; i++) {
        uint16_t mv = jk_get_16bit(d, 6 + i * 2);
        s_bms.cell_mv[i] = mv;
        s_bms.cell_v[i]  = mv * 0.001f;
    }
    s_bms.cell_count = (uint8_t)cellCount;
    s_bms.valid |= (1u << 5);

    /* Tensión y corriente del pack */
    uint32_t packV_raw   = jk_get_32bit(d, 150);      /* d[118+32] */
    int32_t  current_raw = (int32_t)jk_get_32bit(d, 158); /* d[126+32] */
    s_bms.voltage = packV_raw * 0.001f;
    /* Convenio igual que DALY: positivo = descarga, negativo = carga */
    s_bms.current = -(current_raw * 0.001f);
    s_bms.soc     = (float)d[173];                    /* d[141+32] */
    s_bms.valid  |= (1u << 0);

    /* Capacidad restante (raw en mAh) */
    s_bms.remaining_mah = jk_get_32bit(d, 174);       /* d[142+32] */

    /* Ciclos */
    s_bms.cycles = (uint16_t)jk_get_32bit(d, 182);   /* d[150+32] */
    s_bms.valid |= (1u << 4);

    /* Temperaturas JK02_32S (16S = 4 sensores batería, 1 por cada 4 celdas):
     * d[162-163] = Sensor batería 1  [130+32]
     * d[164-165] = Sensor batería 2  [132+32]
     * d[258-259] = Sensor batería 3  [226+32]
     * d[256-257] = Sensor batería 4  [224+32]
     * d[144-145] = MOS (power tube)  [112+32] */
    float temp1   = (float)((int16_t)jk_get_16bit(d, 162)) * 0.1f; /* sensor 1 */
    float temp2   = (float)((int16_t)jk_get_16bit(d, 164)) * 0.1f; /* sensor 2 */
    float temp3   = (float)((int16_t)jk_get_16bit(d, 258)) * 0.1f; /* sensor 3 */
    float temp4   = (float)((int16_t)jk_get_16bit(d, 256)) * 0.1f; /* sensor 4 */
    float tempMos = (float)((int16_t)jk_get_16bit(d, 144)) * 0.1f; /* MOS */
    s_bms.cell_temps[0] = temp1;
    s_bms.cell_temps[1] = temp2;
    s_bms.cell_temps[2] = temp3;
    s_bms.cell_temps[3] = temp4;
    s_bms.cell_temps[4] = tempMos;
    s_bms.temp_count = 5;   /* S1..S4 batería + S5 MOS */
    /* max_temp / min_temp de los 4 sensores de batería */
    float tmax = temp1, tmin = temp1;
    uint8_t smax = 1, smin = 1;
    float tarr[4] = {temp1, temp2, temp3, temp4};
    for (int ti = 1; ti < 4; ti++) {
        if (tarr[ti] > tmax) { tmax = tarr[ti]; smax = ti + 1; }
        if (tarr[ti] < tmin) { tmin = tarr[ti]; smin = ti + 1; }
    }
    s_bms.max_temp = tmax; s_bms.max_temp_s = smax;
    s_bms.min_temp = tmin; s_bms.min_temp_s = smin;
    s_bms.valid |= (1u << 2) | (1u << 7);

    /* MOS charge / discharge */
    s_bms.charge_mos    = (bool)d[198];               /* d[166+32] */
    s_bms.discharge_mos = (bool)d[199];               /* d[167+32] */

    /* Modo de carga derivado de la corriente */
    if      (s_bms.current < -0.1f) s_bms.charge_mode = 1;   /* cargando */
    else if (s_bms.current >  0.1f) s_bms.charge_mode = 2;   /* descargando */
    else                            s_bms.charge_mode = 0;    /* estacionario */

    /* Cargador y carga detectados */
    s_bms.charger = (bool)d[245];                     /* d[213+32] */
    s_bms.load    = (s_bms.current > 0.1f);
    s_bms.valid  |= (1u << 3);

    /* Alarmas (big-endian: d[166]=HI d[167]=LO para JK02_32S → d[134+32]) */
    s_bms.alarm_flags = (uint32_t)((d[166] << 8) | d[167]);
    s_bms.valid |= (1u << 8);

    /* Min / max tensión de celda */
    float minV = 9999.0f, maxV = -1.0f;
    uint8_t minN = 1, maxN = 1;
    for (int i = 0; i < cellCount; i++) {
        float v = s_bms.cell_v[i];
        if (v > maxV) { maxV = v; maxN = (uint8_t)(i + 1); }
        if (v > 0.1f && v < minV) { minV = v; minN = (uint8_t)(i + 1); }
    }
    s_bms.max_cell_v = maxV;  s_bms.max_cell_n = maxN;
    s_bms.min_cell_v = minV;  s_bms.min_cell_n = minN;
    s_bms.diff_mv    = (int)((maxV - minV) * 1000.0f);
    s_bms.valid |= (1u << 1);

    /* Balance: indicador global (d[172] = d[140+32] para JK02_32S) */
    memset(s_bms.balance, 0, sizeof(s_bms.balance));
    if (d[172] != 0 && maxN > 0 && maxN <= 48)
        s_bms.balance[maxN - 1] = true;
    s_bms.valid |= (1u << 6);

    xSemaphoreGive(s_bms_mutex);

    ESP_LOGD(TAG, "JK cell info: V=%.2f I=%.3f SOC=%.0f%% celdas=%d diff=%dmV",
             s_bms.voltage, s_bms.current, s_bms.soc, cellCount, s_bms.diff_mv);
}
//-------------------------------------------------------------------
/* ---- ensamblar y procesar notificaciones BLE del JK BMS ---- */
static void jkAssembleAndProcess(const uint8_t *pData, size_t length)
{
    /* Si llega una cabecera nueva, descartar el buffer anterior */
    if (length >= 4 && pData[0] == 0x55 && pData[1] == 0xAA &&
        pData[2] == 0xEB && pData[3] == 0x90) {
        jkStreamLen = 0;
    }
    /* Descartar si el buffer se desborda */
    if (jkStreamLen + length > sizeof(jkStreamBuf)) {
        ESP_LOGW(TAG, "JK: buffer overflow, descartando");
        jkStreamLen = 0;
        return;
    }
    memcpy(jkStreamBuf + jkStreamLen, pData, length);
    jkStreamLen += length;

    if (jkStreamLen < JK_FRAME_SIZE) return;   /* trama incompleta */

    /* Verificar CRC: suma de bytes 0..298 mod 256 */
    uint8_t crc = 0;
    for (int i = 0; i < JK_FRAME_SIZE - 1; i++) crc += jkStreamBuf[i];
    if (crc != jkStreamBuf[JK_FRAME_SIZE - 1]) {
        /* Sólo advertencia — el BMS puede usar byte[299] como campo de estado,
         * no como CRC, según el firmware. Los datos siguen siendo válidos. */
        ESP_LOGD(TAG, "JK: CRC warn (calc=0x%02X byte299=0x%02X) — parsing igualmente",
                 crc, jkStreamBuf[JK_FRAME_SIZE - 1]);
    }

    ESP_LOGD(TAG, "JK frame: %02X %02X %02X %02X %02X %02X %02X %02X ...",
             jkStreamBuf[0], jkStreamBuf[1], jkStreamBuf[2], jkStreamBuf[3],
             jkStreamBuf[4], jkStreamBuf[5], jkStreamBuf[6], jkStreamBuf[7]);

    uint8_t frameType = jkStreamBuf[4];
    if (frameType == 0x02) parseJkCellInfo(jkStreamBuf);

    /* Eliminar trama procesada del buffer */
    memmove(jkStreamBuf, jkStreamBuf + JK_FRAME_SIZE,
            jkStreamLen - JK_FRAME_SIZE);
    jkStreamLen -= JK_FRAME_SIZE;
}
#endif /* CONFIG_BMS_TYPE_JK */

//-------------------------------------------------------------------
/* ---- callback de notificaciones BLE ---- */
static void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic,
                     uint8_t *pData, size_t length, bool isNotify)
{
#ifdef CONFIG_BMS_TYPE_DALY
    if (rxStreamLen + length > sizeof(rxStreamBuf)) {
        rxStreamLen = 0;
    }
    memcpy(rxStreamBuf + rxStreamLen, pData, length);
    rxStreamLen += length;
    processRxStream();
#endif
#ifdef CONFIG_BMS_TYPE_JK
    jkAssembleAndProcess(pData, length);
#endif
    (void)pRemoteCharacteristic; (void)isNotify;
}
//-------------------------------------------------------------------
/* ---- callbacks de escaneo BLE ---- */
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override
    {
        bool matches = false;

        /* Modo auto-detección: MAC configurada a "00:00:00:00:00:00"
         * → conectar al primer dispositivo que anuncie el UUID del servicio BMS */
        if (std::string(BMS_MAC_ADDRESS) == "00:00:00:00:00:00") {
            matches = dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID_MAIN));
            if (matches) {
                ESP_LOGI(TAG, "Auto-detectado BMS '%s'  MAC=%s  RSSI=%d",
                         dev->haveName() ? dev->getName().c_str() : "(sin nombre)",
                         dev->getAddress().toString().c_str(),
                         dev->getRSSI());
            }
        } else {
            /* Modo MAC fija: comparar dirección en minúsculas */
            std::string addr = dev->getAddress().toString();
            std::transform(addr.begin(), addr.end(), addr.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            matches = (addr == BMS_MAC_ADDRESS);
        }

        if (matches) {
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
/* ---- PIN para pairing BLE con el BMS ---- */
#define BMS_BLE_PASSKEY  1234

//-------------------------------------------------------------------
/* ---- callbacks del cliente BLE ---- */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pClient) override
    {
        connected = true;
#ifdef CONFIG_BMS_TYPE_DALY
        resetCellAssembly();
        rxStreamLen       = 0;
        frameBaseDetected = false;
#endif
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
#ifdef CONFIG_BMS_TYPE_DALY
        resetCellAssembly();
        rxStreamLen = 0;
#endif
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
    /* ---- pairing: el BMS pide que introduzcamos el PIN ---- */
    void onPassKeyEntry(NimBLEConnInfo &connInfo) override
    {
        ESP_LOGI(TAG, "BLE pairing: BMS solicita PIN, enviando %d", BMS_BLE_PASSKEY);
        NimBLEDevice::injectPassKey(connInfo, BMS_BLE_PASSKEY);
    }
    /* ---- pairing: comparación numérica (Just Works / Numeric Comparison) ---- */
    void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pin) override
    {
        ESP_LOGI(TAG, "BLE pairing: confirmando PIN mostrado en BMS = %06lu", (unsigned long)pin);
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }
    /* ---- pairing: resultado de autenticación ---- */
    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
    {
        if (connInfo.isEncrypted()) {
            ESP_LOGI(TAG, "BLE pairing: autenticación completada — enlace cifrado");
        } else {
            ESP_LOGW(TAG, "BLE pairing: autenticación completada — enlace SIN cifrar");
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
#ifdef CONFIG_BMS_TYPE_DALY
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
#endif /* CONFIG_BMS_TYPE_DALY */

//-------------------------------------------------------------------
#ifdef CONFIG_BMS_TYPE_JK
/* ---- enviar comando de lectura al JK BMS (trama de 20 bytes) ---- */
static void sendJkRequest(uint8_t cmd)
{
    if (!pRxCharacteristic || !connected) return;
    uint8_t frame[20] = {};
    frame[0] = 0xAA;  frame[1] = 0x55;
    frame[2] = 0x90;  frame[3] = 0xEB;
    frame[4] = cmd;   /* 0x96 = cell info, 0x97 = device info */
    /* bytes 5..18 = 0x00 (sin valor, solo lectura) */
    uint8_t crc = 0;
    for (int i = 0; i < 19; i++) crc += frame[i];
    frame[19] = crc;
    if (pRxCharacteristic->canWrite()) {
        pRxCharacteristic->writeValue(frame, 20, false);  /* sin ACK */
    }
}
#endif /* CONFIG_BMS_TYPE_JK */
//-------------------------------------------------------------------
static bool discoverAndSubscribe(void)
{
    if (!pClient || !pClient->isConnected()) { ESP_LOGE(TAG, "discoverAndSubscribe: cliente no conectado"); return false; }
    NimBLERemoteService *svc = pClient->getService(SERVICE_UUID_MAIN);
    if (!svc) { ESP_LOGE(TAG, "discoverAndSubscribe: servicio %s no encontrado", SERVICE_UUID_MAIN); return false; }

#ifdef CONFIG_BMS_TYPE_DALY
    pTxCharacteristic = svc->getCharacteristic(CHAR_UUID_TX);
    pRxCharacteristic = svc->getCharacteristic(CHAR_UUID_RX);
    if (!pTxCharacteristic || !pRxCharacteristic) { ESP_LOGE(TAG, "discoverAndSubscribe: características DALY no encontradas (TX=%p RX=%p)", pTxCharacteristic, pRxCharacteristic); return false; }
    if (pTxCharacteristic->canNotify()) {
        pTxCharacteristic->subscribe(true, notifyCB);
    }
#endif

#ifdef CONFIG_BMS_TYPE_JK
    /* Para JK BMS, TX y RX usan la misma UUID "0000ffe1".
     * En el dispositivo hay dos características con esa UUID:
     *   - instancia con Write  (Host→BMS)
     *   - instancia con Notify (BMS→Host)
     * Recorremos todas las características del servicio para
     * identificarlas por sus propiedades. */
    pRxCharacteristic = nullptr;
    pTxCharacteristic = nullptr;
    const auto &chars = svc->getCharacteristics(true);  /* true = forzar discovery GATT */
    ESP_LOGI(TAG, "discoverAndSubscribe: %d características encontradas en servicio ffe0", (int)chars.size());
    for (auto *c : chars) {
        ESP_LOGI(TAG, "  UUID=%s notify=%d write=%d",
                 c->getUUID().toString().c_str(),
                 (int)c->canNotify(), (int)c->canWrite());
        if (c->getUUID() == NimBLEUUID(CHAR_UUID_TX)) {
            if (c->canNotify() && !pTxCharacteristic) pTxCharacteristic = c;
            if (c->canWrite()  && !pRxCharacteristic) pRxCharacteristic = c;
        }
    }
    /* Fallback: si solo hay una characteristic, usarla para ambos roles */
    if (!pTxCharacteristic && pRxCharacteristic) pTxCharacteristic = pRxCharacteristic;
    if (!pRxCharacteristic && pTxCharacteristic) pRxCharacteristic = pTxCharacteristic;
    if (!pTxCharacteristic) { ESP_LOGE(TAG, "discoverAndSubscribe: característica JK (ffe1) no encontrada"); return false; }
    ESP_LOGI(TAG, "discoverAndSubscribe: TX=%s RX=%s",
             pTxCharacteristic ? pTxCharacteristic->getUUID().toString().c_str() : "null",
             pRxCharacteristic ? pRxCharacteristic->getUUID().toString().c_str() : "null");
    if (pTxCharacteristic->canNotify()) {
        pTxCharacteristic->subscribe(true, notifyCB);
    }
#endif

    return true;
}
//-------------------------------------------------------------------
static bool connectToBMS(void)
{
    if (!bmsInfo.found) return false;
    ESP_LOGI(TAG, "connectToBMS: intentando conectar a %s (RSSI=%d)",
             bmsInfo.address.toString().c_str(), bmsInfo.rssi);
    if (pClient) { NimBLEDevice::deleteClient(pClient); pClient = nullptr; }
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());
    pClient->setConnectTimeout(5000);
    if (!pClient->connect(bmsInfo.address)) {
        ESP_LOGE(TAG, "connectToBMS: fallo al conectar a %s", bmsInfo.address.toString().c_str());
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "connectToBMS: enlace BLE establecido, iniciando pairing/cifrado...");
    /* Forzar pairing antes de GATT discovery — el JK BMS protege las características */
    if (!pClient->secureConnection()) {
        ESP_LOGW(TAG, "connectToBMS: secureConnection falló o no requerido, continuando...");
    } else {
        ESP_LOGI(TAG, "connectToBMS: pairing/cifrado completado OK");
    }
    ESP_LOGI(TAG, "connectToBMS: descubriendo servicios...");
    if (!discoverAndSubscribe()) {
        ESP_LOGE(TAG, "connectToBMS: fallo en discoverAndSubscribe");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "connectToBMS: conectado y suscrito OK");
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

#ifdef CONFIG_BMS_TYPE_DALY
    cJSON_AddStringToObject(root, "bms_model", "DALY Smart BMS");
#else
    cJSON_AddStringToObject(root, "bms_model", "JK-PB2A16S-20P");
#endif

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
/* ================================================================== */
/* ---- SPIFFS, SNTP e hist\u00f3rico                                  ---- */
/* ================================================================== */

/* ---- Montar SPIFFS ---- */
static void spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = nullptr,
        .max_files              = 4,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount fall\u00f3: %s", esp_err_to_name(ret));
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info(nullptr, &total, &used);
        ESP_LOGI(TAG, "SPIFFS montado: %u KB total, %u KB usado",
                 (unsigned)(total / 1024), (unsigned)(used / 1024));
    }
}

/* ---- Iniciar SNTP (llamar tras tener IP) ---- */
static void sntp_init_fn(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP iniciado");
}

/* ---- Ruta del archivo binario de hist\u00f3rico seg\u00fan a\u00f1o ---- */
static void hist_file_path(int year, char *buf, size_t buf_len)
{
    snprintf(buf, buf_len, "/spiffs/hist_%d.bin", year);
}

/* ---- Borra archivos con a\u00f1o < current_year - 1 (conserva actual + anterior) ---- */
static void cleanup_old_hist_files(int current_year)
{
    char path[32];
    for (int y = current_year - 6; y < current_year - 1; y++) {
        hist_file_path(y, path, sizeof(path));
        remove(path);  /* silencioso si no existe */
    }
}

/* ---- Añade un registro al archivo binario del a\u00f1o correspondiente ---- */
static void append_hist_record(const HistRecord *rec)
{
    time_t t = (time_t)rec->ts;
    struct tm tm_info;
    localtime_r(&t, &tm_info);
    int year = tm_info.tm_year + 1900;
    char path[32];
    hist_file_path(year, path, sizeof(path));
    FILE *f = fopen(path, "ab");
    if (!f) {
        ESP_LOGW(TAG, "Hist: no se pudo abrir %s", path);
        return;
    }
    fwrite(rec, sizeof(HistRecord), 1, f);
    fclose(f);
    ESP_LOGD(TAG, "Hist: ts=%lu v=%.2f i=%.2f \u2192 %s",
             (unsigned long)rec->ts, rec->voltage, rec->current, path);
}

/* ---- Push de muestra histórica al servidor PHP ---- */
static void remote_push_hist(const HistRecord *rec)
{
    {
        xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
        bool has_ip = (s_bms.wifi_ip[0] != '\0');
        xSemaphoreGive(s_bms_mutex);
        if (!has_ip) return;
    }
    char body[96];
    snprintf(body, sizeof(body), "{\"ts\":%lu,\"v\":%.3f,\"i\":%.3f}",
             (unsigned long)rec->ts, rec->voltage, rec->current);

    esp_http_client_config_t cfg = {};
    cfg.url               = REMOTE_HIST_URL;
    cfg.method            = HTTP_METHOD_POST;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms        = 8000;
    /* Basic auth nativa: envía las credenciales de forma preemptiva sin esperar 401 */
    cfg.username          = REMOTE_HIST_USER;
    cfg.password          = REMOTE_HIST_PASS;
    cfg.auth_type         = HTTP_AUTH_TYPE_BASIC;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-API-Key",    REMOTE_API_KEY);
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Push hist remoto: HTTP %d",
                 esp_http_client_get_status_code(client));
    } else {
        ESP_LOGW(TAG, "Push hist remoto falló: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

/* ---- Tarea: agrega muestra cada 5 min y escribe en SPIFFS ---- */
static void hist_task(void *pvParameters)
{
    /* Esperar sync SNTP (m\u00e1x ~5 min antes de empezar) */
    for (;;) {
        if (time(nullptr) > (time_t)SNTP_EPOCH_MIN) break;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI(TAG, "SNTP sincronizado \u2014 hist\u00f3rico activado");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)HIST_INTERVAL_S * 1000));

        time_t now = time(nullptr);
        if (now <= (time_t)SNTP_EPOCH_MIN) continue;

        /* Leer y resetear acumuladores */
        float volt_avg = 0.0f, curr_avg = 0.0f;
        int   cnt      = 0;
        xSemaphoreTake(s_hist_mutex, portMAX_DELAY);
        if (s_sample_cnt > 0) {
            volt_avg = s_volt_sum / (float)s_sample_cnt;
            curr_avg = s_curr_sum / (float)s_sample_cnt;
            cnt      = s_sample_cnt;
        }
        s_volt_sum   = 0.0f;
        s_curr_sum   = 0.0f;
        s_sample_cnt = 0;
        xSemaphoreGive(s_hist_mutex);

        if (cnt == 0) continue;  /* sin lecturas BLE en este intervalo */

        HistRecord rec;
        rec.ts      = (uint32_t)(now - HIST_INTERVAL_S);  /* inicio del intervalo */
        rec.voltage = volt_avg;
        rec.current = curr_avg;

        append_hist_record(&rec);
        remote_push_hist(&rec);

        struct tm tm_now;
        localtime_r(&now, &tm_now);
        cleanup_old_hist_files(tm_now.tm_year + 1900);
    }
}

/* ================================================================== */
/* ---- Endpoint GET /history                                     ---- */
/* ================================================================== */

struct HistBucket {
    float v_sum;
    float i_sum;
    int   count;
};

static esp_err_t history_handler(httpd_req_t *req)
{
    char view[8]   = "day";
    char ts_str[14] = "0";

    char qbuf[48] = {};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        httpd_query_key_value(qbuf, "view", view,   sizeof(view));
        httpd_query_key_value(qbuf, "ts",   ts_str, sizeof(ts_str));
    }

    uint32_t ts_start = (uint32_t)strtoul(ts_str, nullptr, 10);
    if (ts_start == 0) {
        /* Por defecto: inicio del día actual (UTC).
         * timegm() no está disponible en newlib; calculamos el epoch UTC manualmente
         * restando los segundos del día actual de time(). */
        time_t now = time(nullptr);
        struct tm tm_now;
        gmtime_r(&now, &tm_now);
        uint32_t secs_today = (uint32_t)tm_now.tm_hour * 3600u
                            + (uint32_t)tm_now.tm_min  * 60u
                            + (uint32_t)tm_now.tm_sec;
        ts_start = (uint32_t)now - secs_today;
    }

    /* Configuraci\u00f3n de la vista */
    uint32_t period_s, bucket_s;
    int      n_buckets;
    if (strcmp(view, "week") == 0) {
        bucket_s  = 3600;    n_buckets = 168; period_s = 7u * 86400u;
    } else if (strcmp(view, "month") == 0) {
        bucket_s  = 86400;   n_buckets = 31;  period_s = 31u * 86400u;
    } else if (strcmp(view, "year") == 0) {
        bucket_s  = 2635200;  n_buckets = 12;  period_s = 366u * 86400u;
    } else {   /* day */
        bucket_s  = 300;     n_buckets = 288; period_s = 86400u;
    }

    uint32_t ts_end = ts_start + period_s;

    HistBucket *buckets = (HistBucket *)calloc(n_buckets, sizeof(HistBucket));
    if (!buckets) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    /* Determinar a\u00f1os que cubre el rango */
    struct tm tm1, tm2;
    time_t t1 = (time_t)ts_start, t2 = (time_t)(ts_end - 1);
    gmtime_r(&t1, &tm1);
    gmtime_r(&t2, &tm2);
    int year1 = tm1.tm_year + 1900;
    int year2 = tm2.tm_year + 1900;

    for (int year = year1; year <= year2; year++) {
        char path[32];
        hist_file_path(year, path, sizeof(path));
        FILE *f = fopen(path, "rb");
        if (!f) continue;
        HistRecord recs[64];
        size_t n;
        while ((n = fread(recs, sizeof(HistRecord), 64, f)) > 0) {
            for (size_t k = 0; k < n; k++) {
                uint32_t rts = recs[k].ts;
                if (rts < ts_start || rts >= ts_end) continue;
                int bidx = (int)((rts - ts_start) / bucket_s);
                if (bidx < 0 || bidx >= n_buckets) continue;
                buckets[bidx].v_sum  += recs[k].voltage;
                buckets[bidx].i_sum  += recs[k].current;
                buckets[bidx].count++;
            }
        }
        fclose(f);
    }

    /* Respuesta JSON en chunks */
    httpd_resp_set_type(req, "application/json");
    char hdr[128];
    int hlen = snprintf(hdr, sizeof(hdr),
        "{\"view\":\"%s\",\"ts_start\":%lu,\"bucket_s\":%lu,\"buckets\":[",
        view, (unsigned long)ts_start, (unsigned long)bucket_s);
    httpd_resp_send_chunk(req, hdr, hlen);

    bool first = true;
    for (int b = 0; b < n_buckets; b++) {
        if (buckets[b].count == 0) continue;
        float avg_v = buckets[b].v_sum / (float)buckets[b].count;
        float avg_i = buckets[b].i_sum / (float)buckets[b].count;
        float avg_p = avg_v * avg_i;
        uint32_t bts = ts_start + (uint32_t)b * bucket_s;
        char entry[120];
        int elen = snprintf(entry, sizeof(entry),
            "%s{\"ts\":%lu,\"v\":%.2f,\"i\":%.2f,\"p\":%.1f}",
            first ? "" : ",",
            (unsigned long)bts, avg_v, avg_i, avg_p);
        httpd_resp_send_chunk(req, entry, elen);
        first = false;
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, nullptr, 0);

    free(buckets);
    return ESP_OK;
}

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

        /* Acumular muestra para histórico (1 Hz, siempre activo) */
        if (s_hist_mutex) {
            xSemaphoreTake(s_bms_mutex, portMAX_DELAY);
            bool  valid  = (s_bms.valid & (1u << 0)) != 0;
            float v_now  = s_bms.voltage;
            float i_now  = s_bms.current;
            xSemaphoreGive(s_bms_mutex);
            if (valid) {
                xSemaphoreTake(s_hist_mutex, portMAX_DELAY);
                s_volt_sum   += v_now;
                s_curr_sum   += i_now;
                s_sample_cnt++;
                xSemaphoreGive(s_hist_mutex);
            }
        }

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
#ifdef CONFIG_BMS_TYPE_DALY
            cJSON *code = cJSON_GetObjectItem(json, "code");
            if (code && cJSON_IsNumber(code))
                sendDalyRequest((uint8_t)code->valueint);
#endif
#ifdef CONFIG_BMS_TYPE_JK
            sendJkRequest(JK_CMD_CELL_INFO);
#endif
        } else if (strcmp(cs, "set_charge_mos") == 0) {
#ifdef CONFIG_BMS_TYPE_DALY
            /* { "cmd": "set_charge_mos", "enable": true|false } */
            cJSON *en = cJSON_GetObjectItem(json, "enable");
            if (en && cJSON_IsBool(en)) {
                uint8_t payload[8] = {};
                payload[0] = cJSON_IsTrue(en) ? 0x01 : 0x00;
                sendDalyRequest(0x44, payload);
                ESP_LOGI(TAG, "WS → set_charge_mos %s", cJSON_IsTrue(en) ? "ON" : "OFF");
            }
#endif
        } else if (strcmp(cs, "set_discharge_mos") == 0) {
#ifdef CONFIG_BMS_TYPE_DALY
            /* { "cmd": "set_discharge_mos", "enable": true|false } */
            cJSON *en = cJSON_GetObjectItem(json, "enable");
            if (en && cJSON_IsBool(en)) {
                uint8_t payload[8] = {};
                payload[0] = cJSON_IsTrue(en) ? 0x01 : 0x00;
                sendDalyRequest(0x45, payload);
                ESP_LOGI(TAG, "WS → set_discharge_mos %s", cJSON_IsTrue(en) ? "ON" : "OFF");
            }
#endif
        } else if (strcmp(cs, "reset_soc") == 0) {
#ifdef CONFIG_BMS_TYPE_DALY
            /* { "cmd": "reset_soc" }  — resetea el SOC al 100% */
            sendDalyRequest(0x00);
            ESP_LOGI(TAG, "WS → reset_soc enviado");
#endif
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
    cfg.max_open_sockets = WS_MAX_CLIENTS + 4;  /* +4: WS clients + HTTP histórico + margen */

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

    static const httpd_uri_t history_uri = {
        .uri                    = "/history",
        .method                 = HTTP_GET,
        .handler                = history_handler,
        .user_ctx               = nullptr,
        .is_websocket           = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol  = nullptr,
    };
    httpd_register_uri_handler(s_httpd, &history_uri);

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
"<title>BMS Monitor- WiFi Setup</title>"
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
#ifdef CONFIG_BMS_TYPE_DALY
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
#endif /* CONFIG_BMS_TYPE_DALY */

//-------------------------------------------------------------------
#ifdef CONFIG_BMS_TYPE_JK
/* ---- tarea principal BLE para JK BMS ---- */
static void jk_ble_task(void *pvParameters)
{
    NimBLEDevice::init("ESP32-JK-BLE");
    /* Configurar seguridad BLE para pairing con PIN (JK BMS puede requerirlo) */
    NimBLEDevice::setSecurityAuth(true, true, true);          /* bonding, MITM, SC */
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); /* el ESP32 introduce el PIN */
    NimBLEDevice::setSecurityPasskey(BMS_BLE_PASSKEY);
    ESP_LOGI(TAG, "Escaneando JK BMS en %s ...", BMS_MAC_ADDRESS);
    scanForBMS(8);

    if (bmsInfo.found && connectToBMS()) {
        connected = true;
        delay(500);
        /* Solicitar info de dispositivo y, a continuación, cell info */
        sendJkRequest(JK_CMD_DEV_INFO);
        delay(600);
        sendJkRequest(JK_CMD_CELL_INFO);
    } else {
        if (!bmsInfo.found)
            ESP_LOGW(TAG, "JK BMS no detectado en el escaneo inicial");
        else
            ESP_LOGW(TAG, "JK BMS detectado pero fallo al conectar en el escaneo inicial");
    }

    static uint64_t lastPoll      = 0;
    static uint64_t lastReconnect = 0;

    for (;;) {
        if (connected && pClient && pClient->isConnected()) {
            if (millis() - lastPoll > 2000) {
                sendJkRequest(JK_CMD_CELL_INFO);
                lastPoll = millis();
            }
        } else {
            if (s_trigger_scan || (millis() - lastReconnect > 3000)) {
                s_trigger_scan = false;
                cleanupClient();
                ESP_LOGI(TAG, "Intentando reconexión JK...");
                scanForBMS(6);
                if (bmsInfo.found) connectToBMS();
                lastReconnect = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif /* CONFIG_BMS_TYPE_JK */
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

    /* Mutex para acumuladores históricos */
    s_hist_mutex = xSemaphoreCreateMutex();
    configASSERT(s_hist_mutex);

    /* SPIFFS — almacén de datos históricos (antes de WiFi) */
    spiffs_init();

    /* Reducir verbosidad de NimBLE (oculta cada write/procedure GATT) */
    esp_log_level_set("NimBLE",      ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx",  ESP_LOG_ERROR);  /* silencia EAGAIN/ECONNRESET en WS */
    esp_log_level_set("httpd_ws",    ESP_LOG_ERROR);  /* silencia WS header/payload send fail */

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

    /* WiFi ok: iniciar SNTP para sincronización horaria */
    sntp_init_fn();

    /* Modo normal: HTTP server BMS + mDNS + BLE + remote push */
    http_server_init();

    /* mDNS: accesible como http://bms.local en la red local */
    mdns_init();
#ifdef CONFIG_BMS_TYPE_DALY
    mdns_hostname_set("bms");
    mdns_instance_name_set("Daly BMS");
#else
    mdns_hostname_set("bms");
    mdns_instance_name_set("JK BMS");
#endif
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    ESP_LOGI(TAG, "mDNS iniciado: http://bms.local/");

#ifdef CONFIG_BMS_TYPE_DALY
    xTaskCreate(daly_ble_task,    "daly_ble",    12288, nullptr, 5, nullptr);
#endif
#ifdef CONFIG_BMS_TYPE_JK
    xTaskCreate(jk_ble_task,      "jk_ble",      12288, nullptr, 5, nullptr);
#endif
    xTaskCreate(remote_push_task, "remote_push",  8192, nullptr, 3, nullptr);
    xTaskCreate(hist_task,        "hist",          4096, nullptr, 2, nullptr);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//-------------------------------------------------------------------